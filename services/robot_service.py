"""
Robot Control Service — 机器人控制服务层
=======================================
提供运动学、轨迹规划、仿真的 HTTP API，供 Agent 调用。
基于 inexbot-forward 的正向运动学和轨迹规划算法。

API:
  POST /ik                 逆运动学求解 (位姿 → 关节角)
  POST /fk                 正运动学求解 (关节角 → 位姿)
  POST /plan               轨迹规划 (起点→终点, PTP/LIN)
  POST /execute            执行完整动作序列
  POST /simulate           在仿真中执行动作
  GET  /health             健康检查
  GET  /status             机器人当前状态
"""
import json
import math
import logging
import time
from typing import Optional
from dataclasses import dataclass, field

import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("robot_service")

app = Flask(__name__)
CORS(app)


# ══════════════════════════════════════════════════════════════
# 机器人模型 (独立实现，不依赖 inexbot-forward)
# ══════════════════════════════════════════════════════════════

@dataclass
class DHParam:
    a: float = 0.0
    alpha: float = 0.0
    d: float = 0.0
    theta: float = 0.0
    offset: float = 0.0


@dataclass
class RobotState:
    joints: list[float] = field(default_factory=lambda: [0.0] * 6)
    pose: list[float] = field(default_factory=lambda: [0.0] * 6)
    gripper: float = 0.0  # 0=关, 1=开


class SixAxisRobot:
    """六轴串联机器人运动学模型 (DH 参数法)"""

    # FANUC LR Mate 200iD 关节限位 (度)
    JOINT_LIMITS_DEG = [
        (-170, 170),    # J1
        (-150, 145),    # J2 (FANUC spec: -150 to +145)
        (-130, 250),    # J3
        (-190, 190),    # J4
        (-130, 130),    # J5
        (-360, 360),    # J6
    ]
    # 安全工作限位
    SAFE_LIMITS_DEG = [
        (-165, 165),
        (-145, 140),    # J2: 放宽至 FANUC 真实限位
        (-125, 245),
        (-185, 185),
        (-125, 125),
        (-360, 360),
    ]

    def __init__(self, dh_params: list[DHParam] = None):
        self.dh = dh_params or self._default_dh()
        self.state = RobotState()
        self.max_vel = [math.radians(180)] * 6
        self.max_acc = [math.radians(360)] * 6
        self.joint_limits_rad = [
            (math.radians(lo), math.radians(hi)) for lo, hi in self.SAFE_LIMITS_DEG
        ]

    @staticmethod
    def _default_dh() -> list[DHParam]:
        """近似 DH 参数 (Z-up, 单位mm)，匹配约700mm前伸工作范围"""
        return [
            DHParam(a=0, alpha=-math.pi / 2, d=330, offset=0),           # J1: 基座旋转
            DHParam(a=400, alpha=0, d=0, offset=-math.pi / 2),           # J2: 肩部 (a2≈400mm)
            DHParam(a=50, alpha=-math.pi / 2, d=0, offset=math.pi / 2),  # J3: 肘部
            DHParam(a=0, alpha=math.pi / 2, d=350, offset=0),            # J4: 腕部
            DHParam(a=0, alpha=-math.pi / 2, d=0, offset=0),             # J5: 腕部俯仰
            DHParam(a=0, alpha=0, d=80, offset=0),                       # J6: 法兰
        ]

    # ════════════════════════════════════════════════════════════
    # URDF 正向运动学 (匹配 app.js applyJointsToRobot 的层级)
    # ════════════════════════════════════════════════════════════

    # URDF joint origins in robotAssembly Z-up frame (mm)
    URDF_ORIGINS = [
        [0, 0, 42.741],      # J1
        [50, 0, 287.26],     # J2
        [0, 0, 330],         # J3
        [88.001, 0, 35.027], # J4
        [245.4, 0, 0],       # J5
        [50, 0, 0],          # J6
    ]

    # URDF joint axes in robotAssembly Z-up frame
    URDF_AXES = [
        [0, 0, 1],    # J1: +Z
        [0, -1, 0],   # J2: -Y
        [0, -1, 0],   # J3: -Y
        [-1, 0, 0],   # J4: -X
        [0, -1, 0],   # J5: -Y
        [-1, 0, 0],   # J6: -X
    ]

    # robotGroup position in Three.js world
    ROBOT_GROUP_X = -90
    # robotAssembly Y-position in robotGroup (pedestal top)
    ROBOT_ASSEMBLY_Y = 340
    # TCP offset from J6 pivot along Z in URDF frame
    TCP_OFFSET_Z = 85

    @staticmethod
    def _axis_angle_matrix(axis, angle):
        """Rodrigues formula: unit axis + angle → 3x3 rotation matrix"""
        ax, ay, az = axis
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array([
            [c + ax*ax*(1-c), ax*ay*(1-c) - az*s, ax*az*(1-c) + ay*s],
            [ay*ax*(1-c) + az*s, c + ay*ay*(1-c), ay*az*(1-c) - ax*s],
            [az*ax*(1-c) - ay*s, az*ay*(1-c) + ax*s, c + az*az*(1-c)],
        ])

    def urdf_forward_kinematics(self, joints):
        """
        URDF正运动学: 关节角 → TCP在robotAssembly Z-up坐标系中的4x4位姿矩阵。
        完全匹配 app.js applyJointsToRobot 的嵌套层级变换。
        """
        T = np.eye(4)
        for i in range(6):
            t = self.URDF_ORIGINS[i]
            T_trans = np.array([
                [1, 0, 0, t[0]],
                [0, 1, 0, t[1]],
                [0, 0, 1, t[2]],
                [0, 0, 0, 1],
            ])
            R = np.eye(4)
            R[:3, :3] = self._axis_angle_matrix(self.URDF_AXES[i], joints[i])

            if i == 5:  # J6 fixed rpy(π/2, 0, 0)
                crx = math.cos(math.pi / 2)
                srx = math.sin(math.pi / 2)
                R_fixed = np.array([
                    [1, 0, 0, 0],
                    [0, crx, -srx, 0],
                    [0, srx, crx, 0],
                    [0, 0, 0, 1],
                ])
                T = T @ T_trans @ R @ R_fixed
            else:
                T = T @ T_trans @ R

        tcp_local = np.array([0, 0, self.TCP_OFFSET_Z, 1])
        return T @ tcp_local  # [ux, uy, uz, 1] in robotAssembly frame

    def urdf_fk_world(self, joints):
        """
        URDF FK → TCP位置 (视觉约定世界坐标).
        返回: np.array([X_fwd, Y_lr, Z_up]) 单位mm

        转换链:
          robotAssembly(Z-up) → rotate Rx(-π/2) → robotGroup(Y-up) → translate → world

        视觉约定: X=前后(Three.js X), Y=左右(Three.js Z), Z=上下(Three.js Y)
        """
        tcp_u = self.urdf_forward_kinematics(joints)
        ux, uy, uz = tcp_u[0], tcp_u[1], tcp_u[2]

        # robotAssembly.rotation.set(-π/2, 0, 0):
        #   URDF X → Three.js X, URDF Z → Three.js Y, URDF Y → Three.js -Z
        # + robotAssembly.position.y = 340
        three_x = ux
        three_y = uz + self.ROBOT_ASSEMBLY_Y
        three_z = -uy

        # + robotGroup.position = (-90, 0, 0)
        three_x += self.ROBOT_GROUP_X

        # Three.js → visual: X_fwd=X, Y_lr=Z, Z_up=Y
        return np.array([three_x, three_z, three_y])

    def urdf_fk_pose(self, joints):
        """
        URDF FK → 完整6-DOF位姿 (视觉约定世界坐标).
        返回: {"x","y","z","rx","ry","rz"} 位置mm/角度deg
        """
        T = np.eye(4)
        for i in range(6):
            t = self.URDF_ORIGINS[i]
            T_trans = np.array([
                [1, 0, 0, t[0]],
                [0, 1, 0, t[1]],
                [0, 0, 1, t[2]],
                [0, 0, 0, 1],
            ])
            R = np.eye(4)
            R[:3, :3] = self._axis_angle_matrix(self.URDF_AXES[i], joints[i])

            if i == 5:
                crx = math.cos(math.pi / 2)
                srx = math.sin(math.pi / 2)
                R_fixed = np.array([
                    [1, 0, 0, 0],
                    [0, crx, -srx, 0],
                    [0, srx, crx, 0],
                    [0, 0, 0, 1],
                ])
                T = T @ T_trans @ R @ R_fixed
            else:
                T = T @ T_trans @ R

        # TCP position in robotAssembly
        tcp_u = T @ np.array([0, 0, self.TCP_OFFSET_Z, 1])

        # Convert to visual world position
        wx = tcp_u[0] + self.ROBOT_GROUP_X
        wy = -tcp_u[1]  # Three.js Z = -URDF Y
        wz = tcp_u[2] + self.ROBOT_ASSEMBLY_Y

        # Convert orientation: robotAssembly rotation Rx(-π/2)
        R_urdf = T[:3, :3]
        # Apply robotAssembly rotation: R_world = Rx(-π/2) @ R_urdf
        R_conv = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0],
        ])
        R_world = R_conv @ R_urdf

        # Extract Euler angles from R_world
        sy = math.sqrt(R_world[0, 0]**2 + R_world[1, 0]**2)
        if sy > 1e-6:
            rx = math.atan2(R_world[2, 1], R_world[2, 2])
            ry = math.atan2(-R_world[2, 0], sy)
            rz = math.atan2(R_world[1, 0], R_world[0, 0])
        else:
            rx = math.atan2(-R_world[1, 2], R_world[1, 1])
            ry = math.atan2(-R_world[2, 0], sy)
            rz = 0

        return {
            "x": round(wx, 3), "y": round(wy, 3), "z": round(wz, 3),
            "rx": round(math.degrees(rx), 2),
            "ry": round(math.degrees(ry), 2),
            "rz": round(math.degrees(rz), 2),
        }

    def urdf_inverse_kinematics_world(self, target_wx, target_wy, target_wz,
                                       seed_joints=None):
        """
        URDF逆运动学 (位置3-DOF): 世界坐标 → 关节角.
        使用数值雅可比 + 阻尼最小二乘法。
        """
        if seed_joints is None:
            q = np.array(self.state.joints, dtype=float)
        else:
            q = np.array(seed_joints, dtype=float)

        target = np.array([target_wx, target_wy, target_wz], dtype=float)

        alpha = 0.3       # 步长
        damping = 0.05    # 阻尼因子
        max_iter = 600
        tol = 0.5         # mm 收敛容差

        for iteration in range(max_iter):
            fk_pos = self.urdf_fk_world(q.tolist())
            error = target - fk_pos
            err_norm = float(np.linalg.norm(error))

            if err_norm < tol:
                result = [math.atan2(math.sin(v), math.cos(v)) for v in q]
                return [round(float(v), 4) for v in result]

            # 3x6 数值雅可比
            delta = 0.005  # rad (~0.3°)
            J = np.zeros((3, 6))
            for i in range(6):
                q_plus = q.copy()
                q_plus[i] += delta
                fk_plus = self.urdf_fk_world(q_plus.tolist())
                J[:, i] = (fk_plus - fk_pos) / delta

            # DLS: dq = J^T (J J^T + λ² I)^{-1} error
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + damping**2 * np.eye(3), error) * alpha
            dq = np.clip(dq, -0.3, 0.3)
            q += dq

        logger.warning("URDF IK did not converge (final err=%.1f mm)", err_norm)
        return None

    def urdf_ik_robust(self, target_wx, target_wy, target_wz):
        """
        鲁棒URDF IK: 从多个候选初始构型求解，选择最优。
        不修改 self.state.joints。
        """
        saved_joints = list(self.state.joints)
        deg = math.radians

        # URDF candidates: J2 negative for forward reach, J3 near zero for straight forearm
        candidates = [
            saved_joints,
            [0.0, deg(-90), deg(0), 0.0, deg(0), 0.0],
            [0.0, deg(-85), deg(0), 0.0, deg(0), 0.0],
            [0.0, deg(-95), deg(-10), 0.0, deg(0), 0.0],
            [0.0, deg(-100), deg(-20), 0.0, deg(10), 0.0],
            [0.0, deg(-80), deg(10), 0.0, deg(-10), 0.0],
            [0.0, deg(-70), deg(20), 0.0, deg(-20), 0.0],
            [0.0, deg(-105), deg(-30), 0.0, deg(20), 0.0],
            [deg(15), deg(-85), deg(0), 0.0, deg(0), 0.0],
            [deg(-15), deg(-85), deg(0), 0.0, deg(0), 0.0],
            [deg(30), deg(-90), deg(-10), 0.0, deg(10), 0.0],
            [deg(-30), deg(-90), deg(-10), 0.0, deg(10), 0.0],
            [0.0, deg(-90), deg(30), 0.0, deg(-30), 0.0],
            [0.0, deg(-90), deg(-40), 0.0, deg(30), 0.0],
        ]

        best_joints = None
        best_score = float('inf')

        for i, q0 in enumerate(candidates):
            result = self.urdf_inverse_kinematics_world(
                target_wx, target_wy, target_wz, seed_joints=q0)
            if result is None:
                continue
            ok, violations = self.check_joint_limits(result)
            if not ok:
                continue
            score = self.score_configuration(result)
            if score < best_score:
                best_score = score
                best_joints = result
                logger.info("URDF IK candidate %d: score=%.1f, joints=%s",
                           i, score, [f'{math.degrees(j):.0f}' for j in result])

        if best_joints:
            logger.info("URDF IK selected best (score=%.1f)", best_score)
            return best_joints

        # Fallback: first success regardless of limits
        for i, q0 in enumerate(candidates):
            result = self.urdf_inverse_kinematics_world(
                target_wx, target_wy, target_wz, seed_joints=q0)
            if result is not None:
                logger.warning("URDF IK fallback candidate %d (limits bypassed)", i)
                return result

        return None

    def check_joint_limits(self, joints: list[float]) -> tuple[bool, list[str]]:
        """检查关节角是否在安全限位内。返回 (是否合规, 违规列表)"""
        violations = []
        for i, (j, (lo, hi)) in enumerate(zip(joints, self.joint_limits_rad)):
            if j < lo or j > hi:
                violations.append(
                    f"J{i+1}={math.degrees(j):.1f}° 超出限位 [{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]")
        return len(violations) == 0, violations

    def score_configuration(self, joints: list[float]) -> float:
        """对URDF关节构型评分 (越低越好).
        偏好: J2≈ -90°(前伸), J3≈0°(前臂顺直), 避免极限. """
        score = 0.0
        j2_deg = math.degrees(joints[1])
        j3_deg = math.degrees(joints[2])

        # URDF: J2 ∈ [-110°, -70°] 手臂前伸 (负值, 对应DH的+80°~+100°)
        if -110 <= j2_deg <= -70:
            score += abs(j2_deg + 90) * 0.05
        elif -125 <= j2_deg <= -55:
            score += abs(j2_deg + 90) * 0.5
        else:
            score += abs(j2_deg + 90) * 5.0

        # URDF: J3 ∈ [-30°, 30°] 前臂顺直延伸 (不同于DH的大弯折)
        if -30 <= j3_deg <= 30:
            score += abs(j3_deg) * 0.05
        elif -50 <= j3_deg <= 50:
            score += abs(j3_deg) * 0.5
        else:
            score += abs(j3_deg) * 3.0

        for i in [0, 4, 5]:
            score += abs(math.degrees(joints[i])) * 0.02
        j4_deg = math.degrees(joints[3])
        if abs(j4_deg) > 130:
            score += (abs(j4_deg) - 130) ** 2 * 0.05
        return score

    def dh_transform(self, a, alpha, d, theta) -> np.ndarray:
        """单个 DH 参数的齐次变换矩阵"""
        return np.array([
            [math.cos(theta), -math.sin(theta) * math.cos(alpha),
             math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
            [math.sin(theta), math.cos(theta) * math.cos(alpha),
             -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ])

    def forward_kinematics(self, joints: list[float]) -> np.ndarray:
        """正运动学: 关节角 → 末端位姿 (4x4 齐次矩阵)"""
        T = np.eye(4)
        for i, j in enumerate(joints):
            dh = self.dh[i]
            T = T @ self.dh_transform(dh.a, dh.alpha, dh.d,
                                      j + dh.theta + dh.offset)
        return T

    def pose_from_matrix(self, T: np.ndarray) -> dict:
        """从齐次矩阵提取位姿 (x, y, z, rx, ry, rz)"""
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        sy = math.sqrt(T[0, 0] ** 2 + T[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            rx = math.atan2(T[2, 1], T[2, 2])
            ry = math.atan2(-T[2, 0], sy)
            rz = math.atan2(T[1, 0], T[0, 0])
        else:
            rx = math.atan2(-T[1, 2], T[1, 1])
            ry = math.atan2(-T[2, 0], sy)
            rz = 0
        return {
            "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "rx": round(math.degrees(rx), 2), "ry": round(math.degrees(ry), 2),
            "rz": round(math.degrees(rz), 2),
            "matrix_4x4": T.round(6).tolist(),
        }

    def inverse_kinematics(self, target_pose: dict) -> list[float] | None:
        """
        逆运动学: 位姿 → 关节角。
        使用阻尼最小二乘法 (Damped Least Squares), 对任意构型通用。
        """
        target = np.array([
            target_pose.get("x", 0) * 0.001,   # mm → m 缩放
            target_pose.get("y", 0) * 0.001,
            target_pose.get("z", 0) * 0.001,
            math.radians(target_pose.get("rx", 0)),
            math.radians(target_pose.get("ry", 0)),
            math.radians(target_pose.get("rz", 0)),
        ])
        q = np.array(self.state.joints, dtype=float)
        alpha = 0.5      # 步长
        damping = 0.1    # 阻尼因子 (防奇异)
        max_iter = 500
        tol = 0.01       # 收敛容差 (米/弧度归一化后)

        for iteration in range(max_iter):
            T = self.forward_kinematics(q.tolist())
            current_pose = self.pose_from_matrix(T)
            current = np.array([
                current_pose["x"] * 0.001,
                current_pose["y"] * 0.001,
                current_pose["z"] * 0.001,
                math.radians(current_pose["rx"]),
                math.radians(current_pose["ry"]),
                math.radians(current_pose["rz"]),
            ])
            error = target - current
            err_norm = np.linalg.norm(error)
            if err_norm < tol:
                logger.info("IK converged in %d iterations (err=%.6f)", iteration + 1, err_norm)
                # Normalize to [-π, π] to avoid multi-turn spinning during animation
                result = [math.atan2(math.sin(v), math.cos(v)) for v in q]
                return [round(float(v), 4) for v in result]

            J = self._jacobian_numeric(q)
            J[:3, :] *= 0.001  # 位置部分缩放

            # 阻尼最小二乘: dq = J^T (J J^T + λ^2 I)^{-1} error
            JJT = J @ J.T
            I_6 = np.eye(6)
            dq = J.T @ np.linalg.solve(JJT + damping ** 2 * I_6, error) * alpha
            dq = np.clip(dq, -0.5, 0.5)
            q += dq

        logger.warning("IK did not converge from current config (final err=%.4f)", err_norm)
        return None

    def inverse_kinematics_robust(self, target_pose: dict) -> list[float] | None:
        """
        鲁棒 IK: 从多个初始位姿求解，选择最优构型。
        偏好: 手臂前伸 (J2≈90°), 避免后倾自碰撞, 关节角在限位内。
        不会修改 self.state.joints（调用前后状态一致）。
        """
        saved_joints = list(self.state.joints)

        # 候选初始构型: 手臂前伸 (J2≈90°) + 不同肘部弯折
        deg = math.radians
        candidates = [
            # 当前状态
            saved_joints,
            # 标准前伸构型 (J2≈90° = 水平前伸)
            [0.0, deg(90), deg(-90), 0.0, deg(0), 0.0],
            # 前伸-水平: 手臂平伸
            [0.0, deg(85), deg(-90), 0.0, deg(0), 0.0],
            # 前伸-略低: 手臂稍向下
            [0.0, deg(95), deg(-100), 0.0, deg(0), 0.0],
            # 前伸-更低: 手臂向下
            [0.0, deg(100), deg(-110), 0.0, deg(10), 0.0],
            # 前伸-高: 手臂略向上
            [0.0, deg(80), deg(-70), 0.0, deg(-10), 0.0],
            # 近端构型 (目标靠近基座)
            [0.0, deg(70), deg(-40), 0.0, deg(-20), 0.0],
            # 远端构型 (目标远离基座)
            [0.0, deg(100), deg(-120), 0.0, deg(20), 0.0],
            # 带J1偏移的构型
            [deg(15), deg(85), deg(-90), 0.0, deg(0), 0.0],
            [deg(-15), deg(85), deg(-90), 0.0, deg(0), 0.0],
            [deg(30), deg(90), deg(-100), 0.0, deg(10), 0.0],
            [deg(-30), deg(90), deg(-100), 0.0, deg(10), 0.0],
            # 手肘弯折更小/更大的变体
            [0.0, deg(90), deg(-60), 0.0, deg(-30), 0.0],
            [0.0, deg(90), deg(-130), 0.0, deg(30), 0.0],
        ]

        best_joints = None
        best_score = float('inf')

        for i, q0 in enumerate(candidates):
            self.state.joints = list(q0)
            result = self.inverse_kinematics(target_pose)
            if result is None:
                continue
            # 检查限位
            ok, violations = self.check_joint_limits(result)
            if not ok:
                continue
            score = self.score_configuration(result)
            if score < best_score:
                best_score = score
                best_joints = result
                logger.info("Robust IK candidate %d: score=%.1f, joints=%s",
                           i, score, [f'{math.degrees(j):.0f}' for j in result])

        self.state.joints = saved_joints  # 恢复原始状态

        if best_joints:
            logger.info("Robust IK selected best candidate (score=%.1f)", best_score)
            return best_joints

        # 回退: 不限位的第一个成功解
        for i, q0 in enumerate(candidates):
            self.state.joints = list(q0)
            result = self.inverse_kinematics(target_pose)
            if result is not None:
                self.state.joints = saved_joints
                logger.warning("Robust IK using fallback candidate %d (limits bypassed)", i)
                return result

        self.state.joints = saved_joints
        logger.warning("Robust IK failed from all %d candidate configurations", len(candidates))
        return None

    def _jacobian_numeric(self, q: np.ndarray, delta=1e-3) -> np.ndarray:
        """数值雅可比 (6x6)"""
        J = np.zeros((6, 6))
        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += delta
            T_plus = self.forward_kinematics(q_plus.tolist())
            p_plus = self.pose_from_matrix(T_plus)
            T_minus = self.forward_kinematics(q.tolist())
            p_minus = self.pose_from_matrix(T_minus)
            J[0, i] = (p_plus["x"] - p_minus["x"]) / delta
            J[1, i] = (p_plus["y"] - p_minus["y"]) / delta
            J[2, i] = (p_plus["z"] - p_minus["z"]) / delta
            J[3, i] = (math.radians(p_plus["rx"]) - math.radians(p_minus["rx"])) / delta
            J[4, i] = (math.radians(p_plus["ry"]) - math.radians(p_minus["ry"])) / delta
            J[5, i] = (math.radians(p_plus["rz"]) - math.radians(p_minus["rz"])) / delta
        return J

    def plan_ptp(self, target_joints: list[float], steps: int = 50) -> list[dict]:
        """
        PTP (点到点) 轨迹规划 — 关节空间梯形速度插补。
        返回关节角序列 + 对应的末端位姿序列。
        """
        start = np.array(self.state.joints)
        end = np.array(target_joints)

        # 梯形速度曲线
        t = np.linspace(0, 1, steps)
        # 加速-匀速-减速
        ramp_up = 0.3
        ramp_down = 0.7
        profile = np.piecewise(t,
                               [t < ramp_up, (t >= ramp_up) & (t <= ramp_down), t > ramp_down],
                               [lambda x: x ** 2 / (2 * ramp_up * (1 - ramp_up)),
                                lambda x: (x - ramp_up / 2),
                                lambda x: 1 - (1 - x) ** 2 / (2 * ramp_down * (1 - ramp_down))])

        trajectory = []
        for i in range(steps):
            frac = profile[i] if i < len(profile) else 1.0
            q = start + frac * (end - start)
            T = self.forward_kinematics(q.tolist())
            pose = self.pose_from_matrix(T)
            trajectory.append({
                "step": i,
                "progress": round(frac, 3),
                "joints": [round(float(v), 4) for v in q],
                "pose": pose,
            })

        # 更新状态
        self.state.joints = [round(float(v), 4) for v in end]
        final_T = self.forward_kinematics(end.tolist())
        self.state.pose = list(self.pose_from_matrix(final_T).values())[:6]

        return trajectory

    def plan_ptp_urdf(self, target_joints: list[float], steps: int = 30) -> list[dict]:
        """PTP轨迹规划 — 使用URDF FK计算位姿（匹配3D场景）"""
        start = np.array(self.state.joints)
        end = np.array(target_joints)

        t = np.linspace(0, 1, steps)
        ramp_up = 0.3
        ramp_down = 0.7
        profile = np.piecewise(
            t,
            [t < ramp_up, (t >= ramp_up) & (t <= ramp_down), t > ramp_down],
            [lambda x: x**2 / (2 * ramp_up * (1 - ramp_up)),
             lambda x: (x - ramp_up / 2),
             lambda x: 1 - (1 - x)**2 / (2 * ramp_down * (1 - ramp_down))])

        trajectory = []
        for i in range(steps):
            frac = float(profile[i]) if i < len(profile) else 1.0
            q = start + frac * (end - start)
            pose = self.urdf_fk_pose(q.tolist())
            trajectory.append({
                "step": i,
                "progress": round(frac, 3),
                "joints": [round(float(v), 4) for v in q],
                "pose": pose,
            })

        self.state.joints = [round(float(v), 4) for v in end]
        self.state.pose = list(self.urdf_fk_pose(end.tolist()).values())[:6]
        return trajectory

    def plan_lin(self, target_pose: dict, steps: int = 50) -> list[dict]:
        """
        LIN (直线) 轨迹规划 — 笛卡尔空间直线插补。
        通过反复调用 IK 求解路径点。
        """
        start_T = self.forward_kinematics(self.state.joints)
        end_T = np.eye(4)
        for i, key in enumerate(["x", "y", "z"]):
            end_T[i, 3] = target_pose.get(key, 0)

        trajectory = []
        for i in range(steps):
            frac = i / max(steps - 1, 1)
            Ti = start_T + frac * (end_T - start_T)
            pi = self.pose_from_matrix(Ti)
            qi = self.inverse_kinematics(pi)
            if qi is None:
                logger.warning("LIN trajectory IK failed at step %d", i)
                break
            trajectory.append({
                "step": i,
                "progress": round(frac, 3),
                "joints": [round(float(v), 4) for v in qi],
                "pose": pi,
            })

        if trajectory:
            final_q = trajectory[-1]["joints"]
            self.state.joints = final_q
            self.state.pose = list(trajectory[-1]["pose"].values())[:6]

        return trajectory


# ── 全局机器人实例 ──
robot = SixAxisRobot()


# ══════════════════════════════════════════════════════════════
# API 路由
# ══════════════════════════════════════════════════════════════

@app.route("/ik", methods=["POST"])
def ik():
    """逆运动学求解 (URDF模型, 世界坐标)"""
    try:
        data = request.get_json(force=True)
        wx = data.get("x", 300)
        wy = data.get("y", 0)
        wz = data.get("z", 400)
        joints = robot.urdf_ik_robust(wx, wy, wz)
        if joints is None:
            joints = robot.urdf_inverse_kinematics_world(wx, wy, wz)
        if joints is None:
            return jsonify({"success": False, "error": "IK 未收敛，目标可能超出工作范围"}), 400
        pose = robot.urdf_fk_pose(joints)
        return jsonify({"success": True, "target_xyz": [wx, wy, wz], "joints": joints, "pose": pose})
    except Exception as e:
        logger.exception("IK error")
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/fk", methods=["POST"])
def fk():
    """正运动学求解 (URDF模型)"""
    try:
        data = request.get_json(force=True)
        joints = data.get("joints", robot.state.joints)
        pose = robot.urdf_fk_pose(joints)
        return jsonify({"success": True, "joints": joints, "pose": pose})
    except Exception as e:
        logger.exception("FK error")
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/plan", methods=["POST"])
def plan():
    """轨迹规划"""
    try:
        data = request.get_json(force=True)
        motion_type = data.get("type", "PTP").upper()
        steps = data.get("steps", 50)

        if motion_type == "PTP":
            target_joints = data["joints"]
            traj = robot.plan_ptp(target_joints, steps)
        elif motion_type == "LIN":
            target_pose = {
                "x": data.get("x", 300),
                "y": data.get("y", 0),
                "z": data.get("z", 400),
            }
            traj = robot.plan_lin(target_pose, steps)
        else:
            return jsonify({"success": False, "error": f"不支持的运动类型: {motion_type}"}), 400

        return jsonify({
            "success": True,
            "type": motion_type,
            "steps": len(traj),
            "trajectory": traj,
            "final_pose": traj[-1]["pose"] if traj else None,
        })
    except Exception as e:
        logger.exception("Plan error")
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/execute", methods=["POST"])
def execute():
    """
    执行完整动作。支持动作类型:
      - pick:   移动到目标上方 → 下降 → 夹爪关闭 → 抬升
      - place:  移动到目标上方 → 下降 → 夹爪打开 → 抬升
      - move:   移动到目标位姿
      - pick_and_place: 从source抓取 → 放到place_target
    """
    try:
        data = request.get_json(force=True)
        action = data.get("action", "move")
        target = data.get("target", {})
        # target: {x, y, z} 或 {joints: [...]}

        sequence = []
        gripper_state = robot.state.gripper

        # 辅助函数: URDF鲁棒IK求解 (世界坐标 → 关节角)
        def _solve_ik(wx, wy, wz):
            """使用URDF IK在视觉世界坐标中求解关节角"""
            result = robot.urdf_ik_robust(wx, wy, wz)
            if result is None:
                result = robot.urdf_inverse_kinematics_world(wx, wy, wz)
            return result

        def _do_pick_or_place(tx, ty, tz, is_pick):
            """执行单次 pick 或 place 子步骤 (世界坐标), 返回 (sequence, error)"""
            sub = []
            approach_z = tz + 80

            saved_joints = list(robot.state.joints)

            approach_ik = _solve_ik(tx, ty, approach_z)
            if approach_ik is None:
                return None, f"接近点({tx},{ty},{approach_z})超出工作范围"
            robot.state.joints = saved_joints
            seq1 = robot.plan_ptp_urdf(approach_ik, steps=30)
            robot.state.joints = list(approach_ik)
            sub.append({"phase": "approach", "trajectory": seq1})

            saved_joints = list(robot.state.joints)
            down_ik = _solve_ik(tx, ty, tz)
            if down_ik is None:
                return None, f"下降点({tx},{ty},{tz})超出工作范围"
            robot.state.joints = saved_joints
            seq2 = robot.plan_ptp_urdf(down_ik, steps=20)
            robot.state.joints = list(down_ik)
            sub.append({"phase": "descend", "trajectory": seq2})

            robot.state.gripper = 1.0 if is_pick else 0.0
            sub.append({"phase": "gripper",
                        "action": "close" if is_pick else "open"})

            saved_joints = list(robot.state.joints)
            lift_ik = _solve_ik(tx, ty, approach_z)
            if lift_ik is not None:
                robot.state.joints = saved_joints
                seq4 = robot.plan_ptp_urdf(lift_ik, steps=30)
                robot.state.joints = list(lift_ik)
                sub.append({"phase": "retreat", "trajectory": seq4})

            return sub, None

        if action == "pick_and_place":
            place_target = data.get("place_target", {})
            sx, sy, sz = target.get("x", 300), target.get("y", 0), target.get("z", 300)
            px = place_target.get("x", 220)
            py = place_target.get("y", -100)
            pz = place_target.get("z", 360)

            pick_seq, err = _do_pick_or_place(sx, sy, sz, is_pick=True)
            if err:
                return jsonify({"success": False, "error": f"抓取失败: {err}"}), 400
            sequence.extend(pick_seq)

            place_seq, err = _do_pick_or_place(px, py, pz, is_pick=False)
            if err:
                return jsonify({"success": False, "error": f"放置失败: {err}"}), 400
            sequence.extend(place_seq)

        elif action == "pick" or action == "place":
            tx = target.get("x", 300)
            ty = target.get("y", 0)
            tz = target.get("z", 300)

            sub_seq, err = _do_pick_or_place(tx, ty, tz, is_pick=(action == "pick"))
            if err:
                return jsonify({"success": False, "error": err}), 400
            sequence.extend(sub_seq)

        elif action == "move":
            if "joints" in target:
                traj = robot.plan_ptp_urdf(target["joints"], steps=50)
            else:
                px = target.get("x", 300)
                py = target.get("y", 0)
                pz = target.get("z", 400)
                ik = _solve_ik(px, py, pz)
                if ik is None:
                    return jsonify({"success": False,
                                    "error": f"目标({px},{py},{pz})超出工作范围。"
                                    f"建议范围: x=200-550, y=(-250)-250, z=150-500"}), 400
                traj = robot.plan_ptp_urdf(ik, steps=50)
            sequence.append({"phase": "move", "trajectory": traj})

        total_steps = sum(len(p.get("trajectory", [])) for p in sequence)

        return jsonify({
            "success": True,
            "action": action,
            "target": target,
            "phases": len(sequence),
            "total_steps": total_steps,
            "sequence": sequence,
            "final_state": {
                "joints": robot.state.joints,
                "gripper": robot.state.gripper,
            },
        })
    except Exception as e:
        logger.exception("Execute error")
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/status", methods=["GET"])
def status():
    """获取机器人当前状态 (URDF FK)"""
    pose = robot.urdf_fk_pose(robot.state.joints)
    return jsonify({
        "joints": robot.state.joints,
        "pose": pose,
        "gripper": robot.state.gripper,
    })


@app.route("/reset", methods=["POST"])
def reset():
    """重置机器人为零位"""
    robot.state = RobotState()
    return jsonify({"success": True, "message": "机器人已重置为零位"})


@app.route("/ready", methods=["POST"])
def ready():
    """移动机器人到就绪位姿 (非奇异工作构型)"""
    # 使用URDF模型的自然前伸构型
    ready_joints = [0.0, math.radians(80), math.radians(-90), 0.0, math.radians(0), 0.0]
    try:
        traj = robot.plan_ptp_urdf(ready_joints, steps=40)
        robot.state.joints = [round(float(j), 4) for j in ready_joints]
        pose = robot.urdf_fk_pose(ready_joints)
        return jsonify({
            "success": True,
            "message": "机器人已移动到就绪位姿",
            "joints": [round(float(j), 4) for j in ready_joints],
            "pose": pose,
            "steps": len(traj),
        })
    except Exception as e:
        logger.exception("ready error")
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/jog", methods=["POST"])
def jog():
    """
    点动控制: 关节或笛卡尔增量运动。

    JSON body:
      - type: "joint" | "cartesian"
      Joint mode: index (0-5), delta_deg (float), speed (int 1-100)
      Cartesian mode: axis ("x"|"y"|"z"), delta_mm (float), speed (int 1-100)
    """
    try:
        data = request.get_json(force=True)
        jog_type = data.get("type", "joint")
        speed_pct = data.get("speed", 30)

        current_joints = list(robot.state.joints) if robot.state.joints else [0.0] * 6
        if len(current_joints) < 6:
            current_joints = [0.0] * 6

        if jog_type == "joint":
            idx = int(data.get("index", 0))
            delta_deg = float(data.get("delta_deg", 1.0))
            delta_rad = math.radians(delta_deg)
            target = list(current_joints)
            target[idx] += delta_rad
            # Clamp to safe ranges
            limits_deg = [170, 110, 70, 190, 120, 360]
            for i in range(6):
                lim = math.radians(limits_deg[i])
                target[i] = max(-lim, min(lim, target[i]))
            robot.state.joints = [round(float(j), 4) for j in target]
            trajectory = robot.plan_ptp_urdf(target, steps=max(10, int(30 * speed_pct / 100)))
            return jsonify({
                "success": True,
                "joints": target,
                "joints_deg": [round(math.degrees(j), 2) for j in target],
                "tcp": robot.urdf_fk_world(target).tolist(),
                "trajectory": trajectory,
            })

        elif jog_type == "cartesian":
            axis = data.get("axis", "x")
            delta_mm = float(data.get("delta_mm", 10.0))
            # Get current TCP from URDF FK
            tcp_current = robot.urdf_fk_world(current_joints)  # [X_fwd, Y_lr, Z_up]
            target_tcp = list(tcp_current)
            axis_map = {"x": 0, "y": 1, "z": 2}
            target_tcp[axis_map.get(axis, 0)] += delta_mm

            # IK to reach new TCP
            ik_result = robot.urdf_ik_robust(target_tcp[0], target_tcp[1], target_tcp[2])
            if ik_result:
                target_joints = list(ik_result)
                robot.state.joints = [round(float(j), 4) for j in target_joints]
                trajectory = robot.plan_ptp_urdf(target_joints, steps=max(10, int(30 * speed_pct / 100)))
                return jsonify({
                    "success": True,
                    "joints": target_joints,
                    "joints_deg": [round(math.degrees(j), 2) for j in target_joints],
                    "tcp": target_tcp,
                    "trajectory": trajectory,
                })
            else:
                return jsonify({"success": False, "error": "IK求解失败，目标超出工作范围"}), 400

        else:
            return jsonify({"success": False, "error": f"Unknown jog type: {jog_type}"}), 400

    except Exception as e:
        logger.exception("Jog failed")
        return jsonify({"success": False, "error": str(e)}), 500


_program_store = {}  # In-memory program storage


@app.route("/program/upload", methods=["POST"])
def program_upload():
    """接收并存储 FANUC TP 程序"""
    try:
        data = request.get_json(force=True)
        program_text = data.get("program", "")
        if not program_text:
            return jsonify({"success": False, "error": "Empty program"}), 400
        _program_store["STYLE_MN"] = program_text
        logger.info("Program uploaded: %d chars", len(program_text))
        return jsonify({"success": True, "program_name": "STYLE_MN", "size": len(program_text)})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/program/<name>", methods=["GET"])
def program_get(name):
    """获取已存储的 TP 程序"""
    program = _program_store.get(name, "")
    if not program:
        return jsonify({"success": False, "error": "Program not found"}), 404
    return jsonify({"success": True, "program_name": name, "program": program})


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "service": "robot", "axes": 6})


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8802)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logger.info("Robot Service starting on port %d", args.port)
    app.run(host="0.0.0.0", port=args.port, debug=args.debug)
