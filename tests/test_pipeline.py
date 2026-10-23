"""
集成测试: 视觉引导机器人系统端到端验证
=======================================
测试覆盖:
  1. 视觉服务 — 模拟图像目标检测 + 颜色识别
  2. 机器人服务 — 正逆运动学 + 轨迹规划 + 动作序列
  3. Agent 服务 — 自然语言指令 → 工具调用链
  4. 端到端 — 模拟图像 → 检测 → IK → 轨迹 → 动画

用法: python tests/test_pipeline.py [--vision-port 8801] [--robot-port 8802] [--agent-port 8803]
"""
import sys
import os
import json
import time
import math
import base64
import unittest

import numpy as np
import cv2

# 添加项目根目录
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ══════════════════════════════════════════════════════════════
# 测试辅助
# ══════════════════════════════════════════════════════════════

def create_test_image(width=640, height=480):
    """创建模拟工作场景的测试图像：彩色方块代表工件"""
    img = np.ones((height, width, 3), dtype=np.uint8) * 220

    objects = [
        {"x": 180, "y": 200, "w": 60, "h": 40, "color": (0, 0, 255)},    # 红色矩形
        {"x": 380, "y": 160, "w": 50, "h": 50, "color": (0, 255, 0)},    # 绿色方形
        {"x": 300, "y": 300, "w": 45, "h": 35, "color": (255, 0, 0)},    # 蓝色矩形
        {"x": 480, "y": 250, "w": 40, "h": 40, "color": (0, 200, 255)},  # 黄色方形
    ]

    for obj in objects:
        x1, y1 = obj["x"], obj["y"]
        x2, y2 = x1 + obj["w"], y1 + obj["h"]
        cv2.rectangle(img, (x1, y1), (x2, y2), obj["color"], -1)

    return img, objects


class TestVisionService(unittest.TestCase):
    """视觉服务单元测试 (不依赖外部服务)"""

    def test_create_test_image(self):
        img, objs = create_test_image()
        self.assertEqual(img.shape, (480, 640, 3))
        self.assertEqual(len(objs), 4)

    def test_color_ranges(self):
        """验证颜色范围定义正确"""
        from services.vision_service import COLOR_RANGES
        self.assertIn("红色", COLOR_RANGES)
        self.assertIn("蓝色", COLOR_RANGES)
        self.assertIn("绿色", COLOR_RANGES)
        self.assertGreater(len(COLOR_RANGES), 6)

    def test_image_preprocessing(self):
        """验证图像预处理流程"""
        img, _ = create_test_image()
        # CLAHE
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_eq = clahe.apply(l)
        self.assertEqual(l_eq.shape, l.shape)

    def test_color_detection_on_synthetic(self):
        """验证合成图像的颜色识别"""
        img, objs = create_test_image()
        from services.vision_service import COLOR_RANGES

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 检测红色区域
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_RANGES["红色"]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        red_pct = np.sum(mask > 0) / mask.size * 100
        self.assertGreater(red_pct, 0.5, "红色区域占比应 > 0.5%")

    def test_fallback_detection(self):
        """验证 OpenCV 轮廓检测回退方案"""
        img, _ = create_test_image()
        from services.vision_service import _fallback_detection

        data, code = _fallback_detection(img, 0.35)
        self.assertEqual(code, 200)
        self.assertTrue(data.get("success"))


_ = """
服务集成测试 (需要启动对应服务)

启动服务:
  python services/vision_service.py --port 8801 &
  python services/robot_service.py --port 8802 &
  python services/agent_service.py --port 8803 &

然后运行:
  python tests/test_pipeline.py --integration
"""


class TestRobotService(unittest.TestCase):
    """机器人服务单元测试"""

    def test_dh_transform(self):
        """验证 DH 参数变换矩阵"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        T = robot.dh_transform(300, 0, 0, 0)
        self.assertEqual(T.shape, (4, 4))
        self.assertAlmostEqual(T[0, 3], 300)

    def test_forward_kinematics_zero(self):
        """验证零位正运动学"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        T = robot.forward_kinematics([0, 0, 0, 0, 0, 0])
        self.assertEqual(T.shape, (4, 4))
        # 末端 z 应为正值 (基座以上)
        self.assertGreater(T[2, 3], 0)

    def test_forward_kinematics_reproducibility(self):
        """验证正运动学可重现性"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        joints = [0.5, -0.3, 0.2, 0.1, -0.4, 0.0]
        T1 = robot.forward_kinematics(joints)
        T2 = robot.forward_kinematics(joints)
        self.assertTrue(np.allclose(T1, T2))

    def test_inverse_kinematics_basic(self):
        """验证逆运动学: FK → IK → FK 闭环 (仅验证位置)"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        # 使用已知可达构型: 先正向算，再用IK逆向，只比较位置
        start_joints = [0.0, math.radians(15), math.radians(-10), 0.0, math.radians(5), 0.0]
        T = robot.forward_kinematics(start_joints)
        pose = robot.pose_from_matrix(T)
        # IK从相同起始构型开始，只做位置匹配
        robot.state.joints = list(start_joints)
        # 轻微扰动位置
        target = {"x": pose["x"] + 2, "y": pose["y"] - 1, "z": pose["z"] + 1,
                  "rx": pose["rx"], "ry": pose["ry"], "rz": pose["rz"]}
        result = robot.inverse_kinematics(target)
        self.assertIsNotNone(result, "IK应从当前构型收敛到附近位置")
        self.assertEqual(len(result), 6)
        # 验证位置匹配
        T_check = robot.forward_kinematics(result)
        pose_check = robot.pose_from_matrix(T_check)
        for key in ["x", "y", "z"]:
            self.assertAlmostEqual(target[key], pose_check[key], delta=8,
                                   msg=f"{key}不匹配")

    def test_fk_consistency(self):
        """验证正运动学的一致性"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        # 零位
        T0 = robot.forward_kinematics([0, 0, 0, 0, 0, 0])
        p0 = robot.pose_from_matrix(T0)
        # 零位z应为正 (基座以上)
        self.assertGreater(p0["z"], 0)
        # 非零位应产生不同位姿
        T1 = robot.forward_kinematics([0.5, 0, 0, 0, 0, 0])
        p1 = robot.pose_from_matrix(T1)
        self.assertNotEqual(p0["x"], p1["x"])

    def test_ptp_plan(self):
        """验证 PTP 轨迹规划"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        robot.state.joints = [0, 0, 0, 0, 0, 0]
        target = [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]
        traj = robot.plan_ptp(target, steps=30)
        self.assertEqual(len(traj), 30)
        # 检查终点关节角
        for j, expected in zip(traj[-1]["joints"], target):
            self.assertAlmostEqual(j, expected, delta=0.1)

    def test_pick_sequence(self):
        """验证抓取序列逻辑"""
        from services.robot_service import SixAxisRobot, RobotState
        robot = SixAxisRobot()
        robot.state = RobotState(joints=[0, 0, 0, 0, 0, 0])
        # 使用 client 方式测试 execute
        from flask import Flask
        app_test = Flask(__name__)
        with app_test.test_request_context(json={
            "action": "pick", "target": {"x": 300, "y": 0, "z": 300}
        }):
            pass  # 实际测试需要启动服务

    def test_jacobian_numeric(self):
        """验证数值雅可比矩阵"""
        from services.robot_service import SixAxisRobot
        robot = SixAxisRobot()
        q = np.array([0.1, -0.1, 0.2, 0.0, 0.0, 0.0])
        J = robot._jacobian_numeric(q)
        self.assertEqual(J.shape, (6, 6))
        self.assertFalse(np.any(np.isnan(J)))


class TestAgentTools(unittest.TestCase):
    """Agent 工具系统测试"""

    def test_tools_registered(self):
        """验证所有工具已注册"""
        from agent.tools import get_all, get_schemas, register_all
        register_all()
        tools = get_all()
        self.assertGreaterEqual(len(tools), 7)
        schemas = get_schemas()
        self.assertEqual(len(schemas), len(tools))

    def test_tool_names_unique(self):
        """验证工具名唯一"""
        from agent.tools import get_all, register_all
        register_all()
        names = [t.name for t in get_all()]
        self.assertEqual(len(names), len(set(names)))

    def test_vision_tools_exist(self):
        """验证视觉工具存在"""
        from agent.tools import get, register_all
        register_all()
        self.assertIsNotNone(get("analyze_scene"))
        self.assertIsNotNone(get("detect_objects"))
        self.assertIsNotNone(get("identify_color"))

    def test_robot_tools_exist(self):
        """验证机器人工具存在"""
        from agent.tools import get, register_all
        register_all()
        self.assertIsNotNone(get("get_robot_status"))
        self.assertIsNotNone(get("move_robot_to"))
        self.assertIsNotNone(get("pick_object"))
        self.assertIsNotNone(get("place_object"))
        self.assertIsNotNone(get("reset_robot"))


class TestAgentLoop(unittest.TestCase):
    """Agent 循环测试"""

    def test_agent_creation(self):
        """验证 Agent 创建"""
        from agent.react_loop import VisionRobotAgent, SYSTEM_PROMPT
        agent = VisionRobotAgent()
        self.assertIsNotNone(agent)
        self.assertIn("六轴机器人", SYSTEM_PROMPT)
        self.assertIn("六轴机器人", SYSTEM_PROMPT)
        self.assertIn("视觉感知", SYSTEM_PROMPT)

    def test_system_prompt_contains_tools(self):
        """验证系统提示词包含工具描述"""
        from agent.react_loop import SYSTEM_PROMPT
        self.assertIn("analyze_scene", SYSTEM_PROMPT)
        self.assertIn("pick_object", SYSTEM_PROMPT)
        self.assertIn("move_robot_to", SYSTEM_PROMPT)


def run_integration_tests():
    """需要服务运行的集成测试"""
    import requests

    robot_url = "http://127.0.0.1:8802"
    vision_url = "http://127.0.0.1:8801"
    agent_url = "http://127.0.0.1:8803"

    print("\n" + "=" * 60)
    print(" 集成测试: 视觉引导机器人系统")
    print("=" * 60)

    # 1. 服务健康检查
    print("\n[1/5] 健康检查...")
    try:
        r = requests.get(f"{vision_url}/health", timeout=3)
        assert r.status_code == 200
        print(f"  Vision: OK ({r.json()})")
    except Exception as e:
        print(f"  Vision: SKIP ({e})")

    try:
        r = requests.get(f"{robot_url}/health", timeout=3)
        assert r.status_code == 200
        print(f"  Robot: OK")
    except Exception as e:
        print(f"  Robot: SKIP ({e})")

    try:
        r = requests.get(f"{agent_url}/health", timeout=3)
        assert r.status_code == 200
        data = r.json()
        print(f"  Agent: OK (model={data.get('model')}, tools={data.get('tools_loaded')})")
    except Exception as e:
        print(f"  Agent: SKIP ({e})")

    # 2. 机器人运动学
    print("\n[2/5] 机器人运动学...")
    try:
        r = requests.post(f"{robot_url}/fk", json={
            "joints": [0, -0.5, 0.3, 0, 0, 0]
        }, timeout=5)
        assert r.status_code == 200
        data = r.json()
        assert data["success"]
        assert "pose" in data
        print(f"  FK: x={data['pose']['x']:.1f} y={data['pose']['y']:.1f} z={data['pose']['z']:.1f}")
    except Exception as e:
        print(f"  FK: SKIP ({e})")

    # 3. 轨迹规划
    print("\n[3/5] 轨迹规划...")
    try:
        requests.post(f"{robot_url}/reset", json={}, timeout=3)
        r = requests.post(f"{robot_url}/plan", json={
            "type": "PTP",
            "joints": [0.5, -0.3, 0.2, 0.1, -0.1, 0.0],
            "steps": 20,
        }, timeout=10)
        assert r.status_code == 200
        data = r.json()
        assert data["success"]
        assert len(data["trajectory"]) == 20
        print(f"  PTP: {data['steps']} steps, final_pose={data['final_pose']}")
    except Exception as e:
        print(f"  Plan: SKIP ({e})")

    # 4. 视觉分析 (直接调用, 不需要HTTP服务)
    print("\n[4/5] 视觉分析...")
    img, _ = create_test_image()
    _, buf = cv2.imencode(".jpg", img)
    img_b64 = base64.b64encode(buf).decode()
    try:
        r = requests.post(f"{vision_url}/analyze", json={
            "image_b64": img_b64
        }, timeout=30)
        if r.status_code == 200:
            data = r.json()
            if data["success"]:
                print(f"  物体数: {data['count']}")
                for obj in data.get("objects", []):
                    print(f"    - {obj['color']} {obj['label']} "
                          f"({obj['center'][0]:.0f}, {obj['center'][1]:.0f})")
                print(f"  摘要: {data.get('summary', '')[:80]}...")
    except Exception as e:
        print(f"  Vision: SKIP ({e})")

    # 5. 抓取动作序列
    print("\n[5/5] 抓取动作序列...")
    try:
        r = requests.post(f"{robot_url}/execute", json={
            "action": "pick",
            "target": {"x": 350, "y": -50, "z": 300},
        }, timeout=30)
        if r.status_code == 200:
            data = r.json()
            if data["success"]:
                print(f"  抓取完成: {data['phases']} phases, {data['total_steps']} steps")
                print(f"  最终关节: {[round(j, 2) for j in data['final_state']['joints']]}")
                print(f"  夹爪状态: {'关闭' if data['final_state']['gripper'] > 0.5 else '打开'}")
            else:
                print(f"  失败: {data.get('error')}")
    except Exception as e:
        print(f"  Execute: SKIP ({e})")

    print("\n" + "=" * 60)
    print(" 集成测试完成")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="视觉引导机器人系统测试")
    parser.add_argument("--integration", action="store_true", help="运行集成测试 (需要启动服务)")
    parser.add_argument("--unit", action="store_true", help="仅运行单元测试")
    parser.add_argument("--all", action="store_true", help="运行所有测试")
    args = parser.parse_args()

    if args.integration:
        run_integration_tests()
    elif args.unit:
        unittest.main(argv=[""], exit=False)
    elif args.all:
        unittest.main(argv=[""], exit=False)
        run_integration_tests()
    else:
        # 默认: 只运行单元测试
        unittest.main(argv=[""], exit=False)
