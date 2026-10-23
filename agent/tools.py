"""
Agent 工具系统 — 视觉 + 机器人控制工具
=======================================
每个工具 = Python 函数 + JSON Schema 描述。
Agent 通过 Function Calling 机制调用这些工具完成实际任务。
"""
import json
import base64
import logging
from typing import Callable
from dataclasses import dataclass, field

import requests

from .config import AgentConfig, get_config

logger = logging.getLogger("agent.tools")


@dataclass
class Tool:
    name: str
    description: str
    parameters: dict
    function: Callable
    category: str = "general"


_tools: dict[str, Tool] = {}
_config: AgentConfig | None = None


def _cfg() -> AgentConfig:
    global _config
    if _config is None:
        _config = get_config()
    return _config


def register(tool: Tool):
    _tools[tool.name] = tool


def get_all() -> list[Tool]:
    return list(_tools.values())


def get(name: str) -> Tool | None:
    return _tools.get(name)


def get_schemas() -> list[dict]:
    return [
        {"type": "function", "function": {"name": t.name, "description": t.description,
                                           "parameters": t.parameters}}
        for t in _tools.values()
    ]


def execute(name: str, args: dict) -> str:
    tool = _tools.get(name)
    if not tool:
        return f"[错误] 未找到工具: {name}"
    try:
        result = tool.function(**args)
        if isinstance(result, dict):
            return json.dumps(result, ensure_ascii=False, indent=2)
        return str(result)
    except Exception as e:
        logger.exception("Tool %s failed", name)
        return f"[工具执行失败] {name}: {e}"


# ══════════════════════════════════════════════════════════════
# 坐标系统 & 机器人安装参数 (与前端 app.js 保持一致)
# ══════════════════════════════════════════════════════════════
# 3D场景坐标系: X=前后, Y=上下, Z=左右 (Three.js Y-up)
# 机器人坐标系:  X=前后, Y=左右, Z=上下 (URDF/DH Z-up)
# 机器人基座在3D场景中的位置: (-90, PEDESTAL_H, 0) mm
PEDESTAL_H = 340       # 底座顶面高度 (mm)
ROBOT_BASE_X = -90     # 机器人基座 X 坐标
ROBOT_BASE_Y = 0       # 机器人基座 Y(左右) 坐标

TABLE_SURFACE_Z = 210  # 工作台面高度 (mm)


def _world_to_robot(wx: float, wy: float, wz: float) -> tuple:
    """3D场景世界坐标 → 机器人基座坐标系 (Z-up, 单位mm)

    视觉约定: X=前后, Y=左右, Z=上下
    robotAssembly 绕X轴旋转-π/2: URDF_Z→scene_Y(上), URDF_Y→scene_-Z(右)
    因此: robot_X = scene_X - base_X, robot_Y = -(scene_Z - base_Z), robot_Z = scene_Y - base_Y
    """
    rx = wx - ROBOT_BASE_X   # 前后 (X → X)
    ry = -(wy - ROBOT_BASE_Y) # 左右 (3D场景Z → 机器人Y, 需取反)
    rz = wz - PEDESTAL_H     # 上下 (3D场景Y → 机器人Z)
    return rx, ry, rz


# ══════════════════════════════════════════════════════════════
# 视觉工具
# ══════════════════════════════════════════════════════════════

def analyze_scene(image_path: str = "") -> str:
    """
    分析工作场景：调用视觉服务检测所有物体及其颜色，并自动将像素坐标转换为世界坐标。
    返回场景中的物体列表(含世界坐标)、颜色、位置和空间关系。
    如果未提供图像路径，自动使用默认样本图像 data/workspace_sample.jpg。
    """
    cfg = _cfg()
    if not image_path:
        image_path = "data/workspace_sample.jpg"

    try:
        with open(image_path, "rb") as f:
            img_b64 = base64.b64encode(f.read()).decode()
    except FileNotFoundError:
        img_b64 = _generate_sample_image_b64()
        if not img_b64:
            return {"error": f"图像文件不存在且无法生成样本图像: {image_path}",
                    "hint": "请提供有效的图像路径，或者确保视觉服务可生成样本图像"}

    try:
        resp = requests.post(f"{cfg.vision_url}/analyze",
                             json={"image_b64": img_b64}, timeout=30)
        data = resp.json()

        # 像素→3D世界坐标 (mm). 相机俯视工作台区域: X=200-550, Z=-180-180
        if data.get("objects"):
            for obj in data["objects"]:
                px, py = obj.get("center", [320, 240])
                wx = round(200 + (px / 640) * 350)      # 3D场景X: 200→550
                wz = round(-180 + (py / 480) * 360)      # 3D场景Z: -180→180
                wy = TABLE_SURFACE_Z + 18                # 3D场景Y: 台面+工件高度 ≈228
                obj["world_xyz"] = [wx, wz, wy]          # [X, Z, Y] → 视觉约定: X前后, Y左右, Z上下

        return data
    except requests.ConnectionError:
        return {"error": f"视觉服务未启动 ({cfg.vision_url})，请先运行: python services/vision_service.py"}


def _generate_sample_image_b64() -> str:
    """动态生成合成工作场景图像"""
    try:
        import cv2
        import numpy as np
        img = np.ones((480, 640, 3), dtype=np.uint8) * 200
        objects = [
            (130, 180, 80, 65, (0, 0, 255)),     # 红色工件 (较大)
            (340, 160, 75, 60, (255, 0, 0)),     # 蓝色工件
            (280, 300, 80, 55, (0, 255, 0)),     # 绿色工件
            (460, 240, 70, 60, (0, 200, 255)),   # 黄色工件
        ]
        for x, y, w, h, color in objects:
            cv2.rectangle(img, (x, y), (x + w, y + h), color, -1)
        _, buf = cv2.imencode(".jpg", img)
        return base64.b64encode(buf).decode()
    except ImportError:
        return ""


def detect_objects(image_path: str = "", conf: float = 0.35) -> dict:
    """
    目标检测：检测图像中的物体并返回边界框和类别。
    如果图像不存在，自动使用合成样本图像。
    """
    cfg = _cfg()
    try:
        with open(image_path, "rb") as f:
            img_b64 = base64.b64encode(f.read()).decode()
    except (FileNotFoundError, ValueError):
        img_b64 = _generate_sample_image_b64()
        if not img_b64:
            return {"error": f"图像文件不存在: {image_path}，且无法生成样本图像"}

    try:
        resp = requests.post(f"{cfg.vision_url}/detect",
                             json={"image_b64": img_b64, "conf": conf}, timeout=30)
        return resp.json()
    except requests.ConnectionError:
        return {"error": f"视觉服务未启动 ({cfg.vision_url})"}


def identify_color(image_path: str = "", roi: list = None) -> dict:
    """
    颜色识别：识别指定区域或整张图片中物体的颜色。
    roi: [x1, y1, x2, y2] 指定区域，不传则分析整张图。
    如果图像不存在，自动使用合成样本图像。
    """
    cfg = _cfg()
    try:
        with open(image_path, "rb") as f:
            img_b64 = base64.b64encode(f.read()).decode()
    except (FileNotFoundError, ValueError):
        img_b64 = _generate_sample_image_b64()
        if not img_b64:
            return {"error": f"图像文件不存在: {image_path}，且无法生成样本图像"}
    body = {"image_b64": img_b64}
    if roi:
        body["roi"] = roi
    try:
        resp = requests.post(f"{cfg.vision_url}/color",
                             json=body, timeout=30)
        return resp.json()
    except requests.ConnectionError:
        return {"error": f"视觉服务未启动 ({cfg.vision_url})"}


# ══════════════════════════════════════════════════════════════
# 机器人控制工具
# ══════════════════════════════════════════════════════════════

def get_robot_status() -> dict:
    """获取机器人当前状态：关节角、末端位姿、夹爪状态。"""
    cfg = _cfg()
    try:
        resp = requests.get(f"{cfg.robot_url}/status", timeout=5)
        return resp.json()
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})，请先运行: python services/robot_service.py"}


def move_robot_to(x: float, y: float, z: float, rx: float = 180, ry: float = 0, rz: float = 0) -> dict:
    """
    移动机器人末端到指定位置。
    x, y, z: 3D场景世界坐标 (视觉约定: X=前后, Y=左右, Z=上下)，单位mm。
    典型工作范围: x: 200-550, y: -180-180, z: 150-400
    """
    cfg = _cfg()
    try:
        resp = requests.post(f"{cfg.robot_url}/execute", json={
            "action": "move",
            "target": {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz},
        }, timeout=60)
        data = resp.json()
        if data.get("success"):
            final = data.get("final_state", {})
            return {
                "message": f"机器人已移动到世界({x}, {y}, {z})",
                "world_xyz": [x, y, z],
                "final_joints": final.get("joints"),
                "gripper": final.get("gripper"),
                "total_steps": data.get("total_steps", 0),
                "sequence": data.get("sequence", []),
                "final_state": final,
            }
        return {"error": data.get("error", "移动失败")}
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})"}


def pick_object(x: float, y: float, z: float) -> dict:
    """
    抓取指定位置的物体。
    执行: 移动到上方 → 下降 → 夹爪关闭 → 抬升 → 回到安全点。
    x, y, z: 3D场景世界坐标 (视觉约定: X=前后, Y=左右, Z=上下)，单位mm。
    """
    cfg = _cfg()
    try:
        resp = requests.post(f"{cfg.robot_url}/execute", json={
            "action": "pick", "target": {"x": x, "y": y, "z": z},
        }, timeout=60)
        data = resp.json()
        if data.get("success"):
            return {"message": f"已抓取世界({x}, {y}, {z})处的物体",
                    "world_xyz": [x, y, z],
                    "phases": data.get("phases"), "total_steps": data.get("total_steps"),
                    "sequence": data.get("sequence", []),
                    "final_state": data.get("final_state", {})}
        return {"error": data.get("error", "抓取失败")}
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})"}


def place_object(x: float, y: float, z: float) -> dict:
    """
    放置物体到指定位置。
    执行: 移动到上方 → 下降 → 夹爪打开 → 抬升 → 回到安全点。
    x, y, z: 3D场景世界坐标 (视觉约定: X=前后, Y=左右, Z=上下)，单位mm。
    """
    cfg = _cfg()
    try:
        resp = requests.post(f"{cfg.robot_url}/execute", json={
            "action": "place", "target": {"x": x, "y": y, "z": z},
        }, timeout=60)
        data = resp.json()
        if data.get("success"):
            return {"message": f"已将物体放置到世界({x}, {y}, {z})",
                    "world_xyz": [x, y, z],
                    "phases": data.get("phases"), "total_steps": data.get("total_steps"),
                    "sequence": data.get("sequence", []),
                    "final_state": data.get("final_state", {})}
        return {"error": data.get("error", "放置失败")}
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})"}


def pick_and_place(x: float, y: float, z: float,
                   target_x: float, target_y: float, target_z: float) -> dict:
    """
    一步完成抓取+放置：从(x,y,z)抓取物体，放到(target_x,target_y,target_z)。
    内部自动执行: 移动到抓取点上方→下降→夹紧→抬升→移动到放置点上方→下降→松开→抬升。
    x/y/z: 抓取位置世界坐标(视觉约定: X=前后, Y=左右, Z=上下)mm,
    target_x/y/z: 放置位置世界坐标mm。
    """
    cfg = _cfg()
    try:
        resp = requests.post(f"{cfg.robot_url}/execute", json={
            "action": "pick_and_place",
            "target": {"x": x, "y": y, "z": z},
            "place_target": {"x": target_x, "y": target_y, "z": target_z},
        }, timeout=120)
        data = resp.json()
        if data.get("success"):
            return {
                "message": f"已完成: 从世界({x},{y},{z})抓取 → 放置到世界({target_x},{target_y},{target_z})",
                "phases": data.get("phases"),
                "total_steps": data.get("total_steps"),
                "sequence": data.get("sequence", []),
                "final_state": data.get("final_state", {}),
            }
        return {"error": data.get("error", "抓取放置失败")}
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})"}


def reset_robot() -> dict:
    """将机器人重置为零位(初始姿态)。"""
    cfg = _cfg()
    try:
        resp = requests.post(f"{cfg.robot_url}/reset", json={}, timeout=5)
        return resp.json()
    except requests.ConnectionError:
        return {"error": f"机器人服务未启动 ({cfg.robot_url})"}


# ══════════════════════════════════════════════════════════════
# 注册所有工具
# ══════════════════════════════════════════════════════════════

def register_all():
    """注册所有可用工具到全局注册表"""
    _tools.clear()

    register(Tool(
        name="analyze_scene",
        description="分析工作场景图像，检测所有物体的位置、类别、颜色和空间关系。必须先调用此工具了解场景中有哪些物体。",
        parameters={
            "type": "object",
            "properties": {
                "image_path": {"type": "string",
                               "description": "场景图像路径，默认 data/workspace_sample.jpg"}
            },
            "required": [],
        },
        function=analyze_scene,
        category="vision",
    ))

    register(Tool(
        name="detect_objects",
        description="检测图像中的目标物体，返回每个物体的边界框、类别标签和置信度。",
        parameters={
            "type": "object",
            "properties": {
                "image_path": {"type": "string", "description": "图像路径"},
                "conf": {"type": "number", "description": "检测置信度阈值，默认0.35"},
            },
            "required": ["image_path"],
        },
        function=detect_objects,
        category="vision",
    ))

    register(Tool(
        name="identify_color",
        description="识别物体颜色(红/橙/黄/绿/蓝/紫/白/黑/灰)，支持指定ROI区域。",
        parameters={
            "type": "object",
            "properties": {
                "image_path": {"type": "string", "description": "图像路径"},
                "roi": {"type": "array", "items": {"type": "number"},
                        "description": "可选，检测区域[x1,y1,x2,y2]"},
            },
            "required": ["image_path"],
        },
        function=identify_color,
        category="vision",
    ))

    register(Tool(
        name="get_robot_status",
        description="查询机器人当前关节角、末端位姿和夹爪状态。在执行任何动作前应先调用此工具了解当前状态。",
        parameters={"type": "object", "properties": {}, "required": []},
        function=get_robot_status,
        category="robot",
    ))

    register(Tool(
        name="move_robot_to",
        description="移动机器人末端到指定三维坐标位置。x前后(200-600mm), y左右(-300-300mm), z上下(100-600mm)。",
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "X坐标(mm)，前后方向"},
                "y": {"type": "number", "description": "Y坐标(mm)，左右方向"},
                "z": {"type": "number", "description": "Z坐标(mm)，上下方向"},
                "rx": {"type": "number", "description": "末端绕X轴姿态角(度)，默认180(夹爪朝下)"},
                "ry": {"type": "number", "description": "绕Y轴姿态角，默认0"},
                "rz": {"type": "number", "description": "绕Z轴姿态角，默认0"},
            },
            "required": ["x", "y", "z"],
        },
        function=move_robot_to,
        category="robot",
    ))

    register(Tool(
        name="pick_object",
        description="抓取物体：完整执行接近→下降→夹紧→抬升动作序列。",
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "目标X坐标(mm)"},
                "y": {"type": "number", "description": "目标Y坐标(mm)"},
                "z": {"type": "number", "description": "目标Z坐标(mm)，物体上表面高度"},
            },
            "required": ["x", "y", "z"],
        },
        function=pick_object,
        category="robot",
    ))

    register(Tool(
        name="place_object",
        description="放置物体：完整执行移动到目标→下降→松开→抬升动作序列。",
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "放置目标X坐标(mm)"},
                "y": {"type": "number", "description": "放置目标Y坐标(mm)"},
                "z": {"type": "number", "description": "放置目标Z坐标(mm)"},
            },
            "required": ["x", "y", "z"],
        },
        function=place_object,
        category="robot",
    ))

    register(Tool(
        name="pick_and_place",
        description="一步完成抓取+放置。从指定位置抓取物体后移动到目标位置放下。首选用于抓放任务。",
        parameters={
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "抓取位置X(mm)"},
                "y": {"type": "number", "description": "抓取位置Y(mm)"},
                "z": {"type": "number", "description": "抓取位置Z(mm)，物体上表面"},
                "target_x": {"type": "number", "description": "放置位置X(mm)"},
                "target_y": {"type": "number", "description": "放置位置Y(mm)"},
                "target_z": {"type": "number", "description": "放置位置Z(mm)，通常略高于台面"},
            },
            "required": ["x", "y", "z", "target_x", "target_y", "target_z"],
        },
        function=pick_and_place,
        category="robot",
    ))

    register(Tool(
        name="reset_robot",
        description="重置机器人到初始零位状态。当操作出错或需要从头开始时调用。",
        parameters={"type": "object", "properties": {}, "required": []},
        function=reset_robot,
        category="robot",
    ))


register_all()
