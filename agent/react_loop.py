"""
ReAct Agent 核心循环
==================
Thought → Action → Observation 循环引擎。
将 LLM 从"聊天机器人"变成"能自主完成抓取任务的智能体"。

系统提示词专为视觉引导抓取场景设计:
  - 理解自然语言指令
  - 分解为视觉分析 + 机器人动作
  - 调用视觉/机器人工具执行
  - 多步推理: 先看→再想→再动
"""
import json
import logging
from dataclasses import dataclass, field
from typing import Generator

from .config import get_config, AgentConfig
from .tools import get_schemas, execute as execute_tool

logger = logging.getLogger("agent.react")

SYSTEM_PROMPT = """你是 FANUC LR Mate 200iD 六轴工业机器人智能控制系统，配备虚拟工业相机进行视觉引导。

## 坐标约定
- analyze_scene 返回的 world_xyz = [X, Y, Z] 采用视觉约定: X=前后, Y=左右, Z=上下 (单位mm)
- 所有机器人工具 (pick_and_place / pick_object / place_object / move_robot_to) 接收世界坐标
- 工具内部自动完成世界坐标→机器人基座坐标系的变换，你无需手动换算

## 工作单元配置 (单位: 毫米)
- 机器人基座: 安装于工作台左侧，世界坐标 (X=-90, Y=0, Z=340)，即底座顶面
- 底座地面法兰高12mm，底座柱体高340mm，总高约350mm
- 工作台: X: 200-550, Y: -180-180 (左右), Z=210mm (台面高度)
- 工件区域: X: 280-520, Y: -150-150, Z≈228mm (台面+工件半高)
- 托盘位置 (世界坐标): Tray A (X=220, Y=-120, Z=215), Tray B (X=400, Y=0, Z=215), Tray C (X=220, Y=120, Z=215)
- 机器人工作半径约700mm，覆盖整个台面，末端执行器: 平行气动夹爪

## 标准操作流程 (SOP)

### 场景感知
- 若上下文提供了场景图像路径，analyze_scene 必须传入该路径: analyze_scene(image_path="上下文中的路径")
- analyze_scene (1次) 返回: 物体列表含 world_xyz 世界坐标 + 颜色标签 + 类别 + 空间关系
- 严禁重复调用 analyze_scene / detect_objects / identify_color

### 抓取-放置任务 (标准3步)
1. analyze_scene → 获取目标物体的 world_xyz 世界坐标和颜色信息
2. get_robot_status → 确认关节角/末端位姿/夹爪状态，若关节全为0则先调用 reset_robot 复位
3. pick_and_place(x=world_xyz[0], y=world_xyz[1], z=world_xyz[2], target_x, target_y, target_z) → 一步完成

### 安全规则
- 执行抓取前确认目标在世界工作范围内 (X: 200-550, Y: -180-180, Z: 180-400)
- Z坐标指的是物体上表面高度，通常台面工件 Z≈228mm
- IK 求解失败时，在目标点±30mm范围内尝试替代点
- 连续失败 3 次停止并报告详细原因（关节限位/奇异点/工作范围）
- 放置目标点 Z 坐标应略高于台面 (Z≥220mm)，避免碰撞

## 专业输出规范
- 回复中报告: 目标物体颜色+类别、检测到的世界坐标、目标放置位置
- 任务完成时输出格式:
  ```
  [视觉检测] 识别到{颜色}{类别}，世界坐标 ({X}, {Y}, {Z})，置信度 {conf}%
  [坐标变换] 世界→机器人基座坐标系变换完成
  [逆运动学] IK求解成功，关节角 [{J1}°, {J2}°, {J3}°, {J4}°, {J5}°, {J6}°]
  [轨迹规划] 规划 {steps} 步 PTP 轨迹，平滑度良好
  [执行] 抓取→抬升→移动→放置 完成，指令已通过虚拟 EtherCAT 接口发送
  ```

## 可用工具
- analyze_scene: 综合视觉分析(检测+颜色+空间+世界坐标) — 首选，一次搞定
- detect_objects: 纯目标检测 — 仅调整参数时用
- identify_color: 纯颜色识别 — 仅精确确认时用
- get_robot_status: 查询关节角/末端位姿/夹爪状态
- move_robot_to: 移动末端到世界坐标 (x=前后, y=左右, z=上下) mm
- pick_object: 抓取世界坐标处的物体 (接近→下降→闭合→抬升)
- place_object: 放置物体到世界坐标 (移动→下降→打开→抬升)
- pick_and_place: 组合抓放 — 标准任务首选，参数均为世界坐标
- reset_robot: 复位到零位
"""


@dataclass
class AgentEvent:
    type: str  # thinking | tool_call | tool_result | final | error
    content: str = ""
    meta: dict = field(default_factory=dict)


def _trim_tool_result(result_str: str, max_len: int = 4000) -> str:
    """剔除工具结果中的长base64数据，节省LLM上下文空间"""
    import re
    # 将 annotated_b64 / mask_b64 的值替换为长度标记
    result_str = re.sub(
        r'"annotated_b64":\s*"[^"]{100,}"',
        f'"annotated_b64": "[BASE64_DATA_{len(result_str)}chars_removed]"',
        result_str)
    result_str = re.sub(
        r'"mask_b64":\s*"[^"]{100,}"',
        f'"mask_b64": "[BASE64_DATA_removed]"',
        result_str)
    # 截断到 max_len，保留末尾以捕获关键信息
    if len(result_str) > max_len:
        result_str = result_str[:max_len - 100] + f"...[truncated {len(result_str) - max_len + 100} chars]"
    return result_str


class VisionRobotAgent:
    """视觉引导机器人 Agent — ReAct 循环"""

    def __init__(self, config: AgentConfig | None = None):
        self.cfg = config or get_config()
        self._client = None
        self._history: list[dict] = []

    @property
    def client(self):
        if self._client is None:
            from openai import OpenAI
            self._client = OpenAI(api_key=self.cfg.api_key, base_url=self.cfg.base_url)
        return self._client

    def chat(self, user_input: str, image_path: str = "") -> Generator[AgentEvent, None, None]:
        """
        执行 ReAct 循环处理用户指令。

        流式返回 AgentEvent:
          thinking → tool_call → tool_result → ... → final
        """
        # 构建消息
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]

        # 注入上下文
        context = f"[用户指令] {user_input}"
        if image_path:
            context += f"\n[场景图像路径] {image_path}"
        messages.append({"role": "user", "content": context})

        tools = get_schemas()

        yield AgentEvent(type="system", content=f"收到指令: {user_input}")

        for _round in range(self.cfg.max_rounds):
            try:
                response = self.client.chat.completions.create(
                    model=self.cfg.model,
                    messages=messages,
                    tools=tools or None,
                    temperature=self.cfg.temperature,
                    max_tokens=self.cfg.max_tokens,
                )
            except Exception as e:
                logger.exception("LLM call failed")
                yield AgentEvent(type="error", content=f"LLM 调用失败: {e}")
                break

            msg = response.choices[0].message
            finish_reason = response.choices[0].finish_reason

            # 处理 thinking (DeepSeek reasoning_content)
            if hasattr(msg, "reasoning_content") and msg.reasoning_content:
                yield AgentEvent(type="thinking", content=msg.reasoning_content)

            # 处理 tool_calls
            if msg.tool_calls:
                for tc in msg.tool_calls:
                    tool_name = tc.function.name
                    try:
                        tool_args = json.loads(tc.function.arguments)
                    except json.JSONDecodeError:
                        tool_args = {}

                    yield AgentEvent(type="tool_call", content=tool_name, meta={
                        "name": tool_name, "args": tool_args})

                    result = execute_tool(tool_name, tool_args)
                    result_str = json.dumps(result, ensure_ascii=False) if isinstance(result, dict) else str(result)
                    yield AgentEvent(type="tool_result", content=result_str,
                                     meta={"name": tool_name})

                    # 追加到对话历史 (剔除长base64数据以节省LLM上下文)
                    llm_context = _trim_tool_result(result_str)
                    messages.append({
                        "role": "assistant",
                        "content": msg.content or "",
                        "tool_calls": [{
                            "id": tc.id, "type": "function",
                            "function": {"name": tool_name,
                                         "arguments": tc.function.arguments},
                        }],
                    })
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tc.id,
                        "content": llm_context,
                    })

            # 处理纯文本回复
            elif msg.content:
                yield AgentEvent(type="final", content=msg.content)
                break

            # finish_reason == "stop" 且无 content 时
            if finish_reason == "stop" and not msg.tool_calls:
                break

            if finish_reason == "length":
                yield AgentEvent(type="final",
                                 content="(达到输出上限) 任务可能未完成，请分步执行。")
                break

        else:
            yield AgentEvent(type="error",
                             content=f"达到最大轮次 ({self.cfg.max_rounds})，任务可能未完成。")
