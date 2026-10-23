"""
Agent Service — Agent 编排服务层
================================
统一的 Agent HTTP API + WebSocket 流式推送。
前端通过此服务与整个系统交互 — 一条自然语言指令驱动视觉+机器人全流程。

API:
  POST /chat              发送指令，返回完整执行过程
  GET  /chat/stream       SSE 流式推送执行过程 (实时展示)
  GET  /tools              列出可用工具
  GET  /health             健康检查
"""
import sys
import os
import json
import logging

_here = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from flask import Flask, request, jsonify, Response, stream_with_context
from flask_cors import CORS

from agent.config import get_config
from agent.react_loop import VisionRobotAgent, AgentEvent
from agent.tools import get_all as get_all_tools, get_schemas

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("agent_service")

app = Flask(__name__)
CORS(app)

agent = VisionRobotAgent()


@app.route("/chat", methods=["POST"])
def chat():
    """
    发送指令: { "message": "把红色工件抓起来放到左边托盘", "image_path": "" }
    返回完整执行过程 (非流式)。
    """
    try:
        data = request.get_json(force=True)
        if not data or "message" not in data:
            return jsonify({"error": "需要 message 字段"}), 400

        message = data["message"]
        image_path = data.get("image_path", "")

        events = []
        for event in agent.chat(message, image_path):
            events.append({
                "type": event.type,
                "content": event.content,
                "meta": event.meta,
            })

        # 提取最终答案
        final_answer = ""
        for e in events:
            if e["type"] == "final":
                final_answer = e["content"]
                break

        return jsonify({
            "success": True,
            "message": message,
            "events": events,
            "event_count": len(events),
            "final_answer": final_answer,
        })

    except Exception as e:
        logger.exception("chat error")
        return jsonify({"error": str(e)}), 500


@app.route("/chat/stream", methods=["GET"])
def chat_stream():
    """
    SSE 流式推送: GET /chat/stream?message=抓取红色工件
    实时展示 Agent 思考 → 工具调用 → 执行结果 → 最终回复。
    """
    message = request.args.get("message", "")
    image_path = request.args.get("image_path", "")

    if not message:
        return jsonify({"error": "需要 message 参数"}), 400

    def generate():
        for event in agent.chat(message, image_path):
            data = json.dumps({
                "type": event.type,
                "content": event.content,
                "meta": event.meta,
            }, ensure_ascii=False)
            yield f"data: {data}\n\n"

    return Response(
        stream_with_context(generate()),
        mimetype="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@app.route("/capture", methods=["POST"])
def capture_virtual_frame():
    """
    接收前端虚拟工业相机的离屏渲染帧 (base64) 并保存为临时文件。
    前端 POST: { "image_b64": "<base64 data>" }
    """
    import base64
    try:
        data = request.get_json(force=True)
        img_b64 = data.get("image_b64", "")
        if not img_b64:
            return jsonify({"error": "需要 image_b64 字段"}), 400
        # Strip data URI prefix if present
        if "," in img_b64:
            img_b64 = img_b64.split(",", 1)[1]
        os.makedirs("data", exist_ok=True)
        with open("data/virtual_frame.jpg", "wb") as f:
            f.write(base64.b64decode(img_b64))
        return jsonify({"status": "ok", "path": "data/virtual_frame.jpg"})
    except Exception as e:
        logger.exception("capture error")
        return jsonify({"error": str(e)}), 500


@app.route("/tools", methods=["GET"])
def tools():
    """列出所有可用工具"""
    all_tools = get_all_tools()
    return jsonify({
        "count": len(all_tools),
        "tools": [
            {
                "name": t.name,
                "description": t.description,
                "category": t.category,
                "parameters": t.parameters,
            }
            for t in all_tools
        ],
    })


@app.route("/health", methods=["GET"])
def health():
    """健康检查 + 依赖服务状态"""
    config = get_config()
    vision_ok = False
    robot_ok = False
    try:
        import requests as _r
        vr = _r.get(f"{config.vision_url}/health", timeout=2)
        vision_ok = vr.status_code == 200
        rr = _r.get(f"{config.robot_url}/health", timeout=2)
        robot_ok = rr.status_code == 200
    except Exception:
        pass

    return jsonify({
        "status": "ok",
        "service": "agent",
        "model": config.model,
        "dependencies": {
            "vision": "connected" if vision_ok else "disconnected",
            "robot": "connected" if robot_ok else "disconnected",
        },
        "tools_loaded": len(get_all_tools()),
    })


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8803)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logger.info("Agent Service starting on port %d", args.port)
    app.run(host="0.0.0.0", port=args.port, debug=args.debug)
