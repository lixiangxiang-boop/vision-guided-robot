#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# 视觉引导机器人操控系统 — 一键启动 (Linux/Mac/Git Bash)
# ═══════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 加载 .env
if [ -f ".env" ]; then
    export $(grep -v '^#' .env | grep -v '^$' | xargs)
fi

VISION_PORT=${VISION_PORT:-8801}
ROBOT_PORT=${ROBOT_PORT:-8802}
AGENT_PORT=${AGENT_PORT:-8803}

PYTHON=$(command -v python3 || command -v python)

echo "=========================================="
echo "  视觉引导机器人智能操控系统"
echo "=========================================="
echo ""

# 安装依赖
echo "[1/4] 安装 Python 依赖..."
$PYTHON -m pip install -r requirements.txt -q 2>/dev/null || true
echo "      完成."

# 启动视觉服务
echo "[2/4] 启动视觉服务 (端口 $VISION_PORT)..."
$PYTHON services/vision_service.py --port $VISION_PORT &
sleep 2

# 启动机器人服务
echo "[3/4] 启动机器人服务 (端口 $ROBOT_PORT)..."
$PYTHON services/robot_service.py --port $ROBOT_PORT &
sleep 2

# 启动 Agent 服务
echo "[4/4] 启动 Agent 服务 (端口 $AGENT_PORT)..."
$PYTHON services/agent_service.py --port $AGENT_PORT &
sleep 3

echo ""
echo "=========================================="
echo "  服务启动完成!"
echo ""
echo "  开发模式: 打开 web/index.html"
echo "  生产模式: http://YOUR_DOMAIN"
echo ""
echo "  Vision: http://127.0.0.1:$VISION_PORT/health"
echo "  Robot:  http://127.0.0.1:$ROBOT_PORT/health"
echo "  Agent:  http://127.0.0.1:$AGENT_PORT/health"
echo "=========================================="
echo ""

trap "kill 0; echo '服务已停止.'" EXIT
wait
