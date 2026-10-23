#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# 视觉引导机器人操控系统 — 服务端启动脚本
# ═══════════════════════════════════════════════════════════════
# 用法:
#   chmod +x deploy/start-server.sh
#   ./deploy/start-server.sh            # 启动所有服务
#   ./deploy/start-server.sh stop       # 停止所有服务
#   ./deploy/start-server.sh status     # 查看服务状态
# ═══════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PID_DIR="$PROJECT_DIR/.pids"
LOG_DIR="$PROJECT_DIR/logs"

mkdir -p "$PID_DIR" "$LOG_DIR"

# 加载 .env 文件
if [ -f "$PROJECT_DIR/.env" ]; then
    export $(grep -v '^#' "$PROJECT_DIR/.env" | grep -v '^$' | xargs)
fi

VISION_PORT=${VISION_PORT:-8801}
ROBOT_PORT=${ROBOT_PORT:-8802}
AGENT_PORT=${AGENT_PORT:-8803}

start_service() {
    local name=$1
    local port=$2
    local cmd=$3
    local pidfile="$PID_DIR/$name.pid"

    if [ -f "$pidfile" ] && kill -0 $(cat "$pidfile") 2>/dev/null; then
        echo "  [$name] 已在运行 (PID: $(cat $pidfile), 端口: $port)"
        return
    fi

    echo "  [$name] 启动中 (端口: $port)..."
    cd "$PROJECT_DIR"
    nohup $cmd > "$LOG_DIR/$name.log" 2> "$LOG_DIR/$name-error.log" &
    echo $! > "$pidfile"
    sleep 1
    if kill -0 $(cat "$pidfile") 2>/dev/null; then
        echo "  [$name] 启动成功 (PID: $(cat $pidfile))"
    else
        echo "  [$name] 启动失败，查看日志: $LOG_DIR/$name-error.log"
    fi
}

stop_service() {
    local name=$1
    local pidfile="$PID_DIR/$name.pid"

    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile")
        if kill -0 $pid 2>/dev/null; then
            echo "  [$name] 停止中 (PID: $pid)..."
            kill $pid
            sleep 1
            kill -9 $pid 2>/dev/null || true
        fi
        rm -f "$pidfile"
        echo "  [$name] 已停止"
    else
        # 通过端口查找并停止
        local port=$2
        local pids=$(lsof -ti :$port 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "  [$name] 停止中 (端口: $port, PID: $pids)..."
            kill $pids 2>/dev/null || true
            echo "  [$name] 已停止"
        else
            echo "  [$name] 未运行"
        fi
    fi
}

status_service() {
    local name=$1
    local port=$2
    local pidfile="$PID_DIR/$name.pid"

    if [ -f "$pidfile" ] && kill -0 $(cat "$pidfile") 2>/dev/null; then
        echo "  [$name] ✅ 运行中 (PID: $(cat $pidfile), 端口: $port)"
    elif curl -s "http://127.0.0.1:$port/health" > /dev/null 2>&1; then
        echo "  [$name] ✅ 运行中 (端口: $port, 无PID文件)"
    else
        echo "  [$name] ❌ 未运行"
    fi
}

case "${1:-start}" in
    start)
        echo "════════════════════════════════════════"
        echo "  启动视觉引导机器人服务..."
        echo "════════════════════════════════════════"

        # 检查 Python
        if ! command -v python3 &> /dev/null; then
            echo "❌ 未找到 python3，请先安装 Python 3.10+"
            exit 1
        fi

        # 安装依赖 (首次运行)
        if ! python3 -c "import flask" 2>/dev/null; then
            echo "  安装 Python 依赖..."
            pip3 install -r "$PROJECT_DIR/requirements.txt" -q
        fi

        start_service "vision" $VISION_PORT \
            "python3 services/vision_service.py --port $VISION_PORT"
        sleep 2

        start_service "robot" $ROBOT_PORT \
            "python3 services/robot_service.py --port $ROBOT_PORT"
        sleep 2

        start_service "agent" $AGENT_PORT \
            "python3 services/agent_service.py --port $AGENT_PORT"
        sleep 2

        echo "════════════════════════════════════════"
        echo "  服务启动完成！"
        echo ""
        echo "  健康检查:"
        echo "    Vision: http://127.0.0.1:$VISION_PORT/health"
        echo "    Robot:  http://127.0.0.1:$ROBOT_PORT/health"
        echo "    Agent:  http://127.0.0.1:$AGENT_PORT/health"
        echo "════════════════════════════════════════"
        ;;

    stop)
        echo "停止所有服务..."
        stop_service "agent" $AGENT_PORT
        stop_service "robot" $ROBOT_PORT
        stop_service "vision" $VISION_PORT
        echo "所有服务已停止"
        ;;

    status)
        echo "服务状态:"
        status_service "vision" $VISION_PORT
        status_service "robot" $ROBOT_PORT
        status_service "agent" $AGENT_PORT
        ;;

    restart)
        $0 stop
        sleep 2
        $0 start
        ;;

    *)
        echo "用法: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
