@echo off
chcp 65001 >nul
title 视觉引导机器人操控系统

echo ==========================================
echo   视觉引导机器人智能操控系统
echo   Vision-Guided Robot Control System
echo ==========================================
echo.

:: 检查 Python
where python >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 Python，请先安装 Python 3.10+
    pause
    exit /b 1
)

:: 安装依赖
echo [1/4] 安装 Python 依赖...
python -m pip install -r requirements.txt -q
echo       完成.

:: 启动视觉服务
echo [2/4] 启动视觉服务 (端口 8801)...
start "Vision Service" python services\vision_service.py --port 8801
timeout /t 2 >nul

:: 启动机器人服务
echo [3/4] 启动机器人服务 (端口 8802)...
start "Robot Service" python services\robot_service.py --port 8802
timeout /t 2 >nul

:: 启动 Agent 服务
echo [4/4] 启动 Agent 服务 (端口 8803)...
start "Agent Service" python services\agent_service.py --port 8803
timeout /t 3 >nul

echo.
echo ==========================================
echo   服务启动完成！
echo.
echo   打开浏览器访问:
echo     Web UI: 直接打开 web/index.html
echo     Vision: http://127.0.0.1:8801/health
echo     Robot:  http://127.0.0.1:8802/health
echo     Agent:  http://127.0.0.1:8803/health
echo ==========================================
echo.
echo 按任意键退出所有服务...
pause >nul

:: 清理
taskkill /FI "WINDOWTITLE eq Vision Service" /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq Robot Service" /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq Agent Service" /F >nul 2>&1
echo 服务已停止.
