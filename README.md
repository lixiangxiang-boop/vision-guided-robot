# 视觉引导机器人智能操控系统

> Vision-Guided Robot Intelligent Control System

用自然语言操控六轴机器人完成视觉引导抓取任务。

**在线 Demo**：[http://fanuc.sale](http://fanuc.sale)

```
用户: "把红色工件抓起来放到左边托盘"
  → Agent 理解意图
  → 调用视觉服务分析场景 (YOLO + 颜色识别)
  → 调用机器人服务规划轨迹 (IK + PTP/LIN)
  → 3D 仿真执行
  → 返回结果
```

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    Web UI                                 │
│          Three.js 3D 可视化 + 对话控制台                   │
│          模式切换: Agent / 点动 / 程序                     │
└────────────┬─────────────────────────────────────────────┘
             │ SSE 流式
┌────────────▼─────────────────────────────────────────────┐
│               Agent Service (8803)                        │
│       ReAct 循环: Thought → Action → Observation          │
│       DeepSeek API + Function Calling                     │
└─────┬──────────────────────────────────┬─────────────────┘
      │                                  │
┌─────▼──────────┐              ┌───────▼──────────────────┐
│ Vision (8801)  │              │    Robot (8802)           │
│ YOLOv8 + OpenCV│              │ DH & URDF Kinematics     │
│ 检测·颜色·关系 │              │ DLS IK + PTP/LIN 轨迹     │
│ 像素→世界坐标  │              │ 抓取/放置 动作序列        │
└────────────────┘              └──────────────────────────┘
```

## 快速启动

### 方式一：一键启动

**Windows：**
```bat
start.bat
```

**Linux/Mac：**
```bash
bash start.sh
```

### 方式二：分别启动

```bash
# 安装依赖
pip install -r requirements.txt

# 启动视觉服务 (端口 8801)
python services/vision_service.py

# 启动机器人服务 (端口 8802)
python services/robot_service.py

# 启动 Agent 服务 (端口 8803)
python services/agent_service.py

# 打开 Web UI
start web/index.html
```

### 方式三：Docker

```bash
docker compose up -d
# 访问 http://localhost:8800
```

### 方式四：生产部署（宝塔面板）

```bash
# 1. 上传项目到 /www/wwwroot/your-domain/
# 2. 配置环境变量
cp deploy/.env.production .env
vim .env  # 填入 DEEPSEEK_API_KEY

# 3. 安装依赖（CPU 版 torch，节省空间）
pip3 install torch --index-url https://download.pytorch.org/whl/cpu
pip3 install -r requirements.txt

# 4. 启动服务
bash deploy/start-server.sh start

# 5. Nginx 反向代理（参考 deploy/nginx-baota.conf）
#    将 /api/agent/ → 127.0.0.1:8803
#        /api/vision/ → 127.0.0.1:8801
#        /api/robot/  → 127.0.0.1:8802
#    root 指向 web/ 目录
```

## API 接口

### Vision Service (`http://127.0.0.1:8801`)

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/analyze` | 综合场景分析 (检测 + 颜色 + 空间关系) |
| POST | `/detect` | 目标检测 (YOLOv8) |
| POST | `/color` | 颜色识别 (OpenCV HSV) |
| GET  | `/health` | 健康检查 |

### Robot Service (`http://127.0.0.1:8802`)

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/fk` | 正运动学 (关节角 → 位姿) |
| POST | `/ik` | 逆运动学 (位姿 → 关节角) |
| POST | `/plan` | 轨迹规划 (PTP/LIN) |
| POST | `/execute` | 执行动作序列 (pick/place/move) |
| POST | `/jog` | 点动控制 (关节/笛卡尔) |
| POST | `/reset` | 重置机器人零位 |
| GET  | `/status` | 当前状态 |
| GET  | `/health` | 健康检查 |

### Agent Service (`http://127.0.0.1:8803`)

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/chat` | 发送自然语言指令 |
| GET  | `/chat/stream?message=...` | SSE 流式执行 |
| GET  | `/tools` | 列出可用工具 |
| GET  | `/health` | 健康检查 + 依赖状态 |

## 工具系统

Agent 注册了 9 个工具：

**视觉工具**:
- `analyze_scene` — 分析场景中的物体 (位置/颜色/空间关系/世界坐标)
- `detect_objects` — 目标检测
- `identify_color` — 颜色识别

**机器人工具**:
- `get_robot_status` — 查询当前状态
- `move_robot_to` — 移动机器人到指定位置
- `pick_object` — 抓取物体 (接近→下降→夹紧→抬升)
- `place_object` — 放置物体 (移动→下降→松开→抬升)
- `pick_and_place` — 组合抓放，标准任务首选
- `reset_robot` — 重置零位

## 测试

```bash
# 单元测试 (19个)
python -m pytest tests/test_pipeline.py -v

# 集成测试 (需要启动所有服务)
python tests/test_pipeline.py --integration
```

## 项目结构

```
vision-guided-robot/
├── services/
│   ├── vision_service.py    # 视觉 HTTP API
│   ├── robot_service.py     # 机器人 HTTP API
│   └── agent_service.py     # Agent 编排 API
├── agent/
│   ├── config.py            # Agent 配置 (env 读取)
│   ├── react_loop.py        # ReAct 核心循环
│   └── tools.py             # 9 个工具注册与执行
├── web/
│   ├── index.html           # 主界面 (模式切换)
│   ├── css/style.css        # 工业风格样式
│   └── js/app.js            # Three.js + SSE 前端 (~2000行)
├── deploy/
│   ├── .env.production      # 生产环境变量模板
│   ├── nginx-baota.conf     # 宝塔 Nginx 配置
│   ├── start-server.sh      # 服务管理脚本
│   └── supervisor.ini       # 进程守护配置
├── models/lrmate-200id/     # FANUC 机器人模型
│   ├── meshes/              # STL 网格
│   ├── urdf/                # URDF 模型
│   └── python/              # 参考运动学库
├── tests/
│   └── test_pipeline.py     # 单元 + 集成测试
├── .env.example             # 环境变量模板
├── requirements.txt         # Python 依赖
├── docker-compose.yml       # Docker 编排
├── start.bat / start.sh     # 一键启动脚本
├── README.md
└── DEEP_ANALYSIS.md         # 深度技术解析文档
```

## 技术栈

- **视觉**: OpenCV, YOLOv8, NumPy
- **机器人**: DH 参数法 + URDF 层级法, 阻尼最小二乘 IK, 梯形速度轨迹规划
- **Agent**: ReAct 模式, DeepSeek API, Function Calling
- **前端**: Three.js, SSE, Vanilla JS (模块化)
- **后端**: Flask, Flask-CORS
- **部署**: Nginx 反向代理, Baota Supervisor, 宝塔面板

## 深度解析

完整的技术解析文档见 [DEEP_ANALYSIS.md](DEEP_ANALYSIS.md)，涵盖：

- 是什么 (系统架构、数据流、目录结构)
- 为什么 (7 个关键设计决策的深度分析)
- 怎么做 (7 个核心模块的实现细节)
- 完整测试文档 (单元测试、集成测试、手动测试脚本)

## 后续开发方向

- [ ] MuJoCo 物理引擎集成 (碰撞检测 + 力反馈)
- [ ] 真实相机采集替换模拟图像
- [ ] 多机器人协作
- [ ] 力控/阻抗控制
- [ ] ROS2 集成
