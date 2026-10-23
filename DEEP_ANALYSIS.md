# 视觉引导工业机器人智能操控系统 — 深度解析

> 以"是什么 / 为什么 / 怎么做"为切入点，整合完整测试说明

---

## 目录

- [Part A: 是什么 (What)](#part-a-是什么-what)
  - [A.1 项目定义](#a1-项目定义)
  - [A.2 系统架构](#a2-系统架构)
  - [A.3 技术栈全景图](#a3-技术栈全景图)
  - [A.4 核心数据流](#a4-核心数据流)
  - [A.5 目录结构](#a5-目录结构)
- [Part B: 为什么 (Why)](#part-b-为什么-why)
  - [B.1 为什么要用 ReAct Agent 模式](#b1-为什么要用-react-agent-模式)
  - [B.2 为什么要建立双运动学模型 (DH + URDF)](#b2-为什么要建立双运动学模型-dh--urdf)
  - [B.3 为什么要用数值雅可比 + 阻尼最小二乘法求 IK](#b3-为什么要用数值雅可比--阻尼最小二乘法求-ik)
  - [B.4 为什么选择 SSE 而不是 WebSocket 或轮询](#b4-为什么选择-sse-而不是-websocket-或轮询)
  - [B.5 为什么 Three.js 用 Y-up 但机器人服务用 Z-up](#b5-为什么-threejs-用-y-up-但机器人服务用-z-up)
  - [B.6 为什么要做虚拟工业相机 + 离屏渲染](#b6-为什么要做虚拟工业相机--离屏渲染)
  - [B.7 为什么用鲁棒多候选 IK 而非单次求解](#b7-为什么用鲁棒多候选-ik-而非单次求解)
- [Part C: 怎么做 (How)](#part-c-怎么做-how)
  - [C.1 Agent ReAct 循环引擎实现](#c1-agent-react-循环引擎实现)
  - [C.2 双模型运动学系统实现](#c2-双模型运动学系统实现)
  - [C.3 视觉检测管线实现](#c3-视觉检测管线实现)
  - [C.4 坐标转换链实现](#c4-坐标转换链实现)
  - [C.5 3D 数字孪生前端实现](#c5-3d-数字孪生前端实现)
  - [C.6 点动控制与示教编程实现](#c6-点动控制与示教编程实现)
  - [C.7 多模式 UI 架构实现](#c7-多模式-ui-架构实现)
- [Part D: 测试文档](#part-d-测试文档)
  - [D.1 测试架构总览](#d1-测试架构总览)
  - [D.2 单元测试详细说明](#d2-单元测试详细说明)
  - [D.3 集成测试详细说明](#d3-集成测试详细说明)
  - [D.4 手动测试脚本](#d4-手动测试脚本)
  - [D.5 性能测试与基准](#d5-性能测试与基准)

---

## Part A: 是什么 (What)

### A.1 项目定义

**视觉引导工业机器人智能操控系统**是一款面向工业自动化场景的 **数字孪生 + AI Agent 操控平台**。它实现了"用自然语言操控六轴机器人完成视觉引导抓取任务"的完整闭环。

核心能力：

```
用户: "把红色工件抓起来放到左边托盘"
  → Agent 理解意图 (LLM 自然语言推理)
  → 调用视觉服务分析场景 (YOLOv8 检测 + HSV 颜色识别)
  → 像素坐标 → 世界坐标转换
  → 调用机器人服务求解逆运动学 (DLS IK)
  → 规划点到点轨迹 (梯形速度插补)
  → 3D 数字孪生实时动画执行
  → 返回专业格式结果
```

### A.2 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    Web UI (Three.js 3D + SSE)                     │
│         模式切换: Agent 控制台 / 点动控制 / 程序编辑器             │
│         端口: 开发直连 localhost, 生产通过 Nginx /api/* 代理       │
└──────────────────────────┬──────────────────────────────────────┘
                           │ SSE 流式 (text/event-stream)
┌──────────────────────────▼──────────────────────────────────────┐
│               Agent Service (Flask, 端口 8803)                    │
│     ReAct 循环引擎: Thought(推理) → Action(工具调用) →             │
│     Observation(结果观察) → 循环直到任务完成                       │
│     DeepSeek API (OpenAI 兼容 Function Calling)                   │
│     系统提示词: 工业机器人 SOP + 坐标约定 + 安全规则               │
└───────┬──────────────────────────────────┬───────────────────────┘
        │ REST/JSON                        │ REST/JSON
┌───────▼──────────┐              ┌────────▼───────────────────────┐
│ Vision Service   │              │    Robot Service                │
│ (Flask, 8801)    │              │    (Flask, 8802)               │
│                  │              │                                │
│ YOLOv8n 目标检测 │              │ DH 参数模型 (6-DOF IK/FK)      │
│ HSV 颜色识别     │              │ URDF 层级模型 (匹配3D场景)      │
│ 空间关系分析     │              │ 数值雅可比 DLS IK              │
│ 综合场景分析     │              │ 鲁棒多候选 IK 求解器            │
│ 像素→世界坐标    │              │ PTP 梯形速度轨迹规划            │
│                  │              │ Pick/Place 动作序列编排         │
│ OpenCV 轮廓回退  │              │ 点动控制 (关节/笛卡尔)          │
└──────────────────┘              └────────────────────────────────┘
```

**三层服务解耦设计**：视觉层只负责"看"，机器人层只负责"动"，Agent 层负责"想"。各层通过 HTTP API 通信，可独立开发、独立测试、独立部署。

### A.3 技术栈全景图

| 层级 | 技术 | 版本/说明 |
|------|------|-----------|
| **3D 渲染** | Three.js | 0.160.0, ES Module import |
| **3D 模型** | STL 网格 (FANUC LR Mate 200iD) | convex/low-res/med-res 三套精度 |
| **相机控制** | OrbitControls | 旋转/缩放/平移 |
| **视觉 AI** | YOLOv8n (ultralytics) | PyTorch, GPU/CPU 自适应 |
| **传统视觉** | OpenCV (cv2) | HSV 颜色空间, 轮廓检测, 形态学 |
| **运动学** | DH 参数法 + URDF 层级法 | 双模型并行, Rodrigues 旋转 |
| **数值 IK** | DLS (阻尼最小二乘法) | 3x6 位置雅可比 + 6x6 全雅可比 |
| **轨迹规划** | 梯形速度曲线 (PTP) | 加速-匀速-减速三段式 |
| **Agent 引擎** | ReAct 模式 (VisionRobotAgent) | Thought→Action→Observation 循环 |
| **LLM** | DeepSeek Chat API | OpenAI 兼容 Function Calling |
| **流式传输** | SSE (Server-Sent Events) | text/event-stream |
| **后端框架** | Flask + Flask-CORS | 3 个独立服务 |
| **配置管理** | python-dotenv | .env 文件 + 环境变量 |
| **部署** | Nginx 反向代理 + Supervisor | 宝塔面板兼容 |

### A.4 核心数据流

一条自然语言指令的完整生命周期：

```
[用户输入] "把红色工件抓起来放到左边托盘"
     │
     ▼
┌─────────────────────────────────────────────────────┐
│ Step 1: ReAct Loop 启动                              │
│   messages = [SystemPrompt, UserContext]             │
│   yield "system" event → 前端显示"收到指令"           │
└──────────────┬──────────────────────────────────────┘
               ▼
┌─────────────────────────────────────────────────────┐
│ Step 2: LLM 推理 (Thought)                           │
│   DeepSeek 返回: tool_calls=[analyze_scene]          │
│   reasoning_content → 前端显示 Agent 思考过程         │
│   yield "thinking" event                             │
└──────────────┬──────────────────────────────────────┘
               ▼
┌─────────────────────────────────────────────────────┐
│ Step 3: 工具调用 — analyze_scene                     │
│   GET robot/capture → 前端渲染离屏帧 → base64        │
│   POST vision/analyze {image_b64}                   │
│   YOLO 检测: 4个工件, 分类, BBox, 置信度             │
│   HSV 颜色: 红色/蓝色/绿色/黄色                       │
│   像素→世界坐标: (px,py)→(wx,wy,wz) mm               │
│   yield "tool_result" event                          │
└──────────────┬──────────────────────────────────────┘
               ▼
┌─────────────────────────────────────────────────────┐
│ Step 4: LLM 再次推理                                 │
│   阅读 analyze_scene 结果                            │
│   决定: pick_and_place(                               │
│     x=380, y=20, z=228,    ← 红色工件位置             │
│     target_x=220, target_y=-120, target_z=215        │
│   )                                                  │
│   yield "tool_call" event                            │
└──────────────┬──────────────────────────────────────┘
               ▼
┌─────────────────────────────────────────────────────┐
│ Step 5: 机器人执行 — pick_and_place                   │
│   POST robot/execute {action:"pick_and_place",...}   │
│   ┌─ Phase 1: approach (IK求解接近点, 30步轨迹)       │
│   ├─ Phase 2: descend (IK求解下降点, 20步轨迹)        │
│   ├─ Phase 3: gripper close (夹爪关闭)               │
│   ├─ Phase 4: retreat (IK求解抬升点, 30步轨迹)        │
│   ├─ Phase 5: approach_place (移动到放置点上方)       │
│   ├─ Phase 6: descend_place (下降到放置点)            │
│   ├─ Phase 7: gripper open (夹爪打开)                │
│   └─ Phase 8: retreat_place (抬升离开)               │
│   yield "tool_result" event                          │
└──────────────┬──────────────────────────────────────┘
               ▼
┌─────────────────────────────────────────────────────┐
│ Step 6: 前端动画 + 最终回复                           │
│   接收 trajectory 序列 → requestAnimationFrame 插值   │
│   实时更新: 关节角面板 / TCP面板 / 3D机器人姿态        │
│   工件跟随移动 (工件A从抓取点→放置点)                  │
│   yield "final" event → 显示专业格式结果              │
│   自动生成 FANUC TP 程序 (可选)                       │
└─────────────────────────────────────────────────────┘
```

### A.5 目录结构

```
vision-guided-robot/
├── services/                       # 后端微服务 (Flask)
│   ├── vision_service.py           # 视觉检测 API (405行)
│   ├── robot_service.py            # 机器人控制 API (1046行)
│   └── agent_service.py            # Agent 编排 API (190行)
├── agent/                          # Agent 核心引擎
│   ├── __init__.py
│   ├── config.py                   # DeepSeek 配置管理
│   ├── react_loop.py               # ReAct 循环引擎 (213行)
│   └── tools.py                    # 8个工具定义与注册 (479行)
├── web/                            # 前端 (无框架原生 JS)
│   ├── index.html                  # 主界面 (237行)
│   ├── css/style.css               # 工业风格样式
│   └── js/app.js                   # 3D + 交互逻辑 (~2000行)
├── models/lrmate-200id/            # FANUC 机器人模型库
│   ├── meshes/                     # STL 网格 (convex/low/med)
│   ├── urdf/                       # URDF 模型文件
│   ├── python/lrmate/              # 参考 Python 运动学库
│   └── test_cases/                 # FK 测试数据 (JSON)
├── tests/
│   └── test_pipeline.py            # 19 单元测试 + 集成测试 (399行)
├── data/
│   └── workspace_sample.jpg        # 默认场景样本图像
├── deploy/                         # 部署配置 (待生成)
├── .env.example                    # 环境变量模板
├── requirements.txt                # Python 依赖
├── docker-compose.yml              # Docker 编排
├── nginx.conf                      # Nginx 配置
├── start.bat / start.sh            # 一键启动
└── README.md
```

---

## Part B: 为什么 (Why)

### B.1 为什么要用 ReAct Agent 模式

**决策背景**：传统工业机器人编程需要专业工程师编写 G 代码或 TP 程序，即使是一个简单的"抓取红色工件"任务也需要精确测量坐标、计算关节角、逐点示教。

**ReAct (Reasoning + Acting) 模式的核心优势**：

1. **思考与行动交替**：Agent 不会一次性规划全部步骤，而是"想一步→做一步→看结果→调整"。这在机器人操作中至关重要——视觉检测结果会影响后续的抓取策略。

2. **Function Calling 是自然接口**：LLM 不需要理解像素值或力矩计算，它只需要知道"调用 analyze_scene 可以获取物体列表(含颜色和世界坐标)"、"调用 pick_and_place 可以完成抓放"。每步工具调用都有标准 JSON Schema 描述，LLM 按此格式发送请求。

3. **可观测性强**：每个 `AgentEvent` (thinking / tool_call / tool_result / final) 都被 SSE 流式推送到前端，用户可以实时看到 Agent 在"想什么"、"调用什么工具"、"得到什么结果"。这对面试展示和调试都至关重要。

4. **安全约束内置于提示词**：系统提示词明确规定了工作范围、关节限位、SOP 流程、失败处理策略。LLM 在推理时会遵循这些约束，相当于内建了安全控制器。

**对比其他方案**：

| 方案 | 优点 | 缺点 |
|------|------|------|
| 规则引擎 | 确定性高 | 无法理解自然语言的多样性 |
| 端到端强化学习 | 无需建模 | 训练成本高、泛化差、不可解释 |
| 预定义宏指令 | 响应快 | 用户必须记住指令名 |
| **ReAct Agent** | 自然语言理解 + 可解释 + 灵活组合 | LLM 延迟 (1-3s)、Token 成本 |

### B.2 为什么要建立双运动学模型 (DH + URDF)

**决策背景**：机器人运动学有两个截然不同的使用场景——数学求解和 3D 可视化。

**DH 参数模型** (Denavit-Hartenberg)：
- 6 个参数描述相邻关节间的变换关系 (a, alpha, d, theta + offset)
- 标准工业机器人学教材方法，数学形式简洁
- 用于 `robot_service.py` 中的 **6-DOF 全位姿 IK 求解**和 **DH FK**
- 适合做通用运动学计算，领域文献/论文中的标准表述

**URDF 层级模型**：
- 每个关节的 origin (平移) + axis (旋转轴) 直接描述
- 关节变换顺序：`T = T_trans(origin) × R_axis(angle)`，嵌套相乘
- **与 Three.js 前端场景的 robotAssembly 层级完全一致**
- 用于 **3-DOF 位置 IK 求解**和 **FK 位姿验证**

**为什么需要两个模型并存**：

```
DH 模型:  数学上更紧凑，适合做 6-DOF 全位姿 IK (位置+姿态)
          但参数是从"标准DH"文献推导的，关节角含义与3D场景不完全对齐

URDF 模型: 关节角直接对应3D场景中的旋转 (J2 = -90° 手臂前伸)
          但只做位置 IK (3-DOF)，姿态由系统自动选择

实际使用:
  - 前端动画 → URDF FK (确保视觉一致性)
  - IK 求解 → 优先 URDF IK (位置精度高), 回退 DH IK (全位姿)
```

**物理一致性保证**：`robot_service.py` 中的 `urdf_fk_world()` 和 `urdf_fk_pose()` 完整复制了 `app.js` 中 `applyJointsToRobot()` 的嵌套层级变换 + 坐标转换，确保服务端计算的 TCP 位置与前端 3D 渲染的 TCP 位置完全一致。

### B.3 为什么要用数值雅可比 + 阻尼最小二乘法求 IK

**决策背景**：六轴机器人的逆运动学是一个典型的非线性优化问题。虽然有解析解（如 Pieper 算法），但解析解耦合在特定几何结构上，不够通用。

**数值雅可比 (Numerical Jacobian)**：

```
J = ∂f/∂q  通过有限差分计算:
  q_plus[i] = q[i] + δ    (δ = 0.005 rad ≈ 0.3°)
  J[:, i] = (FK(q_plus) - FK(q)) / δ
```

- **通用性**：对任意机器人构型都适用，不需要解析 DH 偏导
- **3×6 或 6×6 矩阵**：3-DOF 位置 IK 用小雅可比，6-DOF 全位姿 IK 用全雅可比

**阻尼最小二乘法 (Damped Least Squares, DLS)**：

```
dq = J^T (J J^T + λ² I)^{-1} error × α
```

- **λ (damping)**：防止奇异点附近雅可比矩阵条件数爆炸
- **α (step size)**：控制每步的关节角变化幅度，避免超调和振荡
- **相比纯牛顿法**：DLS 在 J 接近奇异时仍然稳定，不会产生巨大的关节角跳变

**收敛特性**：
- 目标容差：0.5mm (位置 IK) / 0.01 (归一化误差, 全位姿 IK)
- 最大迭代：600 步 (位置) / 500 步 (全位姿)
- 步长裁剪：单步关节角变化 ≤ 0.3 rad (约 17°)，防止跳变

### B.4 为什么选择 SSE 而不是 WebSocket 或轮询

**决策对比**：

| 特性 | SSE | WebSocket | 轮询 |
|------|-----|-----------|------|
| 方向 | 服务器→客户端 | 双向 | 客户端→服务器 |
| 协议 | HTTP | WS (独立协议) | HTTP |
| Nginx 代理 | 原生支持 | 需额外配置 | 原生支持 |
| 断线重连 | 浏览器自动 | 需手动实现 | N/A |
| 实现复杂度 | 极低 (Flask Response) | 中等 (socketio) | 低 (setInterval) |
| Agent场景适合度 | 完美 (服务器推送事件) | 过度 (无需双向) | 不够 (延迟高) |

**为什么 SSE 是最佳选择**：

1. **Agent 场景天然是单向流**：Agent 思考→工具调用→结果→最终回复，全部是服务器推送事件。前端只需要接收和渲染，不需要向 Agent 发消息（消息通过 POST /chat 发送）。

2. **兼容 Nginx 反向代理**：生产部署时 Nginx 只需 `proxy_buffering off` + `chunked_transfer_encoding on` 即可透传 SSE。WebSocket 需要额外的 Upgrade 头处理。

3. **浏览器原生 EventSource**：前端使用标准 `EventSource` API，断线自动重连，无需第三方库。

4. **实现简洁**：`flask` 的 `Response(stream_with_context(generate()), mimetype="text/event-stream")` 即可。

### B.5 为什么 Three.js 用 Y-up 但机器人服务用 Z-up

**这是一个必须解决的坐标系阻抗匹配问题**：

| 坐标系 | Y轴含义 | Z轴含义 | 使用场景 |
|--------|---------|---------|----------|
| Three.js | 上下 (Up) | 前后/左右 | 3D 渲染引擎默认 |
| 机器人学 (URDF/DH) | 左右 | 上下 (Up) | 工程标准 (ISO) |
| 视觉约定 (用户面向) | 左右 | 上下 | 人类直觉 |

**转换方案**：

```python
# robotAssembly.rotation.set(-Math.PI/2, 0, 0)  # 前端 Three.js
# 这个旋转将 URDF 的 Z-up 转换为 Three.js 的 Y-up

# robot_service.py 中的转换链:
# Step 1: URDF FK → robotAssembly 坐标 (Z-up)
urdf_pos = T @ [0, 0, TCP_OFFSET, 1]  # [ux, uy, uz]

# Step 2: robotAssembly → Three.js world (Rx(-π/2) 旋转)
three_x = ux
three_y = uz + ROBOT_ASSEMBLY_Y    # Z→Y (上)
three_z = -uy                       # Y→Z (取反)

# Step 3: Three.js world → 视觉约定
visual_x = three_x                  # 前后
visual_y = three_z                  # 左右
visual_z = three_y                  # 上下
```

**为什么不在一个坐标系中统一**：尝试过全部转为 Z-up，但 Three.js 生态（OrbitControls、shadow mapping、camera）都假设 Y-up。强行改会破坏 OrbitControls 的旋转行为、光照方向判断等。最终选择"服务端做精确转换，前端保持 Three.js 原生约定"。

### B.6 为什么要做虚拟工业相机 + 离屏渲染

**决策背景**：项目定位为 Demo/Showcase，不一定有真实相机和机器人硬件。但必须展示完整的"视觉引导"闭环——即相机拍摄→检测→坐标→抓取。

**离屏渲染方案**：

```
[Three.js 离屏渲染器]                    [视觉服务]
WebGLRenderer(offscreen: true)      YOLOv8 + OpenCV
       │                                   │
       │ 640×480 canvas                    │
       │ camera.position(350,500,0)        │
       │ camera.lookAt(350,210,0)          │
       │ (俯视工作台, 模拟工业相机)          │
       │                                   │
       ├── canvas.toDataURL("image/jpeg") ─┤
       │     → POST /analyze {image_b64}   │
       │                                   │
       │     ← {objects, colors, ...,       │
       │        annotated_b64}             │
       │                                   │
       ├── 标注图叠加到视觉面板              │
       └── 物体坐标用于 IK 求解              │
```

**为什么不用随机数据**：随机生成的世界坐标与实际 3D 场景中的物体位置不对应，抓取动画和视觉检测会脱节。离屏渲染保证了"画面中看到的"就是"Agent 操作的"，体现了数字孪生的核心价值。

**工业相机模拟**：使用 640×480 分辨率（模拟 30 万像素工业相机），俯视角度模拟 Eye-in-hand 之外的固定安装相机。场景中有 PBR 材质和工业光照，YOLO 可以检测到与真实场景相似的视觉特征。

### B.7 为什么用鲁棒多候选 IK 而非单次求解

**问题**：数值 IK 是局部优化方法，对初始值敏感。如果机器人当前处于奇异构型（如所有关节角为 0），单次 IK 求解大概率发散或陷于较差局部解。

**鲁棒求解策略**：

```python
candidates = [
    saved_joints,          # 当前构型 (最可能接近)
    [0, -90°, 0, 0, 0, 0],      # 标准前伸构型
    [0, -85°, 0, 0, 0, 0],      # 前伸变体
    [0, -95°, -10°, 0, 0, 0],   # 肘部微弯
    [15°, -85°, 0, 0, 0, 0],    # J1 带偏移
    ...共 13 个候选构型
]

for each candidate:
    result = DLS_IK(target, seed=candidate)
    if converged and within_limits:
        score = evaluate(result)  # 偏好 J2≈90°, J3≈0°
        keep best score

if no valid result:
    fallback: first converged result (bypass limits)
```

**评分函数设计**：

| 关节 | 偏好区间 | 评分逻辑 |
|------|---------|----------|
| J2 | -110° ~ -70° | 偏离 -90° 越小越好 |
| J3 | -30° ~ 30° | 偏离 0° 越小越好 |
| J4 | -130°~130° | 越靠近 0° 越好 |
| J1/J5/J6 | 任意 | 越接近 0° 越好 (弱偏好) |

- **J2 偏好 -90°**：对应手臂水平前伸，是大部分抓取操作的自然构型
- **J3 偏好 0°**：前臂顺直延伸，力矩效率最高
- **肘部弯折惩罚**：J4 远离 0° 意味着手腕大幅翻转，评分迅速变差

---

## Part C: 怎么做 (How)

### C.1 Agent ReAct 循环引擎实现

**文件**：[agent/react_loop.py](agent/react_loop.py) (213 行)

**核心类**：`VisionRobotAgent`

**循环流程**：

```python
def chat(self, user_input, image_path="") -> Generator[AgentEvent]:
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},   # 专业SOP + 坐标约定
        {"role": "user", "content": context},            # 用户指令 + 图像路径
    ]

    for round in range(max_rounds=15):          # 安全上限，防止无限循环
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            tools=tool_schemas,                  # 8 个工具的 JSON Schema
            temperature=0.7,
            max_tokens=2048,
        )

        msg = response.choices[0].message

        # 情况1: LLM 决定调用工具
        if msg.tool_calls:
            for tc in msg.tool_calls:
                yield AgentEvent("tool_call", tc.function.name, ...)
                result = execute_tool(tc.function.name, args)
                yield AgentEvent("tool_result", result, ...)
                messages.append(assistant_msg + tool_result_msg)

        # 情况2: LLM 返回最终文本 (任务完成)
        elif msg.content:
            yield AgentEvent("final", msg.content)
            break

        # 情况3: 达到输出上限
        if finish_reason == "length":
            yield AgentEvent("final", "任务可能未完成，请分步执行")
            break

    else:  # 达到 max_rounds
        yield AgentEvent("error", "达到最大轮次")
```

**系统提示词设计**（[react_loop.py:23-78](agent/react_loop.py#L23-L78)）：

提示词的结构化程度直接决定 Agent 的行为质量：

1. **坐标约定** (L25-28)：明确 world_xyz = [X前后, Y左右, Z上下]，消除 LLM 对坐标轴的混淆
2. **工作单元配置** (L30-36)：精确给出机器人基座位置、工作台范围、工件区域、托盘位置，相当于"给 LLM 一份工厂布局图"
3. **标准操作流程 SOP** (L38-53)：明确 3 步法 (analyze → status → pick_and_place)，防止 LLM 做冗余调用或遗漏步骤
4. **安全规则** (L50-55)：IK 失败时的重试策略 (目标点±30mm)、关节限位检查、工作范围验证
5. **专业输出规范** (L57-66)：要求 LLM 以工业自动化工程师的口吻报告结果，包含视觉检测→坐标变换→IK求解→轨迹规划→执行的完整链路

**工具结果裁剪** ([react_loop.py:88-103](agent/react_loop.py#L88-L103))：

```python
def _trim_tool_result(result_str, max_len=4000):
    # 剔除 annotated_b64 和 mask_b64 (base64 图像数据)
    # 这些数据每个可达 100KB+，对 LLM 无意义但消耗大量 Token
    result_str = re.sub(r'"annotated_b64":\s*"[^"]{100,}"',
                        '"annotated_b64": "[BASE64_DATA_removed]"', result_str)
    # 截断到 max_len
    ...
```

### C.2 双模型运动学系统实现

**文件**：[services/robot_service.py](services/robot_service.py) (1046 行)

#### DH 参数模型

```python
# FANUC LR Mate 200iD 近似 DH 参数 (Z-up, 单位mm)
_default_dh() -> [
    DHParam(a=0, alpha=-π/2, d=330, offset=0),            # J1: 基座旋转
    DHParam(a=400, alpha=0, d=0, offset=-π/2),            # J2: 肩部
    DHParam(a=50, alpha=-π/2, d=0, offset=π/2),           # J3: 肘部
    DHParam(a=0, alpha=π/2, d=350, offset=0),             # J4: 腕部
    DHParam(a=0, alpha=-π/2, d=0, offset=0),              # J5: 腕部俯仰
    DHParam(a=0, alpha=0, d=80, offset=0),                # J6: 法兰
]
```

#### URDF 层级模型

```python
# 直接对应 Three.js 场景中的 robotAssembly 层级
URDF_ORIGINS = [
    [0, 0, 42.741],       # J1
    [50, 0, 287.26],      # J2
    [0, 0, 330],          # J3
    [88.001, 0, 35.027],  # J4
    [245.4, 0, 0],        # J5
    [50, 0, 0],           # J6
]

URDF_AXES = [
    [0, 0, 1],    # J1: +Z
    [0, -1, 0],   # J2: -Y
    [0, -1, 0],   # J3: -Y
    [-1, 0, 0],   # J4: -X
    [0, -1, 0],   # J5: -Y
    [-1, 0, 0],   # J6: -X
]
```

**FK 实现** ([robot_service.py:140-171](services/robot_service.py#L140-L171))：

```python
def urdf_forward_kinematics(self, joints):
    T = np.eye(4)
    for i in range(6):
        T_trans = translation_matrix(URDF_ORIGINS[i])
        R = axis_angle_matrix(URDF_AXES[i], joints[i])
        if i == 5:
            R_fixed = rotation_x(π/2)  # J6 法兰固定偏移
            T = T @ T_trans @ R @ R_fixed
        else:
            T = T @ T_trans @ R  # 先平移再旋转
    tcp = T @ [0, 0, TCP_OFFSET_Z, 1]
    return tcp  # 在 robotAssembly 坐标系中的位置
```

**Rodrigues 旋转公式**：用 `axis_angle_matrix` 将任意轴的旋转角转为 3×3 旋转矩阵：

```
R = I + sin(θ)[k]× + (1-cos(θ))[k]×²

其中 [k]× 是单位轴 k 的叉乘反对称矩阵
```

#### 数值雅可比

```python
# 3×6 位置雅可比 (用于 URDF IK)
delta = 0.005  # rad
J = np.zeros((3, 6))
for i in range(6):
    q_plus[i] += delta
    J[:, i] = (FK(q_plus) - FK(q)) / delta

# DLS 求解
JJT = J @ J.T
dq = J.T @ solve(JJT + λ²I, error) × α
```

#### 轨迹规划

```python
def plan_ptp_urdf(target_joints, steps=30):
    t = np.linspace(0, 1, steps)
    # 梯形速度曲线
    profile = piecewise(t,
        [t < 0.3, (0.3 <= t) & (t <= 0.7), t > 0.7],
        [加速段, 匀速段, 减速段])
    for i in range(steps):
        q = start + profile[i] × (end - start)
        pose = urdf_fk_pose(q)
        trajectory.append({"step": i, "joints": q, "pose": pose})
```

### C.3 视觉检测管线实现

**文件**：[services/vision_service.py](services/vision_service.py) (405 行)

**核心 API**：`POST /analyze`

**三步流水线**：

```
Step 1: YOLOv8 目标检测
  model(img, conf=0.35) → [{bbox, label, confidence, center}]

Step 2: HSV 颜色识别 (对每个检测到的物体)
  roi = hsv[y1:y2, x1:x2]
  for color_name, ranges in COLOR_RANGES:
      mask = inRange(roi, lower, upper)
      score[color_name] = countNonZero(mask) / mask.size
  dominant_color = argmax(score)

Step 3: 空间关系分析
  for each pair (obj_i, obj_j):
      dx, dy = center_j - center_i
      rel = {horizontal: "左/右", vertical: "上/下", distance_px}

→ 自然语言摘要: "物体1: 红色的方块，位置(380, 160)，大小(50×50)；
   红色方块在蓝色方块的左侧上方，距离 200 像素"
```

**HSV 颜色空间选择的原因**：RGB 对光照变化极其敏感。HSV 将色相 (H) 与亮度 (V) 分离，颜色判断只需看 H 和 S 通道。同一红色物体在不同光照下 RGB 值可能差 30%，但 HSV 的 H 值始终在 0-10°/170-180° 范围内。

**轮廓检测回退方案** ([vision_service.py:139-177](services/vision_service.py#L139-L177))：

当 YOLO 模型不可用时（如首次启动下载失败），系统自动降级到 OpenCV 轮廓检测：

```
1. 计算图像平均背景色
2. 像素与背景色的欧氏距离 > 40 → 前景
3. 形态学闭运算 (连接断裂) + 开运算 (去除噪声)
4. findContours → 过滤小面积 (< 0.3% 图像面积)
5. 返回 BBox + "object" 标签 + 估计置信度
```

这个回退方案确保了 Demo 在任何环境下都能运行，即使没有 GPU 和 YOLO 模型。

### C.4 坐标转换链实现

**文件**：[agent/tools.py](agent/tools.py) (479 行)

**像素 → 世界坐标** (tools.py:128-135)：

```python
# 相机俯视工作台区域: X=200-550, Z=-180-180
px, py = obj["center"]           # 像素坐标 (0-640, 0-480)
wx = 200 + (px / 640) × 350      # 3D场景X: 200→550 (前后)
wz = -180 + (py / 480) × 360     # 3D场景Y: -180→180 (左右)
wy = 210 + 18                     # 3D场景Z: 台面+工件高度≈228 (上下)
obj["world_xyz"] = [wx, wz, wy]   # [X前后, Y左右, Z上下]
```

**视觉世界 → 机器人 URDF** (tools.py:87-97)：

```python
def _world_to_robot(wx, wy, wz):
    rx = wx - ROBOT_BASE_X        # 前后: X→X
    ry = -(wy - ROBOT_BASE_Y)     # 左右: Y→机器人Y (取反)
    rz = wz - PEDESTAL_H          # 上下: Z→机器人Z
    return (rx, ry, rz)
```

**机器人 URDF → 3D 场景** (robot_service.py:173-197)：

```python
def urdf_fk_world(joints):
    tcp_u = urdf_fk(joints)                     # URDF Z-up
    three_x = tcp_u[0]                           # X→X
    three_y = tcp_u[2] + ROBOT_ASSEMBLY_Y       # Z→Y
    three_z = -tcp_u[1]                          # Y→Z (取反)
    three_x += ROBOT_GROUP_X                     # 偏移
    return [three_x, three_z, three_y]           # 视觉约定
```

**完整转换链验证**：
```
像素(380, 160) → 世界(408, 20, 228) → URDF(498, -20, -112)
→ IK求解关节角 → FK回到URDF → 世界(408, 20, 228) ✓
```

### C.5 3D 数字孪生前端实现

**文件**：[web/js/app.js](web/js/app.js) (~2000 行)

#### 场景构建

**工业地面**：程序化生成的 checker plate 纹理 (Canvas2D → CanvasTexture)，深蓝灰色金属格子地板，模拟工厂环境。

**三点照明系统**：

| 灯光 | 位置 | 强度 | 作用 |
|------|------|------|------|
| AmbientLight | 全局 | 2.0 (0x334466) | 基础环境光，防止暗部全黑 |
| KeyLight (Directional) | (600, 900, 500) | 5.0 | 主光源，投射阴影 |
| FillLight (Directional) | (-400, 300, -300) | 1.5 | 补光，柔化暗面 |
| RimLight (Directional) | (0, 200, -500) | 2.0 | 轮廓光，突出机器人立体感 |

**阴影**：PCFSoftShadowMap, 2048×2048 shadow map, 主光源级联阴影覆盖 2400×2400mm 范围。

**色调映射**：ACESFilmicToneMapping, exposure 1.2，模拟电影级 HDR→SDR 转换。

#### 机器人模型

**STL 网格加载**（FANUC LR Mate 200iD 官方 CAD 模型）：
- 7 个独立 STL 文件：base_link, J1-J6
- 三套精度：convex (碰撞检测用) / low-res (低带宽) / med-res (展示)
- 材质：MeshStandardMaterial { roughness: 0.3, metalness: 0.9 }，工业金属质感

**程序化回退**：当 STL 文件无法加载时，使用 `createProceduralLink()` 生成简化 Box/Cylinder 几何体，确保场景永不为空。

**关节层级**：

```
robotGroup (position: x=-90)
└── robotAssembly (rotation: Rx(-π/2))
    ├── base_link (固定)
    ├── J1 (绕 Z 轴旋转) → 带动 J2-J6
    ├── J2 (绕 -Y 轴旋转) → 带动 J3-J6
    ├── J3 (绕 -Y 轴旋转) → 带动 J4-J6
    ├── J4 (绕 -X 轴旋转) → 带动 J5-J6
    ├── J5 (绕 -Y 轴旋转) → 带动 J6
    ├── J6 (绕 -X 轴旋转)
    └── gripper (J6 子节点)
        ├── finger_left (沿 X 平移)
        └── finger_right (沿 -X 平移)
```

#### 动画系统

前端接收来自 Agent 的轨迹序列 (joints 数组)，在 `requestAnimationFrame` 中进行关键帧插值：

```
trajectory = [{step:0, joints:[...], pose:{...}}, ...]

每帧 (16ms@60fps):
  current_step = lerp(trajectory[step_i], trajectory[step_i+1], frame_frac)
  applyJointsToRobot(current_step.joints)
  updateJointPanel(current_step.joints)
  updateTCPPanel(current_step.pose)
```

**抓取工件动画**：在执行 pick 阶段时，目标工件会附着到夹爪位置并跟随移动，在 place 阶段时在目标位置脱离。这在 `app.js` 中通过动态修改工件的 parent (从 `scene` 切换到 `gripper_group`) 实现。

#### 场景健康监控

```javascript
setInterval(() => {
    updateSceneHealth()  // 每 2 秒检查一次
}, 2000)

function updateSceneHealth() {
    // 检查: camera position, light count, mesh count, renderer state
    // 状态灯: 🟢正常 / 🟠警告 (网格过少) / 🔴严重 (渲染丢失)
}
```

### C.6 点动控制与示教编程实现

#### 关节点动

```python
# POST /jog {type:"joint", index:0-5, delta_deg:±1.0}
jogJoint(idx, dir):
    target = current_joints.copy()
    target[idx] += dir × step_deg  # 步长: 0.5°/1°/5°/10°
    clamp(target, ±limits[idx])
    trajectory = plan_ptp_urdf(target)
    return {joints, joints_deg, tcp, trajectory}
```

#### 笛卡尔点动

```python
# POST /jog {type:"cartesian", axis:"x"|"y"|"z", delta_mm:±10.0}
jogCartesian(axis, dir):
    tcp = urdf_fk_world(current_joints)
    tcp[axis] += dir × step_mm  # 步长: 5/10/25/50mm
    new_joints = urdf_ik_robust(tcp[x], tcp[y], tcp[z])
    trajectory = plan_ptp_urdf(new_joints)
```

#### 示教与回放

前端维护 `teachPoints[]` 数组，每次 "记录" 时保存当前关节角 + TCP 位姿。回放时按顺序发送 `/jog` 命令到每个示教点。

#### FANUC TP 程序生成

前端 `generateProgram()` 根据任务执行历史 (`programActions[]`) 自动生成符合 FANUC TP 编程规范的 LS 格式程序：

```fanuc
/PROG STYLE_MN
/ATTR
  OWNER = MNEDITOR
  COMMENT = "Auto-generated pick & place"
  PROG_SIZE = 2048
  CREATE = DATE 26-06-12 TIME 14:30:00
  MODIFIED = DATE 26-06-12 TIME 14:30:00
  FILE_NAME = STYLE_MAIN
  VERSION = 0
  ...

/POS
  P[1]{
    GP1:
      UF : 1, UT : 1,
    J1 = 0.000 deg, J2 = 80.000 deg, J3 = -90.000 deg,
    J4 = 0.000 deg, J5 = 0.000 deg, J6 = 0.000 deg
  };
  ...

LBL[1] ;
  J P[1] 30% CNT100 ;
  CALL S01PICK ;
  ...
```

### C.7 多模式 UI 架构实现

**模式切换** ([index.html:58-163](web/index.html#L58-L163))：

```
Agent 模式  ←→  点动模式  ←→  程序模式
(默认)         (手动操控)     (TP编程)
```

**切换机制**：
- 三个 mode-panel div，同一时刻只有一个 `display: block`
- `switchMode(mode)` 更新 tab 按钮 active 状态、隐藏/显示对应面板
- 底部面板 (视觉/关节/TCP) 在所有模式下保持可见

**Agent 模式**：
- 快速指令按钮 (分析场景 / 抓取红色 / 移动蓝色)
- 对话日志 (chat-log)，支持 Agent 事件彩色渲染
- 文本输入框，支持 Enter 发送、Shift+Enter 换行
- 停止按钮 (AbortController + fetch abort)

**点动模式**：
- 子模式切换：关节 / 笛卡尔 / 示教
- 6 轴关节 ± 按钮，实时更新角度显示
- 笛卡尔 X/Y/Z ± 按钮
- 步长选择 (0.5° / 1° / 5° / 10°) 和速度滑块 (10%-100%)
- 示教点列表 + 记录/回放/清空按钮

**程序模式**：
- 生成 / 导出 / 发送按钮
- 代码查看器 (<pre> 标签，等宽字体)

---

## Part D: 测试文档

### D.1 测试架构总览

```
测试层级:
┌─────────────────────────────────────────────────────┐
│ Level 3: 端到端测试 (手动)                            │
│   完整场景: 打开 UI → 输入指令 → 观察 3D 动画 →       │
│   验证视觉画面 → 检查关节角/TCP → 查看程序生成         │
├─────────────────────────────────────────────────────┤
│ Level 2: 集成测试 (Python + HTTP)                    │
│   run_integration_tests()                           │
│   5 个测试步骤: 健康检查 → FK → 轨迹 → 视觉 → 抓取    │
├─────────────────────────────────────────────────────┤
│ Level 1: 单元测试 (unittest, 不依赖外部服务)           │
│   4 个 TestCase 类, 19 个测试方法                     │
│   覆盖: 视觉算法、运动学、工具注册、Agent 循环         │
└─────────────────────────────────────────────────────┘
```

### D.2 单元测试详细说明

**运行方式**：

```bash
# 运行全部单元测试 (19个)
python -m pytest tests/test_pipeline.py -v

# 仅运行单元测试 (不依赖服务)
python tests/test_pipeline.py --unit

# 指定测试类
python -m pytest tests/test_pipeline.py::TestRobotService -v
```

#### TestVisionService (5 个测试)

| # | 测试方法 | 覆盖内容 | 断言条件 |
|---|---------|---------|---------|
| 1 | `test_create_test_image` | 合成图像生成 | shape=(480,640,3), 4个物体 |
| 2 | `test_color_ranges` | HSV 颜色范围定义 | 包含红/蓝/绿, 总数>6 |
| 3 | `test_image_preprocessing` | CLAHE 自适应直方图均衡化 | 输出 shape 等于输入 shape |
| 4 | `test_color_detection_on_synthetic` | 合成图像 HSV 颜色识别 | 红色区域占比 > 0.5% |
| 5 | `test_fallback_detection` | OpenCV 轮廓回退检测 | status=200, success=true |

**设计意图**：
- 测试 3-5 验证核心视觉算法不经 HTTP 层的直接正确性
- 合成图像确保了确定性输入，不受外部因素影响
- CLAHE 预处理验证了图像增强管线不会引入畸变
- 回退检测验证了 YOLO 不可用时的降级策略

#### TestRobotService (7 个测试)

| # | 测试方法 | 覆盖内容 | 断言条件 |
|---|---------|---------|---------|
| 1 | `test_dh_transform` | DH 参数→4×4 齐次矩阵 | shape=(4,4), T[0,3]≈300 |
| 2 | `test_forward_kinematics_zero` | 零位正运动学 | shape=(4,4), Z>0 |
| 3 | `test_forward_kinematics_reproducibility` | FK 确定性 | 相同输入→相同输出 |
| 4 | `test_inverse_kinematics_basic` | FK→IK→FK 闭环 | X/Y/Z 误差 < 8mm |
| 5 | `test_fk_consistency` | 零位/非零位 FK 对比 | 零位 Z>0, 不同关节产生不同位姿 |
| 6 | `test_ptp_plan` | PTP 轨迹规划 | 30步轨迹, 终点关节角匹配 |
| 7 | `test_jacobian_numeric` | 数值雅可比 | shape=(6,6), 无 NaN |

**设计意图**：
- 测试 1-2 验证 DH 参数基础正确性（单次变换 + 完整 FK）
- 测试 3 确保 FK 是纯函数（对 Agent 可靠性至关重要）
- 测试 4 是运动学系统最重要的闭环验证：前向→逆向→前向，保证 IK 收敛到合理位置
- 测试 6 验证轨迹规划输出的完整性和终点精度
- 测试 7 确保雅可比矩阵无数值问题（NaN/Inf）

**已知限制**：test_inverse_kinematics_basic 的 delta=8mm 容差相对宽松，这是因为数值 IK 的局部收敛特性。在实际使用中，URDF IK 的收敛容差为 0.5mm，精度远高于此测试。

#### TestAgentTools (4 个测试)

| # | 测试方法 | 覆盖内容 | 断言条件 |
|---|---------|---------|---------|
| 1 | `test_tools_registered` | 工具注册完整性 | tools ≥ 7, schemas = tools |
| 2 | `test_tool_names_unique` | 工具名唯一性 | len(names) = len(set(names)) |
| 3 | `test_vision_tools_exist` | 视觉工具存在性 | analyze_scene/detect/color 非空 |
| 4 | `test_robot_tools_exist` | 机器人工具存在性 | status/move/pick/place/reset 非空 |

#### TestAgentLoop (2 个测试)

| # | 测试方法 | 覆盖内容 | 断言条件 |
|---|---------|---------|---------|
| 1 | `test_agent_creation` | Agent 实例化 | 非空, 提示词包含"六轴"+"视觉" |
| 2 | `test_system_prompt_contains_tools` | 提示词工具描述 | 包含 analyze_scene/pick/move |

### D.3 集成测试详细说明

**前置条件**：启动所有 3 个后端服务。

```bash
# 终端 1
python services/vision_service.py --port 8801

# 终端 2
python services/robot_service.py --port 8802

# 终端 3
python services/agent_service.py --port 8803

# 终端 4: 运行集成测试
python tests/test_pipeline.py --integration
```

**测试步骤与预期结果**：

```
Step 1/5: 健康检查
  ├── Vision:  GET /health → {"status":"ok", "yolo_available":true, "device":"cpu"}
  ├── Robot:   GET /health → {"status":"ok", "axes":6}
  └── Agent:   GET /health → {"status":"ok", "model":"deepseek-chat", "tools_loaded":8}
  预期: 三项全部返回 200 OK

Step 2/5: 机器人运动学
  └── POST /fk {"joints":[0, -0.5, 0.3, 0, 0, 0]} → {"pose":{x,y,z,rx,ry,rz}}
  预期: 返回有效 6-DOF 位姿，Z>0 (末端高于基座)

Step 3/5: 轨迹规划
  ├── POST /reset → {"success":true}
  └── POST /plan {"type":"PTP","joints":[0.5,-0.3,0.2,0.1,-0.1,0.0],"steps":20}
 预期: trajectory 长度=20, final_pose 存在且关节角匹配

Step 4/5: 视觉分析
  └── POST /analyze {"image_b64": <合成4色工件图像>}
  预期: objects 包含 4 个检测项，每项有 bbox/label/color/center

Step 5/5: 抓取动作序列
  └── POST /execute {"action":"pick","target":{"x":350,"y":-50,"z":300}}
  预期: phases≥3 (approach+descend+gripper+retreat), total_steps≥60
```

### D.4 手动测试脚本

以下是完整的手动端到端测试流程，适合 Demo 展示或功能验证。

#### 测试 1：基础场景渲染

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 1.1 | 打开 `web/index.html` (开发模式) 或生产 URL | 页面加载，品牌名可见 |
| 1.2 | 观察 3D 视图区 | 看到：机器人模型 + 工作台 + 工件 + 地面 + 围栏 |
| 1.3 | 鼠标旋转/缩放/平移场景 | OrbitControls 正常响应 |
| 1.4 | 检查顶部状态灯 | 4 个状态点可见 (视觉/机器人/Agent/场景) |
| 1.5 | 检查底部面板 | 视觉画面 / 关节表 / TCP 表 正常显示 |
| 1.6 | 等待 2 秒 | 场景健康状态显示 "正常" (绿色) |

#### 测试 2：Agent 自然语言操

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 2.1 | 确保 3 个后端服务运行 | `/health` 全部返回 200 |
| 2.2 | 点击 "分析场景" 按钮 | Agent 对话区显示 thinking → tool_call → tool_result |
| 2.3 | 检查视觉面板 | 显示标注后的场景图像 (BBox + 颜色标签) |
| 2.4 | 检查对话日志 | Agent 列出检测到的物体: "物体1: 红色方块..." |
| 2.5 | 点击 "抓取红色" 按钮 | 3D 机器人开始动画: 移动到工件上方→下降→夹爪闭合→抬升 |
| 2.6 | 观察关节面板 | J1-J6 角度实时更新 |
| 2.7 | 观察 TCP 面板 | X/Y/Z 坐标实时更新 |
| 2.8 | 观察夹爪状态 | "夹爪: 关闭" |
| 2.9 | 等待动画完成 | Agent 回复 "[视觉检测]...[逆运动学]...[执行完成]" |

#### 测试 3：点动控制

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 3.1 | 切换到 "点动" 标签 | 显示关节点动界面 |
| 3.2 | 点击 J1+ 按钮 | 机器人 J1 旋转 +1°, 角度显示更新 |
| 3.3 | 连续点击 J1+ 5 次 | 机器人平滑转动 5° |
| 3.4 | 切换到 "笛卡尔" 子模式 | 显示 X/Y/Z 点动按钮 |
| 3.5 | 点击 X+ 按钮 | 机器人末端沿 X 正向移动 10mm |
| 3.6 | 调整步长为 5° | J1+ 每次转动 5° |
| 3.7 | 调整速度为 50% | 运动速度明显加快 |
| 3.8 | 切换到 "示教" 子模式 | 显示空示教列表 |
| 3.9 | 调整机器人到不同位置，点击 "记录" 3 次 | 列表显示 3 个示教点 |
| 3.10 | 点击 "回放" | 机器人依次移动到 3 个示教点 |

#### 测试 4：程序生成

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 4.1 | 执行一次抓取任务 (测试 2.5) | 任务完成 |
| 4.2 | 切换到 "程序" 标签 | 显示 TP 程序代码 |
| 4.3 | 检查程序内容 | 包含 /PROG STYLE_MN, P[1]-P[n] 位置定义, LBL 标签 |
| 4.4 | 点击 "导出" | 浏览器下载 .ls 文件 |
| 4.5 | 点击 "发送" | 程序存储到 robot_service 内存 |

#### 测试 5：错误处理与健壮性

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 5.1 | 停止 Vision 服务 → 点击 "分析场景" | Agent 返回 "视觉服务未启动" 错误 |
| 5.2 | 停止 Robot 服务 → 点击 "抓取红色" | Agent 返回 "机器人服务未启动" 错误 |
| 5.3 | 输入不合理指令 "抓取太阳" | Agent 提示目标超出工作范围 |
| 5.4 | 快速连续发送 3 条指令 | Agent 顺序处理，无状态错乱 |
| 5.5 | 执行中点击 "停止" 按钮 | SSE 流中断，按钮恢复为 "发送" |
| 5.6 | 断网后重连 | 页面保持响应，3D 场景不受影响 |
| 5.7 | 检查浏览器控制台 | 无 JavaScript 错误 (红字) |

#### 测试 6：生产部署验证

| 步骤 | 操作 | 预期结果 |
|------|------|----------|
| 6.1 | 访问生产域名 (如 http://robot.yourdomain.com) | 页面正常加载 |
| 6.2 | 检查 Network 面板 | API 请求走 /api/* 路径，200 返回 |
| 6.3 | 检查 SSE 连接 | /api/agent/chat/stream 使用 text/event-stream |
| 6.4 | 检查 STL 加载 | 3D 机器人模型完整显示 |
| 6.5 | 检查静态资源缓存 | CSS/JS/STL 返回适当 Cache-Control 头 |

### D.5 性能测试与基准

#### 关键性能指标

| 指标 | 目标值 | 测量方法 |
|------|--------|----------|
| YOLO 推理时间 | < 500ms (CPU) / < 50ms (GPU) | analyze_scene 往返时间 |
| IK 求解时间 | < 2s (最坏情况 600 迭代) | urdf_ik_robust 计时 |
| 轨迹规划时间 | < 100ms (30 步 PTP) | plan_ptp_urdf 计时 |
| 3D 渲染帧率 | ≥ 30 FPS | requestAnimationFrame 统计 |
| SSE 流延迟 | < 200ms (首字节) | EventSource onmessage 时间戳 |
| LLM 响应时间 | 1-5s (取决于 DeepSeek API) | chat/completions 往返时间 |
| 页面首屏加载 | < 3s | PerformanceObserver |

#### 压力测试建议

```bash
# 并发视觉分析请求
for i in {1..10}; do
  curl -X POST http://127.0.0.1:8801/analyze \
    -H "Content-Type: application/json" \
    -d '{"image_b64":"...test data..."}' &
done
wait

# 连续轨迹规划
for i in {1..50}; do
  curl -X POST http://127.0.0.1:8802/plan \
    -H "Content-Type: application/json" \
    -d '{"type":"PTP","joints":[0.5,-0.3,0.2,0.1,-0.1,0.0],"steps":50}'
done
```

---

## 附录：改进路线图

基于当前实现的分析，以下是可量化的改进方向：

### 短期 (1-2 周)
- [ ] 集成 MuJoCo 物理引擎进行碰撞检测和力反馈
- [ ] 夹爪力控 (当前为位置控制，加力量模式)
- [ ] 视觉标定自动化 (当前为硬编码像素→世界映射)

### 中期 (1-2 月)
- [ ] ROS2 集成 (发布关节状态和 TF 变换)
- [ ] 真实相机接入 (替换离屏渲染)
- [ ] 多机器人协作场景 (双机协同搬运)

### 长期 (3-6 月)
- [ ] 迁移学习：自定义 YOLO 模型 (工业零件数据集)
- [ ] 离线强化学习：从演示中学习最优抓取策略
- [ ] 数字孪生→真实机器人 sim2real 迁移
