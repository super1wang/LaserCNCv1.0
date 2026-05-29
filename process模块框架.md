# Process 模块框架

审阅日期：2026-05-28  
适用范围：`src/modules/process` 后续重构与实现。

## 1. 定位

Process 模块是 LaserCNC 的加工运行域模块，不负责几何建模、不持有 OCC shape、不直接操作 3D 显示。它在微内核中的职责是：

- 管理机台硬件与外设：运动控制器、激光器、IO、气、水、监控、相机触发、通讯链路。
- 管理加工参数：设备 profile、轴参数、工具参数、图层/工具绑定、工艺参数、外设参数。
- 消费 CAM 模块输出的刀路点集：只接收不含 OCC 的点集快照、轮廓 id、图层 id 和工具绑定信息。
- 对刀路进行排序、启用过滤、图层/工具匹配和运行前校验。
- 根据控制器类型生成或下发控制器指令：PureSimulation、SimulatorCMHP、ACS、GTN 等 profile 分别有 translator/adapter。
- 管理工作流编辑、保存、加载、解释执行和运行控制。
- 维护明确的运行状态机：待机、准备、加工中、暂停、停止中、异常、急停等。

## 2. 当前审阅结论

当前代码已经形成了可运行的基础闭环：`ProcessModule`、`IProcessFacade`、`ProcessFlowDocument`、`ProcessNodeRegistry`、`ProcessWorkflowExecutor`、`ProcessDeviceManager`、`ProcessSettings`、通讯模块和旧 Setting `.ui` 直接加载均已存在。

2026-05-28 本轮落地后，最终框架的关键代码边界已经建立：CAM 导出 `ToolpathExportSnapshot`，Process 运行态只消费纯数值 DTO；`ProcessRuntime` 和 `ProcessStateMachine` 成为状态/参数上下文入口；设备、刀路、工具匹配、指令、工作流和执行服务已拆成独立文件并接入 `ProcessModule`。

主要结构问题是：

- `ProcessModule` 仍承担过多职责：模块生命周期、facade、状态、设备切换、CAM 快照适配、执行器信号桥接、轴位仿真都集中在一个类中。
- `ProcessWorkflowExecutor` 当前是 QTimer 顺序 dry-run，尚未形成“执行计划 -> 指令计划 -> 控制器 translator -> 设备会话”的真实链路。
- CAM facade 当前只暴露刀路存在性、轮廓数和点数，不足以支撑排序、工具匹配和控制器指令生成。
- `ProcessSettings` 已能落地 UI 参数，但仍是基础字段加 `uiSetting` 通用表，关键字段还没有 typed schema、版本迁移和校验。
- `ProcessDeviceManager` 目前更像 descriptor/factory，尚不是完整的设备 session/registry：连接状态、错误、外设生命周期和通讯复用仍需集中管理。
- Process 源码未直接依赖 OCC，但 CAM 当前核心刀路类型仍含 OCC；后续跨模块边界必须引入纯数据 DTO，避免 Process 引入 `LaserToolpath` 或 OCC 类型。

## 3. 微内核边界

Process 作为一个微内核模块，只向外注册少量稳定 service：

```text
Kernel
  └─ ServiceRegistry
       ├─ IProcessFacade          # UI/app 调用入口
       ├─ IMotionController       # 当前活动运动控制器，可供其他模块查询
       └─ 可选只读诊断服务         # 后续需要时再暴露

ProcessModule
  ├─ 只负责 IModule 生命周期、service 注册、依赖装配、facade 转发
  └─ 不持有 QWidget/QTreeView/OCC shape/旧全局 Service
```

跨模块依赖规则：

- Process 可以依赖 `core/kernel`、`core/kinematics` 抽象接口、`core/settings`、`core/logging`。
- Process 可以通过 `ICamFacade` 或后续 `ICamToolpathProvider` 获取 CAM 输出的纯数据快照。
- Process 不 include `TopoDS_*`、`AIS_*`、`gp_*`、`Geom_*`、`BRep*`、`XCAF*` 等 OCC 类型。
- Process 不 include CAM UI、CAD UI、app UI 或 view 层。
- UI/dialog/ribbon 通过 `IProcessFacade`、model/view adapter 和设置页操作，不作为运行数据事实源。

## 4. 是否需要旧工程式 Service 类

不建议恢复旧工程那种单一全局 `Service` 类。

原因：旧式 `Service` 通常同时承担参数、硬件、流程、状态、全局指针和跨模块调用，容易把旧 `DT`、静态 factory、SDK 头、UI 指针重新扩散到新工程，破坏微内核边界。

建议采用“薄协调器 + 多个内部服务”的结构：

- 可以有一个 `ProcessRuntime` 或 `ProcessRuntimeCoordinator`，负责把状态机、设备、参数、刀路、工作流和指令服务串起来。
- `ProcessRuntime` 不作为全局单例，不直接暴露给所有模块，不保存 UI 指针，不直接 include vendor SDK。
- 真正能力拆到独立服务：`ProcessStateMachine`、`ProcessDeviceService`、`ProcessSettingsService`、`ProcessToolpathService`、`ProcessInstructionService`、`ProcessWorkflowService`、`ProcessExecutionService`。
- 对外仍通过 `IProcessFacade` 暴露稳定动作，例如 run/pause/stop/load/save/openSettings/status。

结论：需要统一管理入口，但它应是模块内部的运行时协调器，不是旧工程全局 Service。

## 5. 目标运行时结构

```text
src/modules/process/
├── process_module.*
│   └─ 模块装配层：IModule + IProcessFacade，创建并持有 ProcessRuntime
├── runtime/
│   ├── process_runtime.*
│   ├── process_state_machine.*
│   ├── process_context.*
│   └── process_diagnostics.*
├── settings/
│   ├── process_settings.*
│   ├── process_settings_schema.*
│   ├── process_settings_store.*
│   ├── process_tool_settings.*
│   ├── process_device_settings.*
│   └── process_layer_tool_settings.*
├── device/
│   ├── process_device_service.*
│   ├── process_device_registry.*
│   ├── process_device_session.*
│   ├── i_laser_device.h
│   ├── i_process_io.h
│   ├── i_process_aux_device.h
│   ├── motion/*
│   ├── laser/*
│   └── io/*
├── toolpath/
│   ├── process_toolpath_types.*
│   ├── process_toolpath_provider.*
│   ├── process_toolpath_service.*
│   ├── process_toolpath_sorter.*
│   └── process_tool_matcher.*
├── instructions/
│   ├── process_command.h
│   ├── process_command_buffer.*
│   ├── process_instruction_planner.*
│   ├── i_controller_translator.h
│   ├── pure_simulation_translator.*
│   ├── acs_translator.*
│   └── gtn_translator.*
├── workflow/
│   ├── process_flow_document.*
│   ├── process_flow_store.*
│   ├── process_node_registry.*
│   ├── process_node_validation.*
│   └── process_workflow_service.*
├── execution/
│   ├── process_execution_service.*
│   ├── process_execution_plan.*
│   ├── process_node_executor_registry.*
│   └── node_executors/*
├── communication/
│   └── Mock/TCP/HTTP/Serial 通讯基础设施
├── ui/
│   ├── process_flow_model.*
│   ├── process_flow_tree_view.*
│   ├── process_node_edit_dialog.*
│   └── settings_pages/*
├── Setting/
│   └── 旧 `.ui` 资源与直接加载桥接，不编译旧业务 cpp
└── commands/
    └── commands_process.*
```

## 6. 子模块职责

### 6.1 Runtime

`ProcessRuntime` 是内部协调器，持有各服务实例并处理 facade 调用：

- `startWorkflow()`：状态机校验 -> 读取 CAM 点集 -> 工具匹配 -> 生成执行计划 -> 进入加工。
- `pauseWorkflow()`：状态机切暂停 -> 通知执行服务和设备 session。
- `stopWorkflow()`：请求停止 -> 关闭激光/IO -> 控制器停止 -> 回到待机或异常。
- `emergencyStop()`：立即进入急停 -> 硬件急停 -> 执行服务中断。
- `applySettings()`：保存参数 -> 刷新设备 registry/session -> 通知 UI。

Runtime 只做编排，不实现具体设备、刀路、参数和指令细节。

### 6.2 State Machine

`ProcessStateMachine` 是 Process 运行状态的唯一事实源。

建议状态：

| 状态 | 含义 |
| --- | --- |
| `Uninitialized` | 模块尚未完成 init。 |
| `Idle` | 待机，可编辑参数、流程和设备。 |
| `Connecting` | 正在连接设备或切换 profile。 |
| `Preparing` | 运行前准备：校验参数、拉取 CAM 点集、生成计划。 |
| `Ready` | 准备完成，可开始下发执行。 |
| `Processing` | 加工中。 |
| `Paused` | 暂停中，可继续或停止。 |
| `Stopping` | 停止中，等待设备进入安全态。 |
| `Stopped` | 已停止，可转待机。 |
| `Error` | 异常，需要复位或重新准备。 |
| `EmergencyStop` | 急停锁定，只允许复位急停和安全查询。 |

核心转换：

```text
Uninitialized -> Idle
Idle -> Connecting -> Idle/Error
Idle -> Preparing -> Ready -> Processing
Processing -> Paused -> Processing
Processing -> Stopping -> Stopped -> Idle
Paused -> Stopping -> Stopped -> Idle
Preparing/Ready/Processing/Paused -> Error
任意非 Uninitialized 状态 -> EmergencyStop
EmergencyStop -> Idle        # 仅硬件确认复位后
Error -> Idle                # 清错或重新加载配置后
```

状态机负责拒绝非法操作，例如加工中禁止切换控制器、急停中禁止运行、准备中禁止保存工作流。

### 6.3 Settings

设置层分两级：

- `ProcessSettings` / `ProcessSettingsStore`：TOML 持久化、版本迁移、默认值。
- typed DTO：设备、轴、工具、图层工具绑定、工艺、IO、通讯、监控等可被运行时直接使用的数据结构。

旧 Setting `.ui` 的角色：

- `.ui` 是快速落地界面和字段清单。
- `uiSetting.<page>.<field>` 是兼容存储层。
- 关键字段必须逐步提升为 typed DTO，不能长期让执行逻辑直接查字符串 map。

### 6.4 Device

设备层建议拆成 registry、session、adapter 三层：

- `ProcessDeviceRegistry`：记录所有可用设备、profile、能力、SDK 可用性和缺失原因。
- `ProcessDeviceSession`：管理当前活动设备实例、连接状态、错误状态、安全关闭和 profile 切换。
- Adapter：`IMotionController`、`ILaserDevice`、`IProcessIo`、`IProcessAuxDevice` 的具体实现。

设备 profile：

- `PureSimulation`：无 SDK 依赖，只运行内部仿真 translator。
- `SimulatorCMHP`：ACS SDK simulator profile，可真实调用 ACS 接口，但标记为模拟设备。
- `ACS`：ACS 真机 profile。
- `GTN`：固高真机 profile。

真实 SDK 头文件只允许出现在 adapter private cpp 或 option-gated private header 中，不得进入公共接口。

### 6.5 Toolpath

Process 需要新的 OCC-free 刀路输入 DTO。CAM 负责从 OCC/CAM 内部数据导出，Process 只消费值类型：

```cpp
struct ProcessToolpathPoint {
    double x, y, z;
    double nx, ny, nz;
    double tx, ty, tz;
    double r1, r2;
    QString r1Name, r2Name;
    bool machineCoordValid;
};

struct ProcessToolpathContour {
    quint64 contourId;
    quint64 layerId;
    QString layerName;
    QString toolName;
    bool enabled;
    QVector<ProcessToolpathPoint> points;
};

struct ProcessToolpathSnapshot {
    quint64 revision;
    QVector<ProcessToolpathContour> contours;
};
```

`ProcessToolpathService` 的职责：

- 从 CAM 拉取 snapshot 并复制为运行时不可变数据。
- 按启用状态、图层、工具、工作流选择规则过滤轮廓。
- 按图层、工具、距离、用户顺序或策略排序。
- 校验点集、机床坐标、工具绑定、图层参数和安全条件。
- 输出 `ProcessJobPlan`，供指令规划使用。

### 6.6 Instruction

指令层分两步：

1. `ProcessInstructionPlanner` 把工作流节点和 `ProcessJobPlan` 转成控制器无关的命令缓冲。
2. `IControllerTranslator` 根据当前 controller profile 转成具体 SDK 调用、脚本或仿真动作。

中间命令示例：

```text
SetFeed(rate)
SetLaserPower(energy, frequency, pulseWidth)
MoveLinear(x, y, z, r1, r2)
LaserOn
LaserOff
SetDigitalOutput(channel, value)
Dwell(ms)
WaitSignal(channel, expected, timeoutMs)
```

这样 `Cutting` 节点不直接知道 ACS/GTN 指令，ACS/GTN 差异收敛在 translator。

### 6.7 Workflow

工作流分编辑态和运行态：

- 编辑态：`ProcessFlowDocument`、`ProcessNodeRegistry`、model/view、节点编辑器。
- 运行态：`ProcessWorkflowService` 把流程树编译为 `ProcessExecutionPlan`。
- 节点 executor 不直接操作 UI，只调用 runtime/device/toolpath/instruction 服务。
- `Cutting` 节点应引用 CAM contour/layer/tool selection，不持有 OCC shape。

## 7. 运行流程

典型加工启动流程：

```text
UI/Ribbon -> IProcessFacade::runStart()
ProcessModule -> ProcessRuntime::startWorkflow()
StateMachine: Idle -> Preparing
SettingsService: 读取 typed 参数快照
DeviceSession: 校验控制器、激光器、IO ready
ToolpathService: 从 CAM 获取 OCC-free 点集快照
ToolMatcher: 图层/轮廓匹配工具和工艺参数
ToolpathSorter: 生成有序轮廓列表
WorkflowService: 编译流程节点
InstructionPlanner: 生成 controller-neutral command buffer
Translator: 按 PureSimulation/ACS/GTN 生成执行动作
ExecutionService: 执行、暂停、停止、急停、状态回写
StateMachine: Ready -> Processing -> Idle/Error/EmergencyStop
```

## 8. 对外接口建议

`IProcessFacade` 后续应保持小而稳定，建议按用户动作和只读查询扩展：

- 运行控制：`prepare()`、`runStart()`、`runPause()`、`runResume()`、`runStop()`、`emergencyStop()`、`resetEmergencyStop()`。
- 文件：`newProcess()`、`loadProcess()`、`saveProcess()`。
- 设备：`connectDevices()`、`disconnectDevices()`、`setActiveProfile()`、`deviceStatus()`。
- 设置：`reloadSettings()`、`settingsRevision()`。
- 状态：`state()`、`statusMessage()`、`lastError()`、`progress()`。

跨模块 CAM 接口建议新增或扩展为只读数据 provider：

- `bool hasProcessToolpath() const`
- `ProcessToolpathSnapshot processToolpathSnapshot() const`
- `quint64 processToolpathRevision() const`

该 DTO 必须放在不依赖 OCC 的 contract 头中。

## 9. 设计约束

- 不恢复旧全局 `Service`。
- 不让 `ProcessModule` 继续膨胀为业务实现类。
- 不把旧 Setting/Process/device `.cpp` 整包加入 CMake。
- 不让 Process 运行数据依赖 OCC；运行时只使用 CAM 导出的点集和参数 DTO。
- 不让设备 adapter 的 vendor SDK 头扩散到公共接口。
- 不让执行器直接操作 UI item 或 ProjectExplorer item。
- 不让执行逻辑直接依赖 `uiSetting` 字符串 map；关键参数必须 typed 化。
- 加工中、暂停、急停状态禁止切换控制器和关键设备 profile。

## 10. 验收标准

- SDK-off Debug 构建通过。
- Process 源码搜索不到 OCC 公共类型 include。
- `ProcessModule` 只做装配和 facade 转发，业务迁入 runtime/services。
- 状态机覆盖待机、准备、加工中、暂停、停止中、异常、急停，并拒绝非法转换。
- CAM 向 Process 提供纯数据点集快照，Process 可完成排序、工具匹配和校验。
- PureSimulation 可执行完整 dry-run；SimulatorCMHP/ACS/GTN 通过 translator/adapter 生成对应执行路径。
- 参数 UI 能保存/加载；关键参数逐步进入 typed schema。
- 工作流节点状态、运行进度和错误可回写 UI。
