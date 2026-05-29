# Process 模块重构计划

审阅日期：2026-05-28  
依据：[process模块框架.md](process模块框架.md)

## 1. 重构目标

本轮重构目标是把 Process 模块从“可运行的基础闭环”收敛为清晰的加工运行域：

- 符合微内核结构：`ProcessModule` 只负责模块生命周期、service 注册和 facade 转发。
- 运行数据不依赖 OCC：Process 只消费 CAM 输出的刀路点集 DTO，不 include OCC/CAM 几何类型。
- 具备显式状态机制：待机、准备、加工中、暂停、停止中、异常、急停等状态由统一状态机管理。
- 内部子模块清晰：外设、参数、刀路、指令、工作流、执行、通讯各自独立。
- 可根据控制器类型生成执行路径：PureSimulation、SimulatorCMHP、ACS、GTN 分别由 translator/adapter 处理。
- 支持 CAM 刀路排序、图层/工具匹配、运行前校验和工作流运行控制。

## 2. 当前审阅摘要

已完成基础能力：

- `ProcessFlowDocument` 已是流程事实源，流程树 UI 已通过 model/view 展示。
- `ProcessNodeRegistry` 已注册当前主要节点类型和默认参数。
- `ProcessWorkflowExecutor` 已有顺序 dry-run 和状态回写。
- `ProcessDeviceManager` 已有设备 descriptor 和 PureSimulation/SimulatorCMHP/ACS/GTN profile 基础。
- `ProcessSettingsDialog` 已直接加载 25 个旧 Setting `.ui`，并通过 `uiSetting` 落地字段值。
- 通讯模块已支持 Mock/TCP/HTTP/Serial。

本轮已按最终框架补齐 Phase 0-11 的代码骨架和接入点：CAM 通过 `ICamToolpathProvider` 导出 OCC-free 点集，Process 侧建立 `ProcessRuntime`、状态机、typed settings snapshot、设备协调器、刀路服务、排序/工具匹配、指令 planner/translator、workflow service、execution service 和 node executor registry。默认 SDK-off Debug 构建通过。

主要缺口：

| 区域 | 缺口 | 目标 |
| --- | --- | --- |
| 模块职责 | `ProcessModule` 仍集中处理设备、状态、CAM 适配和执行器桥接 | 拆出 `ProcessRuntime` 与内部服务 |
| 状态机制 | 当前只有简化 `Idle/Running/Paused/Error/EStop` | 建立完整状态机与合法转换表 |
| CAM 边界 | CAM facade 只提供计数，不提供点集 DTO | 新增 OCC-free `ProcessToolpathSnapshot` |
| 刀路处理 | 未排序、未按图层/工具匹配 | 新增 toolpath service/sorter/matcher |
| 指令生成 | 节点直接触发少量副作用信号 | 新增中间命令缓冲与 controller translator |
| 参数系统 | `uiSetting` 可落地但未 typed 化 | 建 typed schema、默认值、校验和迁移 |
| 设备管理 | descriptor/factory 初步完成 | 建 registry/session/adapter 生命周期 |
| 执行引擎 | QTimer dry-run | 建 execution service、节点 executor、暂停/停止/急停语义 |

## 3. 设计决策

### 3.1 不恢复旧式 Service

不设计旧工程那种全局 `Service` 类。旧式 Service 会把参数、设备、流程、UI、SDK 和全局状态揉在一起，违背微内核边界。

采用以下结构：

```text
ProcessModule             # 模块装配和 facade
  └─ ProcessRuntime        # 内部协调器，非全局单例
       ├─ ProcessStateMachine
       ├─ ProcessSettingsService
       ├─ ProcessDeviceService
       ├─ ProcessToolpathService
       ├─ ProcessInstructionService
       ├─ ProcessWorkflowService
       └─ ProcessExecutionService
```

### 3.2 Process 只消费 CAM 点集

CAM 继续拥有 OCC/CAM 几何，Process 只读取纯数据：轮廓 id、图层 id、工具名、启用状态、点集、机床坐标、法向/切向、版本号。

禁止 Process include：`TopoDS_*`、`AIS_*`、`gp_*`、`Geom_*`、`BRep*`、`XCAF*`。

### 3.3 先生成中间指令，再适配控制器

工作流和刀路生成 controller-neutral command buffer，再由 translator 转换为 PureSimulation/ACS/GTN 对应执行路径。这样切割逻辑不依赖具体控制器 SDK。

## 4. 阶段计划

### Phase 0：文档和边界固化

状态：本轮执行。

任务：

- 清理并重写 `process模块框架.md`。
- 清理并重写 `process模块重构计划.md`。
- 明确不恢复旧式全局 `Service`，改为内部 runtime coordinator。
- 明确 Process 不依赖 OCC，只消费 CAM 点集 DTO。

完成标准：

- 两份文档能作为后续代码重构依据。
- 设计边界、状态机、子模块职责和阶段计划清晰。

### Phase 1：建立 OCC-free CAM 到 Process 数据合同

优先级：最高。

任务：

1. 新增不依赖 OCC 的 contract 头，例如 `src/modules/cam/contracts/process_toolpath_contracts.h` 或 `src/modules/process/toolpath/process_toolpath_types.h`。
2. 定义 `ProcessToolpathPoint`、`ProcessToolpathContour`、`ProcessToolpathLayer`、`ProcessToolpathSnapshot`。
3. 扩展 `ICamFacade` 或新增 `ICamToolpathProvider`，提供 `processToolpathSnapshot()` 和 revision。
4. 在 CAM 模块内把 `LaserToolpath` 转换为纯 DTO，转换发生在 CAM 边界内。
5. 修改 Process dry-run，不再只读取轮廓数/点数，而是读取 snapshot 统计。
6. 增加 include 检查：Process 不 include OCC 类型。

完成标准：

- Process 可获得完整刀路点集、图层和工具信息。
- Process 源码不直接包含 OCC 头。
- Debug 构建通过。

### Phase 2：建立 Process 状态机与运行时协调器

优先级：最高。

任务：

1. 新增 `runtime/process_state_machine.*`，定义 `Uninitialized/Idle/Connecting/Preparing/Ready/Processing/Paused/Stopping/Stopped/Error/EmergencyStop`。
2. 明确所有合法状态转换和拒绝原因。
3. 新增 `runtime/process_runtime.*`，承接 `runStart/runPause/runStop/emergencyStop/reloadSettings/connectDevices` 编排。
4. `ProcessModule` 缩减为 IModule + IProcessFacade，内部只转调 runtime。
5. 所有状态变化统一从 state machine 发出 signal。
6. 加工中、暂停、急停时禁止切换控制器和关键设备参数。

完成标准：

- 状态转换有集中实现和日志。
- UI 状态栏、控制按钮和执行器状态都来自同一状态机。
- `ProcessModule` 不再直接处理执行细节。

### Phase 3：参数系统 typed schema 化

优先级：高。

任务：

1. 新增 `settings/process_settings_schema.*`，定义 key、默认值、范围、单位、描述和迁移版本。
2. 新增 typed DTO：`ProcessDeviceSettings`、`ProcessToolSettings`、`ProcessLayerToolSettings`、`ProcessIoSettings`、`ProcessAuxSettings`。
3. 保留 `.ui` 直接加载和 `uiSetting`，但建立 UI 字段到 typed DTO 的映射表。
4. Apply/OK 时先采集 UI，再校验 schema，再写 TOML。
5. 关键运行逻辑只读取 typed DTO，不直接读 `uiSetting` 字符串。
6. 保存后发送 settings revision changed，runtime 决定是否刷新设备或要求空闲态。

完成标准：

- 运动控制器、激光、轴、工具、图层工具绑定、IO、气水监控等关键参数 typed 化。
- 非法范围能在 UI/日志中明确提示。
- 旧 `uiSetting` 仍可兼容保存。

### Phase 4：设备 registry/session 重构

优先级：高。

任务：

1. 将 `ProcessDeviceManager` 拆为 `ProcessDeviceRegistry` 和 `ProcessDeviceSession`。
2. Registry 负责描述设备、SDK 可用性、能力、缺失原因。
3. Session 负责活动设备实例、连接/断开、错误状态、安全停机、profile 切换。
4. 保留 `PureSimulation` 默认 profile。
5. `SimulatorCMHP` 明确为 ACS SDK simulator profile，不与纯仿真混用。
6. ACS/GTN adapter 继续 option-gated，SDK 头不进入公共接口。
7. 激光器、IO、气、水、监控设备逐步接入 `ILaserDevice/IProcessIo/IProcessAuxDevice`。

完成标准：

- 设备列表能展示 available/unavailable/simulation 和原因。
- 活动设备切换只允许在安全状态。
- Run 前能统一检查 motion/laser/io ready。

### Phase 5：刀路排序与工具匹配服务

优先级：高。

任务：

1. 新增 `toolpath/process_toolpath_service.*`，持有运行时 snapshot 副本。
2. 新增 `process_toolpath_sorter.*`，支持按图层、工具、轮廓顺序、距离策略排序。
3. 新增 `process_tool_matcher.*`，按 layer/toolName/contour 参数匹配 typed 工具参数。
4. 定义 `ProcessJobPlan`：有序轮廓、匹配工具、工艺参数、校验结果。
5. 切割前检查：空点集、禁用图层、缺工具、点机床坐标无效、参数越界。
6. 将 CAM 图层工具配置与 Process 工具参数打通。

完成标准：

- Process 能把 CAM 点集转换为可执行 job plan。
- 同一图层轮廓能使用图层工具参数。
- 排序策略可配置并可日志追踪。

### Phase 6：控制器无关指令规划与 translator

优先级：高。

任务：

1. 新增 `instructions/process_command.h` 和 `ProcessCommandBuffer`。
2. 定义中间命令：`SetFeed`、`MoveLinear`、`LaserOn/Off`、`SetLaserPower`、`SetDigitalOutput`、`Dwell`、`WaitSignal`、`Home`、`Stop`。
3. 新增 `ProcessInstructionPlanner`，把 workflow + job plan 编译成 command buffer。
4. 新增 `IControllerTranslator`。
5. 实现 `PureSimulationTranslator`，用于仿真执行和 UI 反馈。
6. 实现 ACS translator 与 GTN translator 的接口壳，真实 SDK 调用由 adapter 完成。
7. 将 controller-specific 差异从 workflow executor 中移走。

完成标准：

- Cutting 节点不直接调用 ACS/GTN。
- 同一个 job plan 可输出仿真执行动作或真实控制器动作。
- 指令 buffer 可打印日志用于调试。

### Phase 7：工作流服务与执行服务重构

优先级：高。

任务：

1. 新增 `workflow/process_workflow_service.*`，负责编辑态 document、保存加载、校验和编译入口。
2. 新增 `execution/process_execution_service.*`，负责执行 command buffer 和节点 executor。
3. 新增 `process_node_executor_registry.*`，按 executor key 注册节点运行器。
4. 拆分当前 `ProcessWorkflowExecutor` 中的节点副作用到 node executors。
5. 支持暂停、继续、停止、急停对等待、运动、切割、IO 节点的统一取消语义。
6. 节点状态只通过 node id 回写 document/model。
7. catch/log 所有执行异常，禁止异常逃逸线程或 Qt 回调栈。

完成标准：

- 工作流编辑和运行解耦。
- 运行中节点状态、错误和进度稳定回写。
- Stop/EStop 能让激光、IO、运动进入安全态。

### Phase 8：UI 接入与清理验收

优先级：中。

任务：

1. 参数 UI 使用 typed schema 提示单位、范围、错误。
2. 流程树 UI 只操作 workflow service/model，不触碰执行器内部。
3. 设备页面显示 SDK 缺失原因、连接状态和错误。
4. 删除或隔离不再使用的旧兼容源码，保留 `.ui` 资源加载桥。
5. 更新 `实施进度.md`、`文件结构.md`、`微内核框架结构.md`。
6. 补手工 smoke 清单。

完成标准：

- Debug 构建通过。
- Process 源码无 OCC 依赖。
- UI 可完成参数编辑、设备选择、流程编辑、加载保存、仿真运行、暂停、停止、急停。

## 5. 近期执行顺序

1. 先做 Phase 1：新增 CAM 到 Process 的纯数据刀路 DTO，并让 Process dry-run 使用真实点集快照。
2. 再做 Phase 2：引入状态机和 runtime，缩小 `ProcessModule`。
3. 并行推进 Phase 3：把运动、激光、工具、图层工具绑定参数 typed 化。
4. 做 Phase 5 和 Phase 6：把点集排序、工具匹配和指令 buffer 串起来。
5. 最后做 Phase 7/8：替换当前 QTimer dry-run 执行器，完善 UI 和清理旧兼容层。

## 6. 验收清单

### 构建

- `cmake --build build --config Debug -- /m /nologo` 通过。
- SDK-off 必须可构建。
- ACS/GTN option 缺路径时 CMake 明确报错。

### 架构

- `ProcessModule` 不 include QWidget/QDialog/QTreeView/OCC/vendor SDK。
- Process 运行态不 include `LaserToolpath`、`TopoDS_*`、`gp_*`。
- 旧 `.cpp/.h` 业务实现不整包加入 CMake。
- 内部 runtime/services 非全局单例。

### 状态

- Idle/Preparing/Processing/Paused/Stopping/Error/EmergencyStop 都可触达并有日志。
- 非法转换会被拒绝并返回明确原因。
- 急停能中断等待、运动、切割、激光和 IO。

### 数据

- CAM 输出的 Process 点集快照包含 contourId/layerId/toolName/points/revision。
- Process 能按图层和工具参数生成 job plan。
- 保存/加载后 workflow、参数、工具绑定一致。

### 功能

- 仿真 profile 能执行完整 dry-run。
- Cutting 使用 CAM 点集，不依赖 OCC。
- ACS/GTN profile 能通过 translator 进入对应 adapter 路径。
- 参数界面可编辑旧 UI 字段，关键字段有 typed 校验。

## 7. 风险控制

- 不恢复旧全局 `Service`。
- 不把旧 `Process/ProcessModule.cpp`、旧 Setting cpp、旧 device cpp 整包加入 CMake。
- 不把 `SimulatorCMHP` 当作纯软件仿真；它是 ACS SDK simulator profile。
- 不让执行器直接操作 UI item。
- 不让执行逻辑长期依赖 `uiSetting` 字符串 map。
- 不让真实 SDK 路径硬编码到源码。
- 不在加工中切换控制器、工具 schema 或关键设备 profile。
