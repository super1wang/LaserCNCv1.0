# Process 模块重构计划

审阅日期：2026-05-26

目标：在不破坏当前微内核边界的前提下，完成 `device / Process / Setting` 三个旧子模块的合理迁移。最终 Process 模块应支持完整外设迁移、统一参数界面、完整流程节点、流程树拖拽排序、双击编辑节点、节点编辑界面迁移，以及面向仿真和真实设备的流程执行。

## 1. 总目标

### 1.1 用户功能目标

- 参数界面统一为一个入口，左侧为参数树，右侧为对应参数页，交互形态参考原程序。
- 工作流程树迁移所有必要节点，至少覆盖切割流程、IO、运动、视觉、测量、计算、分组、循环、条件、等待、能量切换等节点。
- 流程树支持：
  - 右键添加/删除/启用/禁用节点。
  - 拖拽移动节点，支持同级重排和合法父子关系移动。
  - 双击节点打开对应编辑界面。
  - 保存/加载流程文件。
  - 运行时节点状态高亮。
- 外设完整迁移：运动控制、激光器、IO/气/水/相机等必要外设通过新接口接入。
- 支持纯仿真、半实物和真实设备 profile，未启用 SDK 时工程仍可构建。

### 1.2 架构目标

- `ProcessModule` 只做模块生命周期、service 注册、状态机维护、信号转发和 facade 实现。
- 流程数据不再由 UI 树持有，改为 `ProcessFlowDocument` / workflow service 持有。
- UI/dialog/ribbon 只负责展示和参数收集，不持有业务数据所有权。
- 跨模块调用只走 facade/service，Process 不直接 include CAD/CAM 具体 UI 或 app 细节。
- 外设 adapter 隔离 vendor SDK，真实设备能力通过可选 CMake 目标启用。
- 旧 `Service`、`DT`、全局 boost atomic 和静态 factory 不再作为新框架公开依赖。

## 2. 当前已完成步骤

### 已完成：基础分析与隔离

- 已分析旧 `device / Process / Setting` 依赖，确认旧 `Service` 和旧 Process 执行引擎是依赖集中点。
- 已确认旧 `SimulateCMHPMotionControl` 继承 `ACSMotionControl`，不是纯仿真实现，暂不纳入 CMake。
- 已选择“轻量兼容 + 新框架逐步接入”的迁移策略，避免一次性编译旧模块导致 ACS/GTN/bdaq/LibreCAD/Vision/Boost 依赖扩散。

### 已完成：流程树第一阶段

- 已导入 `Process/qg_processeswidget.*` 并挂到 MainWindow 左侧“执行”tab。
- 已用轻量 Qt6 实现替换旧重依赖 `Process_TreeView`。
- 当前支持节点：Start、Stop、Wait、Axis、Group、If、Loop。
- 已支持流程树 TOML 保存/加载，结构为 `Process.items`。
- 已把流程新建/加载/保存命令接入 Ribbon，并通过 `IProcessFacade` 转到 `ProcessModule`。

### 已完成：参数第一阶段

- 已新增 `ProcessSettings`，持久化基础 process/motion/laser 参数。
- 已新增轻量 `ProcessSettingsDialog`，提供加工、运动、激光三个 tab。
- 已把参数 Ribbon 命令接入，能打开不同初始页。
- 已在参数应用后调用 `ProcessModule::reloadDeviceSettings()` 同步当前设备状态。

### 已完成：外设第一阶段

- 已新增 `ProcessDeviceManager`。
- 当前设备目录只暴露安全仿真项：`SimulatorCMHP` 和 `Simulator`。
- 已保留真实运动控制和激光器旧源码，但未纳入 CMake。
- 已注册 `SimulationMotionController` 为 `IMotionController` service。

### 已完成：构建验证

- `cmake --build build --config Debug -- /m /nologo` 已通过。
- 当前仅有既有 `qrc_resources.cpp.obj` PDB 警告。

## 3. 主要缺口

| 区域 | 当前缺口 | 风险 |
| --- | --- | --- |
| 模块边界 | `ProcessModule` 持有 `ProcessTreeView*` | module 依赖 UI，后续执行/保存难以测试 |
| 流程树模型 | 节点无 stable id，拖拽 API 未完整实现 | 保存、拖动、执行定位容易不稳定 |
| 节点迁移 | 只支持 7 个轻量节点 | 不能满足切割和完整工艺流程 |
| 节点编辑 | 双击未打开编辑器 | 节点参数不能配置 |
| 参数界面 | 当前是 tab 结构 | 不符合统一左树右页要求 |
| 参数数据 | 只覆盖少数字段 | 旧工艺、轴、IO、气、水、相机参数缺失 |
| 外设 | 只有仿真设备目录 | 无法连接真实控制器/激光器 |
| 执行引擎 | 新执行器未建立 | 流程树只能编辑保存，不能真实运行 |
| 旧依赖 | 旧源码仍引用 Service、DT、Vision、LibreCAD、Boost、SDK | 直接编译会破坏微内核边界和构建稳定性 |

## 4. 目标架构拆分

### 4.1 Workflow 子系统

新增建议目录：

```text
src/modules/process/workflow/
├── process_node_type.h
├── process_node.h
├── process_flow_document.h/.cpp
├── process_flow_store.h/.cpp
├── process_node_registry.h/.cpp
├── process_node_validation.h/.cpp
└── process_executor.h/.cpp
```

职责：

- `ProcessNode`：保存 stable id、type、name、enabled、children、typed parameters。
- `ProcessFlowDocument`：流程树业务事实源，提供增删改移、查询和 dirty 状态。
- `ProcessFlowStore`：负责 TOML 读写、版本号和旧流程格式迁移。
- `ProcessNodeRegistry`：注册节点元信息、可放置规则、默认参数、编辑器 factory 和执行器 factory。
- `ProcessExecutor`：解释执行流程树，负责暂停、继续、停止、急停、运行状态回传。

### 4.2 UI 子系统

新增或重命名建议目录：

```text
src/modules/process/ui/
├── process_flow_model.h/.cpp
├── process_flow_tree_view.h/.cpp
├── process_settings_dialog.h/.cpp
├── process_settings_tree_model.h/.cpp
├── node_editors/
└── settings_pages/
```

职责：

- `ProcessFlowModel` 是 `QAbstractItemModel` adapter，不拥有业务事实。
- `ProcessFlowTreeView` 只处理视图、右键菜单、拖放手势、双击触发编辑。
- `node_editors/*` 每个节点一个编辑器或共享编辑器，负责参数收集和校验提示。
- `ProcessSettingsDialog` 统一为左侧参数树 + 右侧 `QStackedWidget`。
- `settings_pages/*` 迁移旧 MotionControl、Axis、IO、Laser、Tool、Gas、Water、Monitor、LoadingPos、Camera 等页面。

### 4.3 Device 子系统

新增建议结构：

```text
src/modules/process/device/
├── process_device_registry.h/.cpp
├── process_device_profile.h/.cpp
├── i_laser_device.h
├── i_process_io.h
├── i_aux_device.h
├── motion/
│   ├── simulator_motion_controller_adapter.*
│   ├── acs_motion_controller_adapter.*
│   └── gtn_motion_controller_adapter.*
└── laser/
    ├── simulator_laser_device.*
    ├── ipg_laser_device_adapter.*
    ├── pharos_laser_device_adapter.*
    ├── raycus_laser_device_adapter.*
    ├── raycus_qcw_laser_device_adapter.*
    └── analog_laser_device_adapter.*
```

职责：

- `ProcessDeviceRegistry`：管理设备类型、可用性、当前 profile、活动实例和状态信号。
- `IMotionController`：优先复用现有 `core/kinematics/i_motion_controller.h`，真实 adapter 适配到此接口。
- `ILaserDevice`：定义 start/stop/aiming/energy/frequency/pulseWidth/status 等统一能力。
- `IProcessIo` / `IAuxDevice`：封装数字 IO、模拟 IO、气、水、门禁、传感器等能力。
- Vendor SDK 依赖通过 CMake option 控制，例如 `LCNC_WITH_ACS`、`LCNC_WITH_GTN`、`LCNC_WITH_BDAQ`。

### 4.4 Settings 子系统

新增建议结构：

```text
src/modules/process/settings/
├── process_settings.h/.cpp
├── process_settings_schema.h/.cpp
├── process_settings_store.h/.cpp
├── process_device_settings.h
├── process_tool_settings.h
└── process_io_settings.h
```

职责：

- 统一 process 参数持久化，不再让 UI 页直接读写旧全局 `Settings`。
- 每类参数有默认值、TOML key、版本迁移和校验。
- 页面只读写 settings DTO，点击 Apply 后由 settings store 保存并通知相关 service。

## 5. 分阶段计划

### 阶段 0：现状固化与文档化（当前阶段）

状态：进行中。

任务：

- 生成 `process模块框架.md`。
- 生成 `process模块重构计划.md`。
- 记录当前已完成能力、已导入但未接入的旧源码、构建状态和主要风险。

完成标准：

- 文档能指导后续开发顺序。
- 明确哪些旧文件不能直接加入 CMake。

### 阶段 1：拆除 UI 反向依赖，建立流程数据模型

优先级：最高。

任务：

1. 新建 `workflow/process_node_type.h`，收敛 `ItemType`，保留旧类型映射。
2. 新建 `ProcessNode`，字段至少包括：`id`、`type`、`name`、`enabled`、`state`、`parameters`、`children`。
3. 新建 `ProcessFlowDocument`，提供 `appendNode`、`removeNode`、`moveNode`、`updateNodeParameters`、`nodeById`。
4. 新建 `ProcessFlowStore`，读写 TOML，加入 `schemaVersion`。
5. `ProcessModule` 改为持有 `ProcessFlowDocument` 或 workflow service，不再持有 `ProcessTreeView*`。
6. `IProcessFacade` 新增面向流程文档的接口，UI 通过 facade 或 model service 操作流程。

完成标准：

- 新建/加载/保存流程不再依赖 `ProcessTreeView*`。
- 当前 7 个轻量节点可通过新 document 保存/加载。
- 旧 TOML `Process.items` 可兼容导入。

### 阶段 2：流程树 UI 完整化

优先级：高。

任务：

1. 新建或改造 `ProcessFlowModel`，绑定 `ProcessFlowDocument`。
2. 实现 `flags()`、`supportedDropActions()`、`mimeData()`、`dropMimeData()` 或 `moveRows()`。
3. 实现合法拖放规则：
   - `Group`、`If`、`Loop`、`RunGroup` 可作为容器。
   - `Start`/`Stop` 默认限制为顶层或流程结构规则允许位置。
   - 禁止把节点拖到自身子树中。
4. 双击节点时触发 `editNodeRequested(nodeId)`。
5. 节点状态显示由 model role 提供，view 只根据 role 绘制。
6. 保留右键菜单添加/删除/启用/禁用/清空/保存/加载。

完成标准：

- 拖拽移动同级节点和移动到容器节点均可工作。
- 双击节点能打开至少基础节点编辑器。
- 节点移动后保存/加载顺序稳定。

### 阶段 3：统一参数界面

优先级：高。

任务：

1. 将当前 `ProcessSettingsDialog` 从 tab 改为左侧树 + 右侧 `QStackedWidget`。
2. 建立参数页 registry：每个页面有 id、标题、父节点、权限等级、widget factory。
3. 第一批迁移页面：
   - External / Motion Controller
   - External / Axis
   - External / Laser
   - Processing / Tool / Motion&Laser
   - Processing / Tool / General
4. 第二批迁移页面：
   - IO Index / Digital IN/OUT / Analog IN/OUT
   - Gas
   - Water
   - Monitor
   - LoadingPos
   - Camera
   - Internet
5. 从旧 `QG_dlgSetting` 复制交互结构，但不复制旧 `Service*` 和 `DT` 依赖。
6. 将页面数据接入 `ProcessSettings` 或拆分后的 settings store。
7. Apply/OK 统一保存并发出 settings changed 信号。

完成标准：

- 参数命令只打开一个统一设置窗口。
- 左侧参数树选择能切换右侧页面。
- 至少运动、轴、激光、工具、气、水、监控、上料位页面可保存/加载。
- 旧 tab 对话框入口被替换或只作为兼容内部实现。

### 阶段 4：节点定义和节点编辑器迁移

优先级：高。

任务：

1. 建立 `ProcessNodeRegistry`，为每种节点注册：显示名、图标、默认参数、是否容器、允许子节点、编辑器 factory、执行器 key。
2. 迁移基础节点编辑器：Start、Stop、Wait、Axis、Group、If、Loop。
3. 迁移运动节点编辑器：AxesMove、Feeding、AutoFocus。
4. 迁移 IO/命令节点编辑器：IO、Commands、EnergySwitch、Monitor。
5. 迁移工艺节点编辑器：Cutting、OverCutting。
6. 迁移视觉/测量节点编辑器：Camera、Measurement、MarkAcquire、Alignment、Calculation、Compare。
7. 迁移 RunGroup 和 RunGroupCheck。
8. 每个节点编辑器输出类型化参数，并可转换到旧 `map<QString, QString>` 以兼容旧流程文件。

完成标准：

- 必要节点均可在树中添加。
- 双击每种节点能打开对应参数界面。
- 编辑后节点 Info 列能摘要显示关键参数。
- 节点参数可保存/加载并通过校验。

### 阶段 5：流程执行引擎迁移

优先级：高。

任务：

1. 新建 `ProcessExecutor`，解释 `ProcessFlowDocument`。
2. 使用 Qt 或 `TaskManager` 管理后台执行，替换旧 boost thread 和全局 atomic。
3. 建立执行状态：Idle、Preparing、Running、Paused、Stopping、Stopped、Error、EmergencyStop。
4. 节点执行器通过 registry 获取，执行时只调用 facade/service：运动、激光、IO、相机、CAM 数据。
5. 实现暂停、继续、停止、急停和节点状态回传。
6. 第一批执行节点：Start、Stop、Wait、Axis、AxesMove、Group、If、Loop。
7. 第二批执行节点：IO、Commands、EnergySwitch、Feeding、Cutting、OverCutting。
8. 第三批执行节点：Measurement、MarkAcquire、Alignment、AutoFocus、Camera、Calculation、Compare、Monitor、RunGroup。

完成标准：

- 流程树可以在仿真设备上执行。
- 运行状态能回写流程树节点颜色。
- 停止/急停不会让线程异常逃逸。
- 所有 catch 块记录 `LCNC_ERR` 日志。

### 阶段 6：外设完整迁移

优先级：高。

任务：

1. 定义 `ILaserDevice`、`IProcessIo`、`IProcessAuxDevice`。
2. 将 `ProcessDeviceManager` 升级为 `ProcessDeviceRegistry`，支持设备描述、可用性、active profile、错误状态。
3. 新增 `SimulatorLaserDevice`，使仿真 profile 覆盖运动和激光。
4. 迁移运动控制 adapter：
   - `ACSMotionControl` adapter。
   - `GTNMotionControl` adapter。
   - `SimulatorCMHP` 作为纯仿真 adapter，不再继承 ACS。
5. 迁移激光 adapter：
   - IPG。
   - Pharos。
   - Raycus。
   - RaycusQCW。
   - AnalogControl。
6. 迁移 IO/模拟量/气水相关 adapter，替换旧 bdaq 直接 include。
7. CMake 增加可选开关：
   - `LCNC_WITH_ACS`
   - `LCNC_WITH_GTN`
   - `LCNC_WITH_BDAQ`
   - `LCNC_WITH_REAL_LASER`
8. 真实 SDK 未启用时，设备显示为 unavailable，但工程仍可构建。
9. 参数界面根据 registry 动态列出可用设备和缺失原因。

完成标准：

- 仿真 profile 默认可用。
- 真实设备 profile 可按 CMake option 启用。
- 不再需要旧静态 `MCFactory` / `LDFactory` 作为全局入口。
- 设备连接、断开、状态和错误能通过 Process facade/UI 展示。

### 阶段 7：切割流程和 CAM 集成

优先级：中高。

任务：

1. 明确切割节点输入：CAM contour stable id、刀路/轮廓集合、工艺参数、激光参数、进给参数。
2. `Cutting` 节点执行时通过 Cam facade 获取所需 runtime 数据，避免直接 include CAM UI。
3. `OverCutting` 节点迁移旧过切参数和执行逻辑。
4. 对齐/打标/测量节点通过 Vision 或未来视觉 facade 获取数据，不直接依赖旧 `VisionModule::instance()`。
5. 切割前安全检查：机台已加载、轴已回零、设备已连接、激光 ready、流程参数有效、急停未触发。
6. 仿真模式下以 `MachinePose` 和 CAM 仿真显示反馈运行。

完成标准：

- 至少一个完整切割流程可在仿真 profile 运行。
- 真实设备 profile 下能完成连接前检查和 dry-run。
- 切割节点错误能定位到节点并显示错误信息。

### 阶段 8：清理旧兼容层和验收

优先级：中。

任务：

1. 移除或隔离不再使用的旧 `ProcessModule.*`、旧 `Service` 依赖和旧全局宏。
2. 将仍需保留的旧代码放入 `legacy/` 或 adapter private 区域。
3. 统一文件命名为 snake_case，新增公共类补齐用途说明。
4. 清理 `TreeItem` / `TreeModel` 旧实现或替换为新 model。
5. 更新 `文件结构.md`、`实施进度.md` 和相关 README。
6. 增加 smoke 验证清单。

完成标准：

- 新代码通过分层 include 检查。
- `ProcessModule` 不 include QWidget/QDialog/QTreeView。
- Debug 构建通过。
- 完成执行页、参数界面、流程树、仿真运行、设备 profile 的手工 smoke。

## 6. 节点迁移优先级

### P0：流程骨架节点

- Start
- Stop
- Wait
- Group
- If
- Loop
- RunGroup
- RunGroupCheck

原因：这些节点决定流程树结构、执行控制和拖放规则。

### P1：运动与切割主链路

- Axis
- AxesMove
- Feeding
- Cutting
- OverCutting
- EnergySwitch

原因：这是加工流程最小闭环。

### P2：IO 与辅助设备

- IO
- Commands
- Monitor
- AutoFocus

原因：与外设迁移强相关，但可以在仿真接口稳定后分批接入。

### P3：视觉、测量和计算

- Camera
- Measurement
- MarkAcquire
- Alignment
- Calculation
- Compare

原因：这些节点依赖视觉、CAD/CAM 几何上下文和旧 LibreCAD 选择逻辑，需要先定义 facade 边界。

## 7. 参数界面迁移优先级

### P0：统一壳和基础页

- 左侧参数树。
- 右侧 stacked page。
- Apply/OK/Cancel。
- Motion Controller。
- Axis。
- Laser。
- Tool / Motion&Laser。
- Tool / General。

### P1：工艺和辅助页

- Tool / Servo。
- Tool / Special。
- Gas。
- Water。
- Monitor。
- LoadingPos。

### P2：IO 与通讯

- IO Index。
- Digital IN/OUT。
- Analog IN/OUT。
- Internet。
- TCP/SMC/SignalSource/Sensor 等旧独立对话框按需要合并。

### P3：相机和客户差异

- Camera。
- 客户/权限相关页面显隐。
- 导入/导出配置。

## 8. 外设迁移优先级

### P0：仿真设备完善

- `SimulatorCMHP` 纯仿真运动控制器。
- `Simulator` 激光器。
- 仿真 IO/气/水状态。

### P1：激光器

- Pharos：已有 HTTP 风格实现，适合先 adapter 化。
- IPG。
- Raycus。
- RaycusQCW。
- AnalogControl。

### P2：运动控制

- ACSCMHP：SDK 依赖重，必须可选启用。
- GTN：SDK 和 bdaq 依赖需要独立 CMake 开关。
- 旧 SimulateCMHP：不沿用继承 ACS 的实现，改为纯仿真或 ACS simulator profile。

### P3：IO/辅助硬件

- bdaq IO。
- 气压/水泵/门禁/传感器。
- 相机触发和外部信号源。

## 9. 验收清单

### 构建验收

- Debug 构建通过：`cmake --build build --config Debug -- /m /nologo`。
- 不启用真实 SDK 时仍可构建。
- 启用每个 SDK option 时缺失路径有明确 CMake 错误。

### 分层验收

- `core/**` 不 include `view/`、`modules/`、`app/`。
- `view/**` 不 include `modules/`、`app/`。
- `ProcessModule` 不 include QWidget/QDialog/QTreeView。
- 外设 adapter 不把 vendor SDK 头扩散到公共接口。

### 功能验收

- 参数窗口为左树右页，一个入口覆盖 process 参数。
- 流程树支持添加、删除、拖拽移动、双击编辑、保存、加载。
- 必要节点均可添加并保存参数。
- 仿真 profile 能执行基础流程。
- 切割流程能在仿真 profile 下完成 dry-run。
- 真实设备 profile 能完成设备发现、连接、断开、状态展示和错误提示。

### 稳定性验收

- 后台执行线程 catch/log 所有异常，不让异常逃逸线程栈。
- 急停能打断等待、运动、激光和 IO 节点。
- 拖拽移动后节点 stable id 不变。
- 保存/加载后节点顺序、父子关系和参数一致。

## 10. 近期推荐执行顺序

1. 建立 `workflow` 数据模型和 TOML schema，先把当前轻量树从 UI 事实源迁出来。
2. 重做流程树 model 的拖放和双击编辑信号，保持当前 UI 可用。
3. 把参数窗口改为左树右页统一壳，先迁移当前已有 process/motion/laser 字段。
4. 建立节点 registry，迁移基础节点编辑器。
5. 建立 `ILaserDevice` 和 simulator laser，让设备 profile 闭环。
6. 迁移 Cutting / OverCutting 节点编辑器和仿真执行。
7. 再按 adapter 方式迁移真实激光器和真实运动控制器。

## 11. 风险控制

- 不要把旧 `Process/ProcessModule.cpp` 直接加入 CMake；它会拉入旧 Service、Boost、LibreCAD、Vision 和全局状态。
- 不要把旧 `SimulateCMHPMotionControl` 当纯仿真接入；它继承 ACS 控制器。
- 不要在 `ProcessModule` 中继续增加 UI 对话框或 QWidget 依赖。
- 不要让节点执行器直接操作 UI item；必须通过 node id 和 workflow document。
- 不要让 settings 页面直接保存全局旧 table；先转换为新 settings DTO。
- 不要在 CMake 长期硬编码个人 SDK 路径；使用 cache variable 或 toolchain。
