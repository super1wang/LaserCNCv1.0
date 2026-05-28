# Process 模块框架

审阅日期：2026-05-27

本文记录当前 `src/modules/process` 的真实完成状态、现有框架边界、已接入能力、遗留导入代码和下一阶段重构约束。后续实现以本文、`代码规范.md`、`微内核框架结构.md` 和 `process模块重构计划.md` 为准。

## 0. 2026-05-26 阶段 1-8 实施后状态

本轮已把 Process 模块推进到可运行的新框架闭环。若后文历史审阅缺口与本节冲突，以本节和 `实施进度.md` 为准。

- `ProcessModule` 已彻底移除旧 `ProcessTreeView*` 反向依赖，新建、加载、保存和运行只以 `ProcessFlowDocument` / `ProcessFlowStore` 为事实源。
- `ui/process_flow_model.*`、`ui/process_flow_tree_view.*` 已承接左侧执行页，支持 stable id、拖放、右键增删启禁、清空、保存/加载、双击编辑和运行状态刷新。
- `workflow/process_node_registry.*` 统一管理全部当前节点的 metadata、默认参数、摘要、可放置规则和执行 key；流程树新增菜单、Info 列和拖放规则均从 registry 取数。
- `ui/process_node_edit_dialog.*` 已支持 Wait、Axis 以及 AxesMove、Feeding、Cutting、OverCutting、EnergySwitch、IO、Camera、Measurement、MarkAcquire、Alignment、AutoFocus、Monitor、Commands、Loop、RunGroup、If、Compare、Calculation 等节点的 typed 参数编辑；未知参数仍可通过通用参数表兜底。
- `Setting/process_settings_dialog.*` 已改为左侧树 + 右侧 `QStackedWidget`，页面覆盖 Process、Motion Controller、Laser、Axis、Tool、IO、Gas、Water、Monitor、LoadingPos、Camera、Internet、Communication，以及 Legacy Setting 全量旧 `.ui` 字段镜像页。
- `ProcessSettings` 已扩展对应基础参数、统一通讯参数和 `legacySetting` 嵌套参数表并持久化到 `process.toml`；旧字段已经可保存，后续重点是 typed schema、默认值、单位和校验。
- `device/process_device_manager.*` 已拥有仿真激光 `SimulatorLaserDevice`、仿真 IO `SimulatorProcessIo` 和运动控制 profile 工厂；默认 `PureSimulation` 无 SDK 依赖，`SimulatorCMHP/ACS` 由 ACS SDK option 启用，`GTN` 由 GTN SDK option 启用。
- `communication/**` 已提供统一通讯接口，支持 Mock、TCP、HTTP、Serial 通道，带配置 UI、状态监控和收发日志；真实激光器等外设可在此基础上继承或组合通讯能力。
- `ProcessWorkflowExecutor` 已能按流程树顺序异步推进节点，Run/Pause/Resume/Stop/EStop 按钮控制执行生命周期，节点状态回写流程树；Axis/AxesMove、EnergySwitch、IO、Cutting/OverCutting 已有仿真副作用或明确 dry-run 反馈。
- `ICamFacade` 暴露只读刀路摘要，Cutting dry-run 可读取 CAM 当前轮廓/点数；非 dry-run 切割在无刀路时明确失败，不直接依赖 CAM UI。
- Debug 构建已多次验证通过；当前仍仅可能出现既有 `qrc_resources.cpp.obj : warning LNK4099` PDB 警告。

## 1. 当前定位

Process 模块在 LaserCNC 微内核中的职责是加工执行、流程编排、运行状态、参数管理与外设控制。当前工程已经把 Process 挂入模块系统：

```text
Kernel
  └─ ModuleRegistry
       └─ ProcessModule(id="process", depends=["cam"])
            ├─ IProcessFacade service
            ├─ IMotionController service: active PureSimulation / SimulatorCMHP / ACS / GTN
            ├─ ProcessSettings
            ├─ ProcessDeviceManager
            ├─ ProcessFlowDocument
            ├─ ProcessNodeRegistry
            ├─ ProcessWorkflowExecutor
            └─ CAM toolpath snapshot provider
```

当前实现已形成“流程文档 + registry + model/view + typed editor + executor + simulator device”的新闭环。旧程序中的真实 SDK adapter、复杂 Setting 表、真实相机/测量服务和完整切割实控逻辑仍需在新接口后继续迁移。

## 2. 当前目录状态

```text
src/modules/process/
├── process_module.h/.cpp                 # 新微内核 ProcessModule 与 IProcessFacade 实现
├── i_process_facade.h                    # 对 app/UI 暴露的 process facade contract
├── commands/commands_process.*           # Ribbon 命令：流程、参数、连接、运行、安全
├── controllers/simulation_motion_controller.*
│                                          # 纯软件仿真运动控制器，默认注册为 IMotionController
├── controllers/acs_motion_controller_adapter.*
│                                          # ACS 与 ACS-based SimulatorCMHP adapter，LCNC_WITH_ACS 时编译
├── controllers/gtn_motion_controller_adapter.*
│                                          # 固高 GTN adapter，LCNC_WITH_GTN 时编译
├── communication/                         # 统一通讯接口、Mock/TCP/HTTP/Serial 通道、日志模型和设置页
├── device/
│   ├── process_device_manager.*          # 设备目录/活动设备管理器，负责 profile 可用性和控制器 factory
│   ├── MotionControl/*                   # 旧 yuncocore2 运动控制源码，已导入但未纳入 CMake
│   └── Laser/*                           # 旧 yuncocore2 激光器源码，已导入但未纳入 CMake
├── settings/process_settings.*           # 新 Process TOML 配置入口
├── workflow/
│   ├── process_node_type.h               # 新流程节点类型与状态枚举
│   ├── process_node.h/.cpp               # 新流程节点数据结构，包含 stable id
│   ├── process_flow_document.h/.cpp      # 新流程树业务事实源
│   └── process_flow_store.h/.cpp         # 新/旧流程 TOML 读写与兼容转换
├── execution/
│   └── process_workflow_executor.*       # 新流程执行器壳，负责运行计划准备和生命周期状态
├── ui/
│   ├── process_flow_model.*              # 新流程文档 QAbstractItemModel adapter
│   ├── process_flow_tree_view.*          # 新流程树 view，右键、拖放、双击编辑、文件操作
│   ├── process_node_edit_dialog.*        # 通用节点编辑器壳
│   ├── widget_laser_control.*            # 右侧运行/点动/安全控制面板
│   └── ribbon_process_tab.*              # Process ribbon tab
├── Setting/
│   ├── process_settings_dialog.*         # 左树右页参数对话框，含基础页、通讯页和旧 Setting 字段镜像页
│   └── qg_/Setting_*                     # 旧参数界面源码和 .ui 资源，.ui 已嵌入 qrc 用于字段镜像
├── Process/
│   ├── qg_processeswidget.*              # 左侧“执行”页容器，当前编译
│   ├── ProcessModule/Process_TreeView.*  # 当前编译的轻量流程树
│   ├── ProcessModule/treeitem.*          # 当前编译的旧树节点兼容类
│   ├── ProcessModule/treemodel.*         # 当前编译的旧树模型兼容类
│   ├── ProcessModule/mimeData.*          # 当前编译但尚未真正用于完整拖放
│   └── ProcessModule/Process_*           # 旧流程节点/编辑界面源码，已导入但未纳入 CMake
└── legacy/
    ├── legacy_process_types.h            # 旧 DataType 最小兼容类型
    └── legacy_message_adapter.h          # 旧 MessageModule 最小日志兼容入口
```

当前 `CMakeLists.txt` 纳入的是新框架文件与安全兼容文件：`process_module.cpp`、`simulation_motion_controller.cpp`、`process_device_manager.cpp`、`execution/process_workflow_executor.cpp`、`process_settings.cpp`、`workflow/process_node.cpp`、`workflow/process_flow_document.cpp`、`workflow/process_flow_store.cpp`、`process_settings_dialog.cpp`、`ui/process_flow_model.cpp`、`ui/process_flow_tree_view.cpp`、`ui/process_node_edit_dialog.cpp`、`qg_processeswidget.cpp/.ui`、旧树兼容文件、`commands_process.cpp`、`widget_laser_control.cpp`、`ribbon_process_tab.cpp`。

## 3. 已完成能力

### 3.1 微内核接入

已完成：

- `ProcessModule` 实现 `lcnc::IModule`，模块 id 为 `process`，依赖 `cam`。
- `ProcessModule::init()` 注册 `ProcessModule` service 和 `lcnc::IProcessFacade` service。
- `ProcessModule` 根据当前 settings profile 创建并注册活动 `lcnc::IMotionController`，默认是 `PureSimulation`。
- `MainWindow` 在左侧创建“执行”页，UI 绑定 `ProcessFlowDocument` 的 model/view，不再把树控件传入 module。
- `MainWindow` 右侧已有 `WidgetLaserControl`，用于轴位置、点动、运行/暂停/停止、急停、倍率和状态显示。

当前不足：

- 真实控制器 profile 的 SDK-on 编译和硬件联调尚未完成。
- 运行命令已解释当前流程树节点，但条件/循环/并行等复杂语义仍需继续增强。

### 3.2 Facade 与命令

已完成：

- `IProcessFacade` 暴露连接、断开、仿真模式、运行、暂停、停止、急停、复位、回零、新建流程、加载流程、保存流程和状态文本。
- Ribbon 已注册流程命令：新建流程、加载流程、保存流程。
- Ribbon 已注册参数命令：加工设置、运动参数、激光参数。
- Ribbon 已注册运行/安全命令：运行、暂停、停止、回零、急停、复位急停、连接、断开、仿真模式。
- `ProcessModule::newProcess/loadProcess/saveProcess` 通过 `ProcessFlowDocument` + `ProcessFlowStore` 读写流程数据。

当前不足：

- 命令层仍直接创建 `QFileDialog`、`QInputDialog` 和 `ProcessSettingsDialog`，这在现阶段可接受，但后续可考虑通过 app 对话服务统一。
- `CmdToggleSimulationMode` 构造时连接 facade 信号，若命令早于 service 完整初始化，需要确认生命周期稳定性。
- 参数命令打开统一左树右页对话框，并可定位到 Process/Motion/Laser 初始页。

### 3.3 流程树

已完成：

- 左侧“执行”tab 已显示 `QG_ProcessesWidget`。
- 当前 `ProcessFlowTreeView` 支持 registry 驱动的右键添加、删除、启用、禁用、清空、保存、加载、拖放和双击编辑。
- 当前可添加节点覆盖 Start、Stop、Wait、Axis、AxesMove、Feeding、AutoFocus、Cutting、OverCutting、EnergySwitch、IO、Commands、Monitor、Camera、Measurement、MarkAcquire、Alignment、Loop、RunGroup、If、Compare、Calculation 等类型。
- `ProcessFlowStore` 输出 `Process.schemaVersion` 和 `Process.nodes`，同时兼容读取旧 `Process.items`。

当前不足：

- 旧专用节点 UI 的细节交互和复杂校验仍需按节点逐步补齐。
- 复杂嵌套拖放、保存加载和运行状态需要补充人工 smoke 记录。

### 3.4 参数界面

已完成：

- `ProcessSettingsDialog` 是左树右页结构，覆盖基础 process/motion/laser、轴、工具、IO、气、水、监控、上料位、相机、联网、通讯和 Legacy Setting 镜像页。
- `ProcessSettings` 持久化到 `<exeDir>/config/process.toml`，包含基础字段、通讯字段和 `legacySetting` 嵌套参数表。
- 参数保存后会调用 `ProcessModule::reloadDeviceSettings()` 同步仿真模式和活动设备。

旧界面资源迁移方式：

- 旧 `Setting.ui`、`Setting_*` 和 qg 细分页 `.ui` 已嵌入 `resources.qrc`。
- 新对话框运行时解析旧 `.ui` 中的可编辑控件，生成 Legacy Setting 参数页，不链接旧 `Settings/Service/DT`。

当前不足：

- Legacy Setting 当前是字段镜像，后续需要把关键参数提升为 typed schema、默认值、单位、范围和设备刷新事件。
- 旧列表/表格类控件若有业务语义，需要单独迁移为 DTO 和专用编辑器。

### 3.5 外设和仿真

已完成：

- `SimulationMotionController` 是新框架中的纯仿真运动控制器，通过 `MachinePose` 更新姿态。
- `ProcessDeviceManager` 当前作为设备目录和 profile 工厂，暴露：
  - 运动控制器：`PureSimulation`、`SimulatorCMHP`、`ACS`、`GTN`
  - 激光器：`Simulator`
- `ProcessDeviceManager::syncFromSettings()` 能从 `ProcessSettings` 同步活动设备，不合法时回退到仿真设备。
- `ProcessModule` 持有活动运动控制器实例，设置变更时在非运行状态下可切换 controller。

已导入但未接入的旧外设：

- 运动控制：`MotionControl`、`ACSMotionControl`、`SimulateCMHPMotionControl`、`GTNMotionControl`、`MCFactory`。
- 激光器：`LaserDevice`、`IPGLaserDevice`、`PharosLaserDevice`、`RaycusLaserDevice`、`RaycusQCWLaserDevice`、`AnalogLaserDevice`、`LDFactory`。

当前不足：

- 旧 `SimulateCMHPMotionControl` 继承 `ACSMotionControl`，会拉入 ACSC SDK 和模拟器文件，不能作为纯仿真直接接入。
- 旧 `MCFactory` / `LDFactory` 使用静态全局设备对象，不适合新微内核生命周期。
- 旧外设日志、错误码、参数表、线程模型和 SDK 路径尚未抽象为新接口。
- 当前没有统一的 `ILaserDevice`、`IProcessDeviceRegistry`、IO/气/水/相机辅助设备 service。

## 4. 必迁流程节点清单

旧流程节点已经导入源码，但当前只有部分轻量节点进入实际 UI。后续完整迁移至少覆盖以下 `ItemType`：

| 类别 | 节点 | 当前状态 |
| --- | --- | --- |
| 起止与结构 | Start、Stop、Group、RunGroup、RunGroupCheck、If、Loop、Wait | 除 RunGroupCheck 外已进入 registry、编辑和执行框架；复杂语义待增强 |
| 运动 | Axis、AxesMove、Feeding、AutoFocus、LoadingPos 相关流程 | Axis/AxesMove/Feeding/AutoFocus 已有 typed 参数和基础执行反馈；LoadingPos 待补专用语义 |
| IO/外设 | IO、Commands、EnergySwitch、Monitor、Camera | 已有 typed 参数和仿真/日志执行反馈；真实服务 adapter 待接入 |
| 工艺 | Cutting、OverCutting | 已有 typed 参数、校验和 CAM dry-run；非 dry-run 实控待接入 |
| 视觉/测量 | Measurement、MarkAcquire、Alignment、Calculation、Compare | 已进入 registry/编辑器/执行框架；真实视觉和测量服务待接入 |

后续节点迁移不能简单把旧 `Process_*.cpp` 全部加入 CMake。应先建立新节点定义、参数 schema、编辑器接口和执行器接口，再逐个迁移旧 UI 与旧执行逻辑。

## 5. 架构合规审阅

### 当前符合规范的方向

- Process 已通过 `IModule` 和 `IProcessFacade` 接入微内核。
- 命令大多通过 `IProcessFacade` 调用模块能力。
- 旧外设未直接加入 CMake，避免把 ACS/GTN/bdaq/旧 Service 依赖一次性引入。
- 活动运动控制器作为 `IMotionController` service 暴露给其他模块，默认 profile 无 SDK 依赖。
- 参数持久化已收敛到 `ProcessSettings` 和 TOML，旧 `.ui` 仅作为字段 schema 资源使用。
- 统一通讯模块与业务外设解耦，真实设备可复用 Mock/TCP/HTTP/Serial 通道。

### 当前需要重构的边界问题

- ACS/GTN adapter 源已隔离 SDK include，但仍需 SDK-on 编译和硬件联调。
- 旧 Setting 字段当前为镜像页，关键业务字段应继续转换为 typed schema + 校验 + 刷新事件。
- 真实外设 SDK 不能硬编码个人路径；继续通过 CMake cache option、toolchain 或独立 adapter 目标接入。
- 真实激光器、BDAQ、相机和测量服务仍不能直接编译旧源码，应按新接口迁移。

## 6. 当前完成度评估

| 子域 | 完成度 | 说明 |
| --- | --- | --- |
| 微内核模块接入 | 90% | 模块、facade、workflow、active motion service 已接入；真实 SDK 验证待补 |
| Ribbon 命令 | 80% | 运行/流程/参数命令已接入；对话服务和错误反馈仍可统一 |
| 执行页 UI | 90% | 新流程树、右控制面板、状态回写、双击编辑和拖放基础可用 |
| 流程树保存/加载 | 90% | stable id、schema、legacy 兼容和节点状态已完成；复杂 smoke 待记录 |
| 节点迁移 | 80% | 当前节点已入 registry/typed editor/executor 框架；旧专用 UI 细节待补 |
| 参数界面 | 85% | 左树右页、通讯页和旧 Setting 全量字段镜像已完成；typed schema 待提升 |
| 外设迁移 | 65% | 仿真激光/IO、通讯模块、ACS/GTN adapter 源已接入；SDK-on 和真实激光/BDAQ 待补 |
| 流程执行引擎 | 75% | QTimer 顺序执行、按钮控制和 dry-run 可用；复杂语义和实控待增强 |
| 构建稳定性 | 90% | SDK-off Debug 构建通过；SDK-on 矩阵待验证 |

## 7. 当前构建验证

最近一次已验证命令：

```powershell
cmake --build build --config Debug -- /m /nologo
```

结果：构建通过，仅保留 `qrc_resources.cpp.obj : warning LNK4099` 的既有 PDB 警告。

如果后续出现 `LNK1168`，优先检查是否有正在运行的 `LaserCNC.exe`：

```powershell
Get-Process LaserCNC -ErrorAction SilentlyContinue | Select-Object Id,ProcessName,Path
```

## 8. 框架目标形态

最终 Process 模块应收敛为：

```text
src/modules/process/
├── process_module.*                 # 只负责生命周期、service 注册、状态转发
├── i_process_facade.h               # UI/app 可调用的稳定 contract
├── workflow/
│   ├── process_flow_document.*      # 流程文档，稳定节点 id 和类型化参数
│   ├── process_node_registry.*      # 节点定义、编辑器、执行器注册
│   ├── process_flow_store.*         # TOML/.lcnc 持久化和 schema 迁移
│   └── process_executor.*           # 流程解释执行，状态机、取消、暂停、恢复
├── device/
│   ├── process_device_registry.*    # 外设发现、活动设备、profile
│   ├── i_laser_device.h
│   ├── i_process_io.h
│   ├── motion/*                     # 仿真、ACS、GTN adapter
│   └── laser/*                      # 仿真、IPG、Pharos、Raycus、Analog adapter
├── settings/
│   ├── process_settings.*           # 持久化入口
│   ├── process_settings_schema.*    # 参数 schema/default/validation
│   └── process_settings_store.*     # 读写与迁移
├── ui/
│   ├── process_flow_tree_view.*     # 只负责显示、拖放、选择、编辑触发
│   ├── process_flow_model.*         # QAbstractItemModel adapter
│   ├── process_settings_dialog.*    # 左树右页统一参数窗口
│   └── node_editors/*               # 节点编辑界面
└── commands/
    └── commands_process.*           # QAction 装配与 facade 调用
```

关键原则：

- Module 不持有 QWidget，不以 UI 树作为业务事实源。
- 流程节点有稳定 id，保存/加载不依赖 row index。
- 参数界面统一树+stack，页面通过 registry 管理。
- 外设通过接口和 adapter 接入，SDK 依赖可选、可禁用、可测试。
- 旧源码只作为迁移参考或 adapter 内部实现，不继续把 `Service`、`DT`、旧全局状态扩散到新模块。
