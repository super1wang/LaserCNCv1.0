# Process 模块框架

审阅日期：2026-05-26

本文记录当前 `src/modules/process` 的真实完成状态、现有框架边界、已接入能力、遗留导入代码和下一阶段重构约束。后续实现以本文、`代码规范.md`、`微内核框架结构.md` 和 `process模块重构计划.md` 为准。

## 0. 2026-05-26 阶段 1-8 实施后状态

本轮已把 Process 模块推进到可运行的新框架闭环。若后文历史审阅缺口与本节冲突，以本节和 `实施进度.md` 为准。

- `ProcessModule` 已彻底移除旧 `ProcessTreeView*` 反向依赖，新建、加载、保存和运行只以 `ProcessFlowDocument` / `ProcessFlowStore` 为事实源。
- `ui/process_flow_model.*`、`ui/process_flow_tree_view.*` 已承接左侧执行页，支持 stable id、拖放、右键增删启禁、清空、保存/加载、双击编辑和运行状态刷新。
- `workflow/process_node_registry.*` 统一管理全部当前节点的 metadata、默认参数、摘要、可放置规则和执行 key；流程树新增菜单、Info 列和拖放规则均从 registry 取数。
- `ui/process_node_edit_dialog.*` 已支持 Wait、Axis 以及 AxesMove、Feeding、Cutting、OverCutting、EnergySwitch、IO、Camera、Measurement、MarkAcquire、Alignment、AutoFocus、Monitor、Commands、Loop、RunGroup、If、Compare、Calculation 等节点的 typed 参数编辑；未知参数仍可通过通用参数表兜底。
- `Setting/process_settings_dialog.*` 已改为左侧树 + 右侧 `QStackedWidget`，页面覆盖 Process、Motion Controller、Laser、Axis、Tool、IO、Gas、Water、Monitor、LoadingPos、Camera、Internet 的基础字段。
- `ProcessSettings` 已扩展对应基础参数并持久化到 `process.toml`；复杂旧 Setting 表、设备列表页和旧权限逻辑仍待逐项迁移。
- `device/process_device_manager.*` 已拥有仿真激光 `SimulatorLaserDevice` 和仿真 IO `SimulatorProcessIo`，真实 ACS/GTN/BDAQ/激光 adapter 通过 SDK-off CMake option 保持默认不可用。
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
            ├─ IMotionController service: SimulationMotionController
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
│                                          # 新仿真运动控制器，注册为 IMotionController
├── device/
│   ├── process_device_manager.*          # 新设备目录/活动设备管理器，当前只暴露仿真设备
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
│   ├── process_settings_dialog.*         # 当前编译的左树右页参数对话框
│   └── qg_/Setting_*                     # 旧参数界面源码，已导入但多数未纳入 CMake
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
- `ProcessModule` 注册 `SimulationMotionController` 为 `lcnc::IMotionController`。
- `MainWindow` 在左侧创建“执行”页，并把 `QG_ProcessesWidget::GetTreeView()` 临时传给 `ProcessModule`。
- `MainWindow` 右侧已有 `WidgetLaserControl`，用于轴位置、点动、运行/暂停/停止、急停、倍率和状态显示。

当前不足：

- `ProcessModule` 仍持有 `ProcessTreeView*`，这是 UI 反向进入 module 的临时桥接，不符合长期边界。
- 流程数据仍以 UI 树为事实源，尚未形成独立的 `ProcessFlowDocument` 或 workflow service。
- 运行命令只驱动状态机和仿真定时器，尚未解释执行流程节点。

### 3.2 Facade 与命令

已完成：

- `IProcessFacade` 暴露连接、断开、仿真模式、运行、暂停、停止、急停、复位、回零、新建流程、加载流程、保存流程和状态文本。
- Ribbon 已注册流程命令：新建流程、加载流程、保存流程。
- Ribbon 已注册参数命令：加工设置、运动参数、激光参数。
- Ribbon 已注册运行/安全命令：运行、暂停、停止、回零、急停、复位急停、连接、断开、仿真模式。
- `ProcessModule::newProcess/loadProcess/saveProcess` 已开始通过 `ProcessFlowDocument` + `ProcessFlowStore` 读写流程数据；旧 `ProcessTreeView` 仅作为过渡 UI 同步。

当前不足：

- 命令层仍直接创建 `QFileDialog`、`QInputDialog` 和 `ProcessSettingsDialog`，这在现阶段可接受，但后续可考虑通过 app 对话服务统一。
- `CmdToggleSimulationMode` 构造时连接 facade 信号，若命令早于 service 完整初始化，需要确认生命周期稳定性。
- 参数命令目前打开同一个轻量 tab 对话框的不同初始页，尚未达到旧程序“左树右页”的统一界面要求。

### 3.3 流程树

已完成：

- 左侧“执行”tab 已显示 `QG_ProcessesWidget`。
- 当前 `ProcessTreeView` 为 Qt6 友好的轻量实现，支持右键添加、删除、启用、禁用、清空、保存、加载。
- 当前可添加节点：`Start`、`Stop`、`Wait`、`Axis`、`Group`、`If`、`Loop`。
- `SaveValue()` / `LoadValue()` 使用 TOML，结构为 `Process.items`，可保存类型、状态、标签、信息和子节点。
- 新 `ProcessFlowStore` 输出 `Process.schemaVersion` 和 `Process.nodes`，同时保留 `Process.items` 兼容旧流程树。
- `GetTreeItemVector()` 可导出 `std::vector<Item>`，用于后续执行引擎适配。

当前不足：

- 旧程序已有节点远多于当前可用节点，尚未完整迁移。
- `QTreeView` 已设置 `InternalMove`，但 `TreeModel` 尚未实现 `flags()`、`supportedDropActions()`、`mimeData()`、`dropMimeData()` 或 `moveRows()`，实际完整拖拽移动能力还未完成。
- `mouseDoubleClickEvent()` 目前只记录当前 index 并调用基类，尚未打开节点编辑界面。
- 当前节点参数仍是 `map<QString, QString>` 兼容结构，没有类型化参数对象、校验、默认值和版本迁移。
- 当前流程树模型直接复用旧 `TreeItem`，析构中存在重复 `qDeleteAll(m_childItems)` 的历史问题，后续应在重构时清理。
- `collectItems()` 当前递归时使用子节点 row 作为下一层 parent index，不能稳定表达全树节点身份；后续必须改为 stable id。

### 3.4 参数界面

已完成：

- 新增轻量 `ProcessSettingsDialog`，当前包含 `加工`、`运动`、`激光` 三个 tab。
- 新增 `ProcessSettings`，持久化到 `<exeDir>/config/process.toml`。
- 当前字段：`controllerEndpoint`、`simulationMode`、`motionController`、`laserDevice`、`laserEnergy`、`laserFrequency`、`laserPulseWidth`。
- 参数保存后会调用 `ProcessModule::reloadDeviceSettings()` 同步仿真模式和活动设备。

已导入但未接入的新旧界面资源：

- 旧统一参数窗口 `QG_dlgSetting` 使用左侧 `QTreeWidget` + 右侧 `QStackedWidget`，符合用户期望的交互形态。
- 旧参数页包括 MotionControl、Axis、IOIndex、Digital、Analog、Laser、Internet、Tool、Gas、Water、Monitor、LoadingPos、Camera 等。
- 旧 `Setting.ui` 也包含更大的统一参数界面结构，但当前没有进入 CMake。

当前不足：

- 当前轻量对话框是 tab 结构，不符合最终“左侧树形节点 + 右侧参数界面”的要求。
- 旧参数页依赖 `Service`、`DT`、旧 `Settings`、全局权限和旧设备表，不能直接编译进新模块。
- 当前参数数据只覆盖少量 process/motion/laser 字段，尚未覆盖轴、IO、气、水、相机、上料位、工具和切割工艺参数。

### 3.5 外设和仿真

已完成：

- `SimulationMotionController` 是新框架中的纯仿真运动控制器，通过 `MachinePose` 更新姿态。
- `ProcessDeviceManager` 当前作为第一阶段设备目录，暴露：
  - 运动控制器：`SimulatorCMHP`
  - 激光器：`Simulator`
- `ProcessDeviceManager::syncFromSettings()` 能从 `ProcessSettings` 同步活动设备，不合法时回退到仿真设备。

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
| 起止与结构 | Start、Stop、Group、RunGroup、RunGroupCheck、If、Loop、Wait | Start/Stop/Group/If/Loop/Wait 可添加；RunGroup/RunGroupCheck 未接入 |
| 运动 | Axis、AxesMove、Feeding、AutoFocus、LoadingPos 相关流程 | Axis 可添加；其余未接入执行和编辑 |
| IO/外设 | IO、Commands、EnergySwitch、Monitor、Camera | 未接入完整编辑和执行 |
| 工艺 | Cutting、OverCutting | 未接入，属于切割流程迁移重点 |
| 视觉/测量 | Measurement、MarkAcquire、Alignment、Calculation、Compare | 未接入，依赖视觉和几何上下文重构 |

后续节点迁移不能简单把旧 `Process_*.cpp` 全部加入 CMake。应先建立新节点定义、参数 schema、编辑器接口和执行器接口，再逐个迁移旧 UI 与旧执行逻辑。

## 5. 架构合规审阅

### 当前符合规范的方向

- Process 已通过 `IModule` 和 `IProcessFacade` 接入微内核。
- 命令大多通过 `IProcessFacade` 调用模块能力。
- 旧外设未直接加入 CMake，避免把 ACS/GTN/bdaq/旧 Service 依赖一次性引入。
- 新仿真控制器作为 `IMotionController` service 暴露给其他模块。
- 参数持久化开始收敛到 `ProcessSettings` 和 TOML。

### 当前需要重构的边界问题

- `ProcessModule` 持有 `ProcessTreeView*`，module 依赖 UI widget，后续必须替换为 module/service 持有流程数据，UI 只绑定模型。
- `Process_TreeView` 同时负责 UI、序列化和部分数据结构，后续应拆分为 workflow document、store、model、view 和 editor。
- 旧 `TreeItem` 使用公共成员、全局 `using std::map`、旧命名和 UI/业务混合，后续应隔离或替换。
- 旧 Setting 页依赖 `Service*` 和全局 `DT`，需要转换为 settings service + page presenter，而不是继续扩散旧全局入口。
- 真实外设 SDK 不能硬编码个人路径；必须通过 CMake cache option、toolchain 或独立 adapter 目标接入。

## 6. 当前完成度评估

| 子域 | 完成度 | 说明 |
| --- | --- | --- |
| 微内核模块接入 | 70% | 模块、facade、service 已有；UI 指针桥接待去除 |
| Ribbon 命令 | 65% | 运行/流程/参数命令已接入；对话服务和错误反馈待完善 |
| 执行页 UI | 55% | 左流程树、右控制面板已可见；流程树功能和右侧状态联动需增强 |
| 流程树保存/加载 | 45% | TOML 基础可用；稳定 id、schema、节点参数和版本迁移待补 |
| 节点迁移 | 20% | 只迁移轻量基础节点；切割、视觉、IO、测量等未完成 |
| 参数界面 | 20% | 轻量 tab 可用；最终统一树+页界面未完成 |
| 外设迁移 | 15% | 只有仿真目录和仿真运动服务；真实设备未接入 |
| 流程执行引擎 | 10% | 旧引擎未迁移，新引擎尚未建立 |
| 构建稳定性 | 65% | Debug 构建曾验证通过；旧导入源码未全部纳入构建 |

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
