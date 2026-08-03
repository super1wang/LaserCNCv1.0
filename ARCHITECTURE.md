# LaserCNC 架构说明

本文描述 2026-07-30 源码基线的实际架构。若本文与阶段性设计记录冲突，以源码、`CMakeLists.txt` 和本文为准。

## 1. 系统定位

LaserCNC 是 Windows 单进程桌面应用，将 CAD、CAM 与五轴激光加工执行整合在同一工作区。技术栈为 C++17、Qt 6、OpenCASCADE/XCAF、SARibbon、toml11、spdlog 与 QuaZip。

核心设计目标：

- 数据所有权集中在 `core`，业务模块只借用数据并执行领域操作。
- OCC 几何止于 CAD/CAM/view 边界，Process 只消费无 OCC 类型的 DTO。
- 每个打开工程拥有独立数据和视图生命周期；机台模型是跨工程的独立参考资产。
- 异步任务、硬件 IO 和 GUI 对象必须有明确的停止、等待与反向析构顺序。

## 2. 分层与依赖

允许的主要依赖方向为：

```text
app / modules/*/ui / commands
            |
            v
modules facade / services
            |
            v
view --------+--------> core
                         ^
core/algorithms ----------+
```

硬约束：

- `core/**` 不得包含 `view/`、`modules/`、`app/`。
- `view/**` 不得包含 `modules/`、`app/`。
- `core/algorithms/**` 只使用 OCC、数学类型和轻量参数，不依赖 UI、文档所有者或 `Kernel`。
- `src/modules/process/**` 不得出现 `TopoDS_*`、`AIS_*`、`gp_*`、`Geom_*`、`BRep*`、`XCAF*`。
- 跨模块同步读取使用接口或服务；状态通知使用 Qt signal 或 `EventBus`。

CMake 将源码拆为 `lcnc_core`、`lcnc_view`、`lcnc_module_cad`、`lcnc_module_cam`、`lcnc_module_process`、`lcnc_app` 六个静态库，最终链接到 `LaserCNC`。

## 3. Kernel 与生命周期

`lcnc::Kernel` 是进程内唯一入口，由 `main()` 按值创建。它拥有：

| 对象 | 所有权与职责 |
| --- | --- |
| `ServiceRegistry` | 以接口类型注册服务；模块停止后清空。 |
| `EventBus` | 类型化同步事件总线。 |
| `ModuleRegistry` | 模块拓扑排序、`init/start/stop` 与反向停止。 |
| `AppSettings` | 应用级配置。 |
| `LcncProjectManager` | 多工作区及工程数据生命周期。 |
| `MachineWorkspace` | 跨工程常驻的机台参考资产。 |
| `TaskManager` | `QtConcurrent` 后台任务、进度和协作式取消。 |
| `MachineConfigurationService` | 机台轴与硬件配置事实源。 |

`GuiApplication` 由 `main()` 拥有后以非拥有指针注入 Kernel，`CommandContainer` 由 `MainWindow` 拥有并注入。正常关闭顺序为：

```text
停止业务模块
  -> 等待后台任务和硬件轮询退出
  -> 销毁 MainWindow
  -> 销毁 GuiApplication / GuiDocument
  -> 销毁 Kernel 数据服务
  -> 最后关闭 Logger 和 QApplication
```

模块依赖固定为 `cad -> cam -> process`。配置禁用上游模块时，下游模块不得启动。

## 4. 工程、文档与数据域

当前并非“三份独立工程文档”。一个 `ProjectWorkspace` 实际拥有：

- 一个统一的项目 `LcncDocument`：同时保存 Workpiece 与 CAM 的 XCAF 实体。
- 一个 `CamDataManager`：保存稠密刀路点、轮廓、图层、排序、引线、生成参数和运行时 CAM 状态。
- 一个 `LcncProjectSession`：保存工程名、路径、manifest、保存选项与 dirty flags。

逻辑数据域与物理存储关系如下：

| 逻辑域 | 物理所有者 | 说明 |
| --- | --- | --- |
| Workpiece | `ProjectWorkspace::m_projectDocument` | 源几何及 CAD 建模结果，实体标记为 Workpiece/Auxiliary。 |
| CAM 稀疏几何 | 同一 `m_projectDocument` | 轮廓 wire 以 CAM `EntityKind` 存入 XCAF。 |
| CAM 稠密数据 | `ProjectWorkspace::m_camData` | 不放入 OCC 文档，使用稳定 `ContourId` 与 `xcafEntry` 关联。 |
| Machine | Kernel 拥有的 `MachineWorkspace` | 独立参考资产，不写入工程包，不应使工程变脏。 |

`workpieceDocument()` 与 `camDocument()` 当前返回同一项目文档；这是统一存储的兼容 API，不代表两个独立实例。机台文档只以非拥有引用登记到 `LcncProjectManager`，用于域路由。

`LcncProjectManager` 维护多个 `ProjectWorkspace`。单文档模式打开新文件时替换全部旧工作区；多文档模式允许并存并切换活动工作区。

## 5. View 架构

`GuiApplication` 为每个 `ProjectWorkspaceId` 创建并拥有一个 `GuiDocument`。`MainWindow` 为工作区创建对应 `WidgetOccView` 并在活动工作区切换时切换视图。

读取当前视图只能使用 `activeGuiDocument()`；涉及工作区身份的切换通知使用 `activeWorkspaceDocumentChanged(ProjectWorkspaceId, GuiDocument*)`。`GuiDocument::document()` 是其绑定文档的唯一访问器，显示实体必须继续显式携带 domain 与 `EntityKind`。

每个 `GuiDocument` 包含：

- `GraphicsScene`、`AIS_InteractiveContext`、`V3d_Viewer/V3d_View`。
- `RenderingManager` 与渲染配置。
- 以 `{DocumentId, XCAF entry}` 为身份的 AIS 显示注册表。
- CAM 轮廓、辅助对象、机台和工件的逻辑域/实体类型标记。

域刷新必须是差量操作。纯工作区切换只恢复显示，不得把显示恢复误报成数据修改。`fitAll` 优先可见工件，其次可见 CAM，辅助轴线和坐标系不参与包围盒。

## 6. CAD 模块

`CadModule` 负责 Workpiece 域的导入、导出、建模、草图、特征、变换、删除和选择路由。复杂 OCC 运算应位于 `core/algorithms/cad`，文档写回和刷新由模块/service 协调。

异步导入通过 `TaskManager` 在 detached workspace 或明确的目标文档上计算，完成后再由主线程采纳结果；后台任务不得直接持有生命周期不受保护的 UI 对象。

## 7. CAM 模块

`CamModule` 借用项目的统一 XCAF 文档、`CamDataManager`、机台工作区和机台配置服务，负责轮廓提取、离散、刀路求解、图层、排序、引线与渲染协调。

业务边界为：

```text
生成：提取轮廓 + 离散点集
  -> 标记 CAM 数据 dirty
  -> 用户显式重新计算/排序
  -> 五轴刀路求解与顺序应用
```

Process 不读取 OCC。CAM 通过 `ICamToolpathProvider` 输出 `ToolpathExportSnapshot`，其中只包含控制器无关的点、姿态、轮廓和工艺引用。

## 8. Process 模块

当前 `ProcessModule` 仍是较大的 facade/coordinator，实际拥有或协调：

- typed settings 与参数/IO 模型。
- `ProcessDeviceRuntime` 私有持有运动控制器、激光器和工具表；`DeviceCommandQueue` 承接分优先级调度，`ProcessDeviceCoordinator` 租约仅作为 runtime 内部的供应商 SDK 防御串行边界。
- `ProcessCuttingPlanService`、`NormalCuttingManager` 与 motion sink。
- 流程文档、步骤插件注册表和 `ProcessWorkflowExecutor`。
- `ProcessConnectionService` 的连接/断开事务、`ProcessPreflightService` 的不可变硬件预检报告、`ProcessStatusService` 的控制器/外设轮询与安全监控启停，以及 `ProcessRunCoordinator` 的运行状态迁移；workflow/UI facade 组合仍待继续下沉。

执行链路为：

```text
CAM ToolpathExportSnapshot
  -> ProcessToolpathService / ProcessCuttingPlanService
  -> NormalCuttingManager
  -> ProcessDeviceRuntime typed sink/轮廓边界门禁
  -> MotionSinkFactory
  -> PureSimulation / ACS text / GTN buffered sink
  -> 运动控制器与激光/IO
```

`runStart()` 是加工硬门禁：流程、CAM dirty 状态、图层/工具映射、控制器/激光器、轴、IO 与监控条件必须正常才允许进入加工。任何 Error、EmergencyStop 或停止路径必须关闭激光与吹气等安全输出。

硬件 SDK 类型只能存在于私有实现。`ProcessDeviceRuntime` 持有当前控制器和激光器，所有硬件操作由设备队列线程调用，禁止函数内 static 控制器。设备队列提供 `Stop > Workflow > Interactive > Normal > Polling`、同级 FIFO、同 key 合并和可追踪 completion；每个被接受的命令必须恰好完成一次。等待超时只完成等待方并阻止继续发送普通命令，不强制中断正在运行的供应商调用。

业务路径不再取得控制器、激光器或设备锁；预检通过 `ProcessPreflightService` 提交 generation 化请求，普通切割在每个轮廓下发前通过 runtime 再次检查连接、故障、电机创建和轴使能。架构门禁拒绝 runtime 外重新调用原始设备访问 API。`ProcessDeviceCoordinator` 暂留在 runtime 内部，不能作为跨模块入口。设备公共接口不得泄漏供应商类型；兼容 `MessageModule`、设备日志宏和旧 `LogModule` 均已删除，统一使用 `lcnc::Logger`。

连接、断开、控制器状态和外设状态也不再由 `ProcessModule` 直接进入 runtime：`ProcessConnectionService` 只提交 typed 连接事务并把进度/完成回调投递回 GUI 线程；`ProcessStatusService` 独占 150 ms/2 s 定时调度、同 key 合并和防堆积，随后把不可变状态快照交给 facade 发射既有 Qt 信号。500 ms 安全 IO 的 `ProcessMonitorService` 生命周期由 status service 统一编排。
Process 模块读取监控、轮询和面板 IO 配置只经其注入的 `ProcessSettingsService`，不得回退到 `current()` 全局查询。

构建时 ACS、GTN、BDAQ 与真实激光均由 CMake 开关控制。GTN adapter 仅在 `LCNC_WITH_GTN=ON` 时参与构建；ACS adapter、`SimulatorCMHP`、文本 sink、供应商头、import library 和随程序部署的 `Simulator.prg` 也只在 `LCNC_WITH_ACS=ON` 时形成完整 ACS 路径。all-off 构建中的本地状态控制器仅服务显式 PureSimulation，不代表 ACS Simulator。支持的验证预设见 `CMakePresets.json`。

## 自动化架构门禁

`scripts/check_architecture.ps1` 是 CTest 的 `architecture_checks`。它拒绝 core/view 反向依赖、纯算法依赖 UI/文档/Kernel、Process 对 OCC 的 include、设备公共头泄漏兼容日志、Process settings singleton、runtime 外原始设备访问、已淘汰文档 API，以及未纳入 `CMakeLists.txt` 的 `.cpp`。标准验证命令为 `ctest --test-dir build-cmake --build-config Debug --output-on-failure`。构建目录和应用输出约定以 `BUILD.md` 为准。

## 9. 异步与内存安全规则

- C++ 所有权优先使用 `unique_ptr/shared_ptr`；OCC 使用 `Handle`；QObject 使用父子树。
- 非拥有裸指针必须由更长生命周期对象保证，并在异步边界前转换为快照、受控句柄或在关闭时等待。
- `TaskManager` 析构会先请求取消，再等待全部 worker 退出，最后释放任务实体。
- CAD、CAM 与 Process 模块均对其 `TaskManager` 任务持有 task-id；模块停止时先请求取消并有界等待。Process 的 worker 持有 `shared_ptr<Service>`，超时降级时不会释放仍可能在供应商 SDK 调用中的对象；连接、断开和回零在同一设备会话中互斥。
- Process 固定关闭顺序为：停止接收普通命令，取消并等待工作流/后台任务，停止监控，提交 Stop 优先级安全输出，断开设备，最后关闭执行线程。若活动供应商调用超时，则保活 SDK 对象并进入 Error，不得在调用仍活跃时释放。
- 控制器坐标、轴使能和 IO 状态由 150 ms 定时器调度到独立单线程池；安全环境监控使用另一条 500 ms 单线程池。激光器等串口外设按 2 s 低频调度到外设单线程池，`QSerialPort` 本身归属独立 IO 线程，任何等待串口响应的调用都不得在 GUI 线程执行。
- Process 硬件轮询和环境监控在停止时等待当前 future 完成，控制器和外设对象不得先于 worker 销毁。
- Qt GUI 只能在主线程访问；供应商 SDK 是否线程安全不能假定，硬件调用最终应串行化到单一设备执行上下文。
- 所有 catch 必须记录错误；析构和停止路径不得向外抛异常。
- `ModuleRegistry` 对 init/start/stop 的标准和未知异常均建立边界；失败模块和所有已初始化模块以反向顺序调用幂等 `stop()`。`main()` 是最后一道异常边界，并在 Logger 仍存活时记录错误。
- 内存验证使用 `CMakePresets.json:asan`；`scripts/collect_runtime_baseline.ps1` 只终止自己启动的进程并记录工作集、私有内存、句柄和线程，`scripts/application_verifier.ps1` 必须显式启用/禁用。

静态审计和一次成功构建不能证明“绝无泄漏”。发布门槛必须包含 ASan/Application Verifier、长时间开关工程/连接设备/仿真循环和退出压力测试。

## 10. `.lcnc` 工程包

工程包由 QuaZip 在 staging 目录中事务式生成，当前格式版本为 4：

```text
project.toml
workpiece.xbf
cam_toolpath.toml
cam_toolpath_points.bin
tools.toml
```

- `workpiece.xbf` 同时含 Workpiece/Auxiliary 与 CAM 类型实体。
- `cam_toolpath.toml` 保存图层、轮廓、签名、引线、排序和生成参数等元数据。
- `cam_toolpath_points.bin` 保存稠密采样点。
- `tools.toml` 保存项目使用的工具参数快照；加载时优先于全局工具配置恢复。
- manifest 保存软件版本、机台构型 SHA-256 指纹、配置 schema 与刀路算法版本。
- 打开工程时 `LcncProjectManager` 比较 manifest 与当前机台指纹并保存 session 兼容性；MainWindow 显示警告，Process 只在真实加工前拒绝不匹配工程，仿真不受阻断。
- 机台模型不进入工程包。
- 归档保存先在目标目录创建 staging zip，成功后用 Windows 原子替换提交；失败不会删除原 `.lcnc`。
- `lcnc_project_package_test` 以独立临时目录验证 v4 写入/读取工具快照、缺快照拒绝、失败保存后原包 SHA-256 不变，并验证 v1/v2/v3 manifest 被明确拒绝。staging `QTemporaryFile` 必须在 QuaZip 创建归档前析构，以避免 `MoveFileExW` 的 Win32=32 共享冲突。
- 应用和库只接受 v4，且 core 校验 `tools.toml` 必须存在；不存在离线升级器、迁移 API 或旧 `process_cutting_plan.toml` 回退。历史项目必须由外部受控迁移流程处理，不得在产品内猜测或修复。

## 11. 配置与可选硬件

Process 运行时服务、`MotionControl`、`LaserDevice`、`LDFactory` 和 `ProcessParameterRegistry` 通过构造函数接收其所属的 `ProcessSettingsService`；`SimulatorCMHP`、ACS、GTN 与全部激光适配器均经此路径创建。流程步骤注册表在模块初始化注入设置、停止时清空，编辑器只读取该受控引用。`ProcessSettingsService::current()` 已删除。设置必须先完成初始化，才允许创建设备、工具与 IO 服务。

`SimulatorCMHP` 归属 ACS：启用 ACS 时使用 `acsc_OpenCommSimulator`，从可执行目录加载受 CMake 管理的 `Simulator.prg`，再完成 ACS 初始化。ACS 关闭时只保留名为 `Simulator` 的 SDK-free 本地状态实现供显式 PureSimulation 使用。启用任意 ACS/GTN 后 PureSimulation 默认关闭；实体控制器或 `SimulatorCMHP` 被选择但连接失败时，状态进入 Error 并提示，执行工厂不得回退到 `PureSimulationSink`。

`ToolFactory` 查询无副作用：缺失工具不得创建空项，重载相同索引时必须替换旧参数。

`ProcessRuntimeConfiguration` 由 `ProcessModule` 拥有，并借用给 `Service` 和运动控制器；它是 ACS/GTN 标准轴、扩展轴和仿真选择的运行时事实源。配置会归一化并去重轴名，且拒绝把 `BASE` 伪轴下发给设备层；其行为由独立 CTest 覆盖。旧 `DT` 静态运行时状态已删除，`data_type.h` 仅保留共享枚举、数据结构和数值常量。

Process 对外仅保留异步全设备连接/断开；旧同步单控制器接口与 `ProcessLayerJob::order` 兼容字段已删除。

设备关闭统一由 `Service::shutdownDevices()` 执行：先停止激光和红光，再断开激光，随后停止运动/缓冲并断开控制器。

`runtime/process_axis_utilities` 承载回零顺序、轴定义比较和仿真轴坐标等纯 Process 轴逻辑，不访问 UI 或设备 SDK。

Qt SerialPort 是当前 `LaserDevice -> SerialPort` 继承链的必需依赖。ACS、GTN、BDAQ 和真实激光器由 CMake 选项控制；可选源文件和供应商库必须在同一个开关下成对加入，禁止“选项关闭但实现仍无条件编译/链接”。

个人 SDK 路径只允许作为 CMake cache 默认值，长期应迁移到 `CMakePresets.json` 或本机 preset。每种硬件组合必须有独立配置/编译验证。

## 12. 维护准则

- 架构事实只维护在本文；阶段计划只维护在 `todo.md`。
- 不保留未接入运行时、仅用于演示的平行框架。
- 兼容读取必须有版本、测试样本、删除条件和截止版本；禁止无限期双写。
- 单文件持续超过约 1000 行时，应按业务服务拆分，禁止继续扩张 facade。
- 每次提交至少执行分层扫描、过时 API 扫描、Process OCC 扫描、`git diff --check` 和 Debug 构建。
- `lcnc_task_manager_test` 覆盖 TaskManager 的协作取消、有限超时与异常边界；任务终止与取消语义改动必须保持该回归通过。
- `asan` preset 的所有可执行目标自动部署 `clang_rt.asan_dynamic-x86_64.dll` 与 OCCT 所需 TBB runtime；CTest 不得依赖开发机 PATH 或手工 DLL 拷贝。
- `collect_runtime_baseline.ps1` 即使被测进程提前退出也会输出已采集 CSV，并可选用私有工作集/句柄增长阈值作为门禁；启动初始化样本必须与稳定段分开评估。
