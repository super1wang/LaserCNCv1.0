# LaserCNC 架构说明

本文描述截至 2026-08-21、源码提交 `47e5408` 的实际架构。源码和 `CMakeLists.txt` 是实现事实；本文是架构事实源；尚未收口的偏差记录在 [AUDIT.md](AUDIT.md) 与 [todo.md](todo.md)。历史版本文档不得覆盖当前事实。

## 1. 系统边界

LaserCNC 是 Windows 单进程、多工作区的五轴激光加工桌面应用，包含 CAD、CAM、只读离线仿真和 Process 设备执行。核心目标是：

- 项目数据、机台参考资产、视图和设备运行时分别拥有明确生命周期。
- OCC 几何止于 core/CAD/CAM/view；Process 只消费无 OCC 类型的 DTO。
- CAM 唯一生产轮廓顺序、运动计划、物理轴坐标和碰撞验证快照。
- 异步任务只处理 detached/不可变输入，并以 generation/revision 校验后提交。
- 设备调用可取消、可超时、可安全停止；不能中断的供应商调用必须保活对象并进入明确错误状态。

## 2. 分层和目标依赖

允许的源码调用方向为：

```text
app / modules/*/ui / commands
              |
              v
module facade / contract / service
              |
       +------+------+
       v             v
      view           core
                      ^
                      |
              core/algorithms
```

硬约束：

- `core/**` 不得 include `view/`、`modules/`、`app/`。
- `view/**` 不得 include `modules/`、`app/`。
- `core/algorithms/**` 不得依赖 UI、`LcncDocument`、`GuiDocument` 或 `Kernel`。
- `modules/process/**` 不得出现 OCC 类型或头文件。
- 跨模块读取应使用 facade/contract/service；通知使用 Qt signal 或 `EventBus`。

CMake 目前形成七个主要静态库：

```text
lcnc_core
  -> lcnc_view
     -> lcnc_module_cad
        -> lcnc_module_cam
           -> lcnc_module_simulation
           -> lcnc_module_process
              -> lcnc_app
                 -> LaserCNC
```

该图反映当前链接事实，不代表所有边界已收口：主模块依赖多为 `PUBLIC`，App、Simulation 和部分 Process 仍直接获取具体 `CadModule`/`CamModule`/`ProcessModule`。目标是只公开契约和必要 DTO，把实现库与 UI 依赖改为最小 `PRIVATE` 传播。

## 3. Kernel、服务和生命周期

`lcnc::Kernel` 由 `main()` 按值创建，是进程内全局入口。它拥有：

| 对象 | 职责 |
| --- | --- |
| `ServiceRegistry` | 按接口/具体类型注册服务。 |
| `EventBus` | 类型化同步事件通知。 |
| `ModuleRegistry` | 依赖拓扑、init/start/stop、失败回滚和反向停止。 |
| `AppSettings` | 应用级设置。 |
| `LcncProjectManager` | 工作区、工程包和活动工程。 |
| `MachineWorkspace` | 独立机台参考资产。 |
| `TaskManager` | QtConcurrent 任务、进度、取消和终态。 |
| `MachineConfigurationService` | 机台拓扑、轴角色和配置。 |

`GuiApplication` 由 `main()` 拥有并以非拥有指针注入 Kernel；`CommandContainer` 由 `MainWindow` 拥有。模块依赖为 `cad -> cam -> {simulation, process}`，停止顺序相反。模块启动的任务必须由 `ModuleTaskScope` 跟踪，`stop()` 先取消、有限等待，再释放借用状态；超时时不能销毁仍被 worker/SDK 使用的对象。

当前 `ServiceRegistry` 还会注册以 no-op deleter 包装模块 `this` 的 `shared_ptr`。它实际是非拥有句柄，生命周期安全依赖 ModuleRegistry 和注册表清空顺序；后续应改成显式非拥有服务句柄或只注册独立拥有的 facade/service，避免共享所有权语义失真。

## 4. 工程、文档和视图所有权

一个 `ProjectWorkspace` 拥有：

- 一个项目 `LcncDocument`，同时存储 Workpiece/Auxiliary 与 CAM-kind XCAF 实体。
- 一个 `CamDataManager`，存储稠密点、轮廓、图层、排序、工艺参数和运行时 CAM 状态。
- 一个 `LcncProjectSession`，存储名称、路径、manifest、dirty 与兼容状态。

`workpieceDocument()` 与 `camDocument()` 当前指向同一物理项目文档。机台文档由 Kernel 的 `MachineWorkspace` 独立拥有，不持久化到项目，也不应使项目变脏。

`GuiApplication` 为每个 `ProjectWorkspaceId` 拥有一个 `GuiDocument`。显示对象按 `{DocumentId, XCAF entry, domain, EntityKind}` 注册；工作区切换只恢复/切换显示，不得重建业务数据或制造 dirty。`MainWindow` 维护每工作区 `WidgetOccView`，但工作区呈现、工程树、CAD 任务面板和视图状态仍未完全从主窗口下沉。

## 5. CAD

CAD 负责工件导入导出、建模、草图、选择和 Workpiece 域写回。纯 OCC 建模位于 `core/algorithms/cad`；`CadDocumentIoService` 承接工程/STEP/IGES/STL/BREP IO 和显示网格准备。

当前工程包打开使用 pending workspace，成功后再采纳；但普通 CAD 导入仍可能由 `TaskManager` worker 捕获裸 `LcncDocument*` 并直接调用 `addShapeEntity()` 或 STEP/IGES transfer 写入活动 XCAF 文档。关闭只按模块整体取消任务，尚未形成 document-scoped task ownership。目标事务是：

```text
worker 读取文件并构造 detached shape/document
  -> 检查取消与 workspace/document generation
  -> 在文档所属线程一次提交
  -> 刷新显示和 dirty
```

失败、取消或陈旧任务不得留下半提交实体。

## 6. CAM

CAM 已按 `contracts/`、`pipeline/`、`toolpath/`、`collision/`、`machine/`、`display/`、`interaction/`、`integration/`、`ui/` 和 `internal/` 拆分。`cam_module.cpp` 主要保留生命周期、服务注册、工作区绑定和基础配置；但 `cam_module.h` 及多个分区实现仍共享同一个大型 `CamModule` 状态面。

CAM 的权威契约：

- `ContourSequenceSnapshot`：唯一轮廓加工顺序。
- `ToolpathExportSnapshot`：控制器无关、OCC-free 的已提交执行快照。
- `CollisionValidationSnapshot`：完整性、状态、环境修订和节点验证结果。
- `Retract/Traverse/Approach`：空程阶段语义及最终物理轴坐标。

刀路生成遵循“GUI 快照 → 后台 OCC/IK → revision 校验 → 原子提交”。有序求解在副本上完成，全量成功后才替换已提交坐标。切割偏置、空程偏置和工具高度只在 CAM 应用一次；Process 与 Simulation 不得重排、重求或再次叠加。

碰撞以私有几何、AABB/OBB、表面网格/BVH 和受全局 `OcctExactOperationLock` 保护的精确距离组成。Pending、Indeterminate、环境过期或 `complete=false` 必须阻断真实加工。当前仍是离散节点/稀疏证书体系，连续段保守扫掠、碰撞后重规划和完整 C1-C3 安全域见专项计划。

`MachiningFacePipelineService` 已复用等价自动面的稳定 ID，`CamDisplayProjectionService` 已记录 AIS owning context；但 service 仍公开 mutable `entries()`，`CamModule` 仍长期持有其可变引用，双工作区投影回归也未完成。

## 7. Simulation

`SimulationModule` 创建独立 `GuiDocument`/OCC 沙箱，冻结 CAM 刀路、轮廓顺序、机台运动学、显示和碰撞输入后回放。它不连接控制器、激光器或串口，不写项目/CAM，也不改变实时机台姿态。

链接层面 Simulation 只依赖 CAM 与 view；源码层面仍直接获取 `CamModule` 以读取配置、机台和场景信息，同时使用部分 CAM contracts。目标是由一个完整的 `OfflineSimulationSnapshotProvider` 提供不可变快照，彻底移除对具体 CAM 入口的依赖。

## 8. Process

Process 负责执行，不负责几何。其输入是 CAM 的 OCC-free 快照和权威顺序。主要组成：

- `ProcessModule`：facade、状态/信号编排和现有服务装配。
- `ProcessRunCoordinator`、`ProcessWorkflowExecutor`：运行状态与流程执行。
- `ProcessCuttingPlanService`、`ProcessToolpathService`、`NormalCuttingManager`：准备和执行 CAM 快照。
- `ProcessDeviceRuntime`：供应商对象、typed motion/IO 操作和安全关闭顺序。
- `DeviceCommandQueue`：Stop/Workflow/Interactive/Normal/Polling 优先级队列。
- `ProcessConnectionService`、`ProcessStatusService`、`ProcessPreflightService`、手动运动和交互 IO service。

实际设备执行拓扑不是单一队列：主运行队列之外，`ProcessStatusService` 还拥有控制器状态和外设状态两个独立 `DeviceCommandQueue`。三者最终通过同一个 `ProcessDeviceCoordinator` 递归互斥租约串行进入 SDK。这样可避免轮询长期排在工作流之后，但 Stop 优先级只在主队列内部成立，无法抢占另一个队列中已经持有租约的供应商调用。因此所有 SDK 调用必须有真实 deadline/abort；当前 ACS/GTN 尚有无超时等待环，是发布阻断项。

Process 的安全边界：

- 真实控制器失败不得回退 PureSimulation。
- Stop 是唯一软件安全停机入口；失败保持 Error，复位前必须重新检查设备。
- 设备关闭先关闭激光/红光/吹气等输出，再停止运动并断开。
- GUI 不得同步等待普通设备命令；供应商对象不得跨 worker 边界泄漏。
- 机台指纹不匹配、CAM 快照不完整、碰撞 Pending/Collision/Indeterminate 都必须阻止真实加工。

## 9. 工程包和迁移

桌面应用只读取 format v4，并要求：

```text
project.toml
workpiece.xbf
cam_toolpath.toml
cam_toolpath_points.bin
tools.toml
```

保存先构造 staging archive，再使用 Windows 原子替换；失败保留旧包。项目工具从包内 `tools.toml` 恢复。机台指纹不匹配允许查看和 PureSimulation，但禁止真实加工。

v1/v2/v3 不由桌面应用或普通库路径兼容。`src/tools/lcnc_project_upgrade.cpp` 构建独立 `lcnc_project_upgrade.exe`，它是唯一允许调用迁移入口的受控离线工具；输入输出必须不同，缺少嵌入工具快照的历史包必须显式提供 `--tools`。

## 10. 构建、版本和验证

唯一构建约定见 [BUILD.md](BUILD.md)。`build-cmake/` 与 `build-vs/` 不能共享生成树或并发构建；运行输出按生成器/变体隔离。

当前存在三套版本语义：CMake `project(... VERSION 1.2.1)`、`QApplication` 的 `1.0.0` 和文档交付记录 `v1.5.9`。在建立单一版本源前，不应把任一值单独声明为完整产品版本；`v1.5.9` 仅表示本轮文档基线。

`scripts/check_architecture.ps1` 当前拒绝 core/view 反向依赖、pure algorithm 污染、Process OCC include、淘汰 API、settings singleton、runtime 外原始设备访问和孤儿 `.cpp`。它尚不能证明 CAD worker 事务、具体 Module 耦合、供应商调用有界、catch 日志、格式风格或新 `tr()` 已进入 catalog。

2026-08-21 审计验证：日常 ACS+GTN Debug 构建通过，35/35 CTest 通过；real-laser Debug 变体可链接，但暴露真实激光源的未初始化使用、非全路径返回和大量 `/W4` 告警。构建/CTest/SimulatorCMHP 不是 GUI、长稳或实体机证据。

## 11. 后续架构顺序

1. 先关闭真实激光未定义行为、供应商无界等待、CAD worker 活动文档写入和连续碰撞发布门禁。
2. 将所有设备调用收敛为可证明有界的设备 actor/命令模型，确保 Stop 不被轮询或长调用无限阻塞。
3. 让 App、Simulation、Process 和 module UI 只依赖 facade/immutable snapshot contract，停止注册具体 Module 公共服务。
4. 继续拆分 `MainWindow`、`ProcessModule`、`CadModule`、`SimulationModule`、CAM 大状态面和超大算法文件。
5. 建立单一版本源、CMake helper、格式化/lint 和架构门禁，防止结构债务重新增长。
