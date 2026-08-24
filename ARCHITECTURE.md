# LaserCNC 架构说明

本文描述截至 2026-08-24、本次审计整改工作区的实际架构。源码和 `CMakeLists.txt` 是实现事实；本文是架构事实源；尚未收口的偏差记录在 [AUDIT.md](AUDIT.md) 与 [todo.md](todo.md)。历史版本文档不得覆盖当前事实。

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

模块实现和 UI 依赖已收紧为 `PRIVATE`，只把公共头实际暴露的 view/contract/Qt 依赖设为 `PUBLIC`。Process 与 Simulation 不再获取具体 `CamModule`；App 仍作为组合根持有具体模块并完成 UI 信号接线，这是顶层装配职责，业务层不得沿该路径反向调用实现。

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

`ServiceRegistry::registerBorrowedService(T&)` 显式表达模块 facade 的非拥有生命周期，并集中实现借用句柄；独立服务继续使用拥有型注册。模块停止后由 Kernel 按逆拓扑清空注册表，不再在各模块散落 no-op-deleter `shared_ptr(this)`。

## 4. 工程、文档和视图所有权

一个 `ProjectWorkspace` 拥有：

- 一个项目 `LcncDocument`，同时存储 Workpiece/Auxiliary 与 CAM-kind XCAF 实体。
- 一个 `CamDataManager`，存储稠密点、轮廓、图层、排序、工艺参数和运行时 CAM 状态。
- 一个 `LcncProjectSession`，存储名称、路径、manifest、dirty 与兼容状态。

`workpieceDocument()` 与 `camDocument()` 当前指向同一物理项目文档。机台文档由 Kernel 的 `MachineWorkspace` 独立拥有，不持久化到项目，也不应使项目变脏。

`GuiApplication` 为每个 `ProjectWorkspaceId` 拥有一个 `GuiDocument`。显示对象按 `{DocumentId, XCAF entry, domain, EntityKind}` 注册；工作区切换只恢复/切换显示，不得重建业务数据或制造 dirty。`MainWindow` 维护每工作区 `WidgetOccView`，但工作区呈现、工程树、CAD 任务面板和视图状态仍未完全从主窗口下沉。

## 5. CAD

CAD 负责工件导入导出、建模、草图、选择和 Workpiece 域写回。纯 OCC 建模位于 `core/algorithms/cad`；`CadDocumentIoService` 承接工程/STEP/IGES/STL/BREP IO 和显示网格准备。

工程包打开使用 pending workspace，成功后再采纳。普通 CAD 导入已采用两阶段事务：worker 只读取 STEP/IGES/STL/BREP、构造临时文档/shape 并生成 `CadImportPayload`，不捕获或修改活动 `LcncDocument`；返回所有者线程后重新解析 document id 并一次提交。任务由 document id 对应的 `ModuleTaskScope` 跟踪，关闭只取消相关文档任务。

```text
worker 读取文件并构造 detached shape/document
  -> 检查取消与 workspace/document generation
  -> 在文档所属线程一次提交
  -> 刷新显示和 dirty
```

失败、取消、文档已关闭或陈旧任务不得留下半提交实体。导入提交不包裹 OCC undo command：这是现有 XCAF 导入销毁稳定性的已知约束，回归测试覆盖真实 `model/半球.stp` 的 detached 读取和提交。

## 6. CAM

CAM 已按 `contracts/`、`pipeline/`、`toolpath/`、`collision/`、`machine/`、`display/`、`interaction/`、`integration/`、`ui/` 和 `internal/` 拆分。`cam_module.cpp` 主要保留生命周期、服务注册、工作区绑定和基础配置；但 `cam_module.h` 及多个分区实现仍共享同一个大型 `CamModule` 状态面。

CAM 的权威契约：

- `ContourSequenceSnapshot`：唯一轮廓加工顺序。
- `ToolpathExportSnapshot`：控制器无关、OCC-free 的已提交执行快照。
- `CollisionValidationSnapshot`：完整性、状态、环境修订和节点验证结果。
- `Retract/Traverse/Approach`：空程阶段语义及最终物理轴坐标。

刀路生成遵循“GUI 快照 → 后台 OCC/IK → revision 校验 → 原子提交”。有序求解在副本上完成，全量成功后才替换已提交坐标。切割偏置、空程偏置和工具高度只在 CAM 应用一次；Process 与 Simulation 不得重排、重求或再次叠加。

碰撞生产链为“离线固定 `.lmsp/.lmsi` 机台域（完整切割头属于 Z 轴实体） AND 工件局部 Job Overlay → Surface-BVH/Coal 极窄残差 → 连续运动边证书”。模拟锥头/喷嘴代理只用于显示，工件是唯一运行时几何变量；有效包与工件同时存在后即后台准备 Overlay。Rapid、LeadIn、Cutting 和 Traverse 每条边都必须有与包键、环境代际和端点绑定的证书；Process 不做 OCC 或在线几何，只校验证书和固定/连续点动许可证。生产连续证书的 OCCT exact 预算为 0，exact 仅由 CAM 保留作离线 CertifiedSafe 反向审计。Pending、Indeterminate、BoundaryUnknown、环境过期、证书缺失或 `complete=false` 必须阻断真实加工。碰撞后自动绕障/重规划仍见专项计划。

`MachiningFacePipelineService` 独占 entry 写入，只向调用方发布 const 视图和 revision；`CamModule` 不再长期持有可变容器引用。`CamDisplayProjectionService` 记录 AIS owning context；双工作区反复投影仍是后续回归项。

## 7. Simulation

`SimulationModule` 创建独立 `GuiDocument`/OCC 沙箱，冻结 CAM 刀路、轮廓顺序、机台运动学、显示和碰撞输入后回放。它不连接控制器、激光器或串口，不写项目/CAM，也不改变实时机台姿态。

Simulation 只通过 `ICamOfflineSimulationProvider` 在会话开始时捕获不可变 `OfflineSimulationSnapshot`。快照包含已提交执行/激光刀路、轴与机台配置、装配、机台/工件形状、碰撞和显示参数；CAM 通过 revision 与 `OfflineSimulationSourceChanged` 事件通知失效。Simulation 不再 include、查询或持有具体 `CamModule`。

## 8. Process

Process 负责执行，不负责几何。其输入是 CAM 的 OCC-free 快照和权威顺序。主要组成：

- `ProcessModule`：facade、状态/信号编排和现有服务装配。
- `ProcessRunCoordinator`、`ProcessWorkflowExecutor`：运行状态与流程执行。
- `ProcessCuttingPlanService`、`ProcessToolpathService`、`NormalCuttingManager`：准备和执行 CAM 快照。
- `ProcessDeviceRuntime`：供应商对象、typed motion/IO 操作和安全关闭顺序。
- `DeviceCommandQueue`：Stop/Workflow/Interactive/Normal/Polling 优先级队列。
- `ProcessConnectionService`、`ProcessStatusService`、`ProcessPreflightService`、手动运动和交互 IO service。

设备执行使用模块级单一 `DeviceCommandQueue`：Stop、Workflow、Interactive、Normal、Polling 形成全局优先级，`ProcessStatusService` 的控制器和外设轮询使用最低优先级，不再绕过主队列。`ProcessDeviceCoordinator` 仍是进入供应商 SDK 的递归串行租约。ACS/GTN 可轮询等待统一使用 monotonic deadline、取消条件和有限轮询间隔；供应商函数自身若永不返回，仍需厂商超时或进程外看门狗，软件队列不能抢占正在执行的外部调用。

Process 的安全边界：

- 真实控制器失败不得回退 PureSimulation。
- Stop 是唯一软件安全停机入口；失败保持 Error，复位前必须重新检查设备。
- 设备关闭先关闭激光/红光/吹气等输出，再停止运动并断开。
- GUI 不得同步等待普通设备命令；供应商对象不得跨 worker 边界泄漏。
- 机台指纹不匹配、CAM 快照不完整、碰撞 Pending/Collision/Indeterminate 都必须阻止真实加工。
- 固定运动必须持有未过期的端点许可证；连续点动按 100 ms 续签 300 ms 视界，续签失败立即排队停止。

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

版本由 CMake `project(... VERSION 1.6.0)` 单点定义并生成 `LCNC_VERSION_STRING`；`QApplication`、模块信息和工程包 fallback 使用同一值。版本交付文档必须与该值一致。

`scripts/check_architecture.ps1` 拒绝 core/view 反向依赖、pure algorithm 污染、Process OCC include、淘汰 API、settings singleton、runtime 外原始设备访问、Process/Simulation 具体跨模块头、旧 CAD 活动文档导入入口、重复 settings 目录、设备公共头命名空间污染、`M_PI` 和孤儿 `.cpp`。运行时 deadline、catch 日志、格式风格和新 `tr()` catalog 仍需测试或后续门禁补充。

2026-08-24 碰撞闭环验证：日常 ACS+GTN Debug 构建通过，完整 CTest 41/41 通过；真实 AC 转台生产档约 108 秒生成，索引 21.94 MB、包 11.45 MB，32/32 CertifiedSafe 抽样无假安全；真实半球第六轮 4,255 条连续边全部生成安全证书且几何回退为 0，Process 查询 P99 0.2 us。索引 schema 5 将全表面距离 BVH 与封闭实体包含 BVH 分离并一同持久化，旧持久网格要求重建。构建/CTest/benchmark/SimulatorCMHP 不是 GUI、长稳或实体机证据。

## 11. 后续架构顺序

1. 在现有连续碰撞证书基线上补碰撞后自动绕障/重规划、正式夹具 Overlay、扩大 exact 审计和实体机性能门限。
2. 为不可中断的外部 SDK 调用补厂商硬超时或进程外看门狗，并完成真实设备低速验证。
3. 继续把 `MainWindow` UI 接线、`ProcessModule` 编排、CAM 大状态面和超大适配器下沉为窄 controller/service。
4. 扩展双工作区、长稳、GUI 和物理机证据，同时保持自动化、SDK 仿真和实体机结论相互独立。
