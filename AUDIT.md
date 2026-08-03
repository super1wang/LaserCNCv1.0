# LaserCNC 文件级代码与架构审计

审计日期：2026-08-03

审计基线：`4bec435c13c5bfc84541f1c6f80f408f5dccb451..61c9535`（今日 23 个提交）

工作区状态：仅有未跟踪的 `ctest.log`，未纳入审计改动

## 结论

今日提交完成了有价值的入口下沉：Process typed runtime/预检/连接/状态/流程/手动操作，CAD 文档 IO，CAM 加工面状态与显示投影，以及 MainWindow 的 TaskPanel/工程树控制器均已形成独立文件和测试骨架。分层、Process OCC-free、CMake 源文件收录、`/W4 /WX` quality 构建和 23/23 CTest 当前均通过。

审阅后已优先修复全部 2 组 P0 与 4 组可局部关闭的 P1 缺口，并为队列 completion、状态跨代和自动面稳定 ID 添加回归。CAD detached 事务、CAM immutable/multi-workspace 回归和入口 facade 化仍未完成；当前仍不能作为物理加工安全发布候选。

## 1. 审计范围与方法

### 1.1 今日提交

逐文件审阅今日 23 个提交涉及的 92 个文件，按以下边界核对：

| 文件组 | 审阅重点 | 结论 |
| --- | --- | --- |
| `core/task/module_task_scope.*`、TaskManager 测试 | task-id 所有权、取消、有限等待 | 模块停止路径可用；尚未覆盖文档级关闭。 |
| `modules/process/runtime/*`、`workflow/*`、`process_module.*`、普通切割 | SDK 线程、队列 completion、Stop/E-stop、预检 generation、轮询生命周期 | Stop completion、E-stop stop-only、IO 锁存和轮询 generation 已修；完整失败注入仍待补。 |
| `modules/cad/services/cad_document_io_service.*`、`cad_module.*` | 导入事务、文档寿命、OCC 异常、工作区身份 | 已在关闭前取消模块任务、拒绝非活动文档保存、STL/BREP 的提交点前取消；裸指针与 detached import 仍待修。 |
| `modules/cam/services/*`、`cam_module.*`、CAM 命令 | 加工面 ID/revision、陈旧提交、AIS 所有权 | stale-result、自动面稳定 ID 与 document-owned context 已修；缺少 immutable state 和双工作区回归。 |
| `app/controllers/*`、`main_window.*`、工程树 projection | 具体 Module 依赖、选择/显隐/排序语义 | 控制器迁移有效，但入口与 concrete Module 耦合仍大量存在。 |
| `CMakeLists.txt`、架构脚本、测试、翻译和版本文档 | 收录、门禁真实性、测试覆盖与事实一致性 | 构建/CTest 绿色；门禁尚未覆盖本次发现的关键竞态。 |

### 1.2 全项目架构

同时扫描全部 `src/`、`tests/`、CMake 与维护脚本：

- `core -> view/modules/app`、`view -> modules/app` 反向 include：0。
- Process OCC 类型/include：0。
- 淘汰文档 API与 runtime 外 `motionControl()/laserDevice()/lockDeviceAccess()` 调用：0。
- 旧迁移入口 `loadForMigration()`、`lcnc_project_upgrade`：0。
- 今日入口规模：`MainWindow` 2414 行、`CadModule` 2020 行、`CamModule` 6444 行、`ProcessModule` 2122 行。行数不作为完成标准，但业务密度仍表明四个入口均未完成 facade 化。

未在问题表中单列的文件未发现可静态证明的独立缺陷；这不等于证明不存在运行时缺陷。

## 2. 代码审阅发现

### 已修复 P0：加工安全事务

| 编号 | 文件/位置 | 问题与影响 | 必须完成的修正 |
| --- | --- | --- | --- |
| P0-1 | `process_module.cpp` 的 workflow device stopper、`runStop()` | 普通 Stop 先报告正常停止，失败可能被忽略。 | 已将普通 Stop 改为 Stop-lane completion 事务；排队/执行失败进入 Error，成功后才进入 Stopped。 |
| P0-2 | `ProcessModule::requestStop()/resetStop()/setDigitalOutput()/setAxisEnabled()` | Stop 后交互 IO 可重新进入，恢复未复核。 | 已引入 stop-only 门禁；安全停机事务未完成时交互 IO 拒绝，成功后恢复队列通道。停止复位在 executor 中安全停机，复查加工配置和 contour-boundary 健康状态后才回 Idle。 |

### P1：并发、所有权与数据一致性

| 编号 | 文件/位置 | 问题与影响 | 建议 |
| --- | --- | --- | --- |
| P1-1 | `cad_document_io_service.cpp` 的异步 import/export 与 `closeDocument()` | TaskManager worker 捕获裸 `LcncDocument*`，而关闭工作区不会先取消该文档任务；关闭期间可能发生悬空访问。导入还直接在 worker 修改活动 XCAF 文档，与 GUI 读取缺少串行边界。 | 任务只构建 detached 结果；以 workspace/document generation 验证后在所属线程提交。关闭前按文档取消并等待任务。 |
| P1-2 | 同文件的 STL/BREP/STEP/IGES 导入与 `saveDocument()` | STL/BREP 在 `addShapeEntity()` 后才检查取消，失败/取消可留下半提交实体；导入已有文档不是事务式。`saveDocument(document, ...)` 只校验指针，实际调用当前活动 workspace 的保存/导出接口，非活动文档可能保存错工程。 | 所有格式先导入临时 XCAF/shape snapshot，成功且 generation 匹配后一次提交；保存 API 必须显式携带 workspace/document identity。 |
| P1-3 | `process_status_service.cpp` 的 `stop()` 与 completion | 旧 completion 可污染新会话。 | 已按 start generation 隔离 completion，并在 stop 取消 pending ticket；快速 stop/start 回归通过。 |
| P1-4 | `cam_display_projection_service.cpp` | 旧 context 的 AIS 可能用新 context 移除。 | 已按 document 保存 projection 与 owning context，并在 context 更换前用旧 context 清理；双工作区 offscreen 回归仍待添加。 |
| P1-5 | `machining_face_pipeline_service.cpp::replaceAutomaticFaces()` | 等价自动面重算更换 ID。 | 已以 workpiece entry、role 与签名复用 ID；重建相同 box 的独立回归通过。 |
| P1-6 | `device_command_queue.cpp::runWorker()` | completion 抛异常会重复回调。 | 已分离 command/completion 异常边界，completion 只通知一次；throwing-completion 回归通过。 |

### P2：规范与可维护性

| 编号 | 文件/范围 | 问题 | 建议 |
| --- | --- | --- | --- |
| P2-1 | `cad_document_io_service.cpp`、`process_workflow_executor.cpp`、`cam_module.cpp` 等 | 仍有 catch 未执行 `LCNC_ERR`，其中 CAD 新 service 的 OCC catch 违反本轮边界约定。 | 按异常边界补结构化错误日志，再转换为 UI 错误。 |
| P2-2 | `process_manual_motion_service.cpp`、`process_interactive_io_service.cpp` | 新增 27 处可见 `tr()` 文本，但 `lasercnc_zh_CN.ts` 没有对应 service context；中文界面会回退英文，且部分新文本缺相邻中文翻译注释。 | 更新 TS catalog 和注释，并加翻译 catalog 检查。 |
| P2-3 | `CadTaskPanelController` 与 `MainWindow` | 两处重复 primitive/feature/transform 参数映射；controller 仍直接持有 `CadModule*`。后续参数 schema 变化容易出现预览与执行不一致。 | 抽取共享 typed request/controller contract，并让 controller 依赖 `CadModelingController`/facade。 |

## 3. 全项目架构评价

### 已成立的边界

- Kernel、统一工程文档、MachineWorkspace 和 workspace-bound GuiDocument 的总体所有权清晰。
- core/view 分层与 Process OCC-free 门禁有效。
- 桌面工程/CAM v4、当前 workflow、Process settings v2 的拒绝门禁仍在。
- DeviceCommandQueue 已形成单设备线程、五级优先级、FIFO、合并、取消、超时 barrier 和关闭语义。
- 普通切割真实硬件 sink 的创建、调用和销毁目前都经设备队列执行，轮廓前双门禁仍在。
- ModuleTaskScope 统一了模块级 TaskManager 取消/等待。

### 仍未完成的入口瘦身

1. `AppContext`、commands、module UI、`DialogOptions` 和 `MainWindow` 仍广泛获取 `CadModule/CamModule/ProcessModule`；具体 Module 也仍注册为公共 service。已批准的“外部只依赖 facade/service contract”尚未完成。
2. `ProcessRunCoordinator` 仅是状态迁移表；run/preflight/stop completion、缓存和 UI 信号仍由 `ProcessModule` 组合。`ProcessDeviceRuntime::createMotionSink()` 的 public contract 仍暴露 `ProcessModule*`，`NormalCuttingManager` 和 simulation sink 反向持有入口。
3. `MachiningFacePipelineService` 暴露 mutable `entries()`，`CamModule` 长期持有其可变引用；“service 独占状态、下游只读 immutable snapshot”尚未成立。`CamDisplayProjectionService` 只覆盖加工面，机台、轴导引、刀路、travel path 仍在入口。
4. `CadDocumentIoService` 已注册，但 `CadModelingController`、`CadSelectionController` 尚不存在；CAD facade 仍执行建模、草图、选择和显示业务。
5. `CadTaskPanelController`、工程树写回、最近文件、CAM panel 和机器视图接线仍在 `MainWindow`。当前控制器拆分减少了局部复杂度，但还没有切断 concrete Module 耦合。

### 门禁缺口

当前 `check_architecture.ps1` 不能发现：

- app/commands/module UI 对具体 Module 的获取；
- Process E-stop 后 Interactive/Normal 命令重新进入；
- Process runtime/sink public contract 中的 `ProcessModule*` 反向依赖；
- CAD worker 对活动文档的直接写入和文档关闭竞态；
- CAM projection 是否按 workspace 隔离；
- 所有 catch 是否记录 `LCNC_ERR`；
- 新增 `tr()` 是否进入翻译 catalog。

这些规则应在对应实现修复后加入，避免先把现有债务固化成误报豁免。

## 4. 验证证据

本次修复后工作区实测：

- `git diff --check`：通过。
- `scripts/check_architecture.ps1 -Root .`：通过。
- quality Ninja `/W4 /WX` 增量构建：通过。
- `ctest --test-dir build-cmake-quality -C Debug --output-on-failure`：23/23 通过，总耗时 25.60 s。
- `lcnc_simulator_cmhp_sdk_integration_test`：通过，使用真实 ACS SDK Simulator；该证据不等于 ACS/GTN/激光物理硬件验证。
- `lcnc_startup_smoke_test`：通过，Qt platform plugin 部署当前有效。

新增回归已覆盖队列 completion 抛异常、status 快速 stop/start、自动面稳定 ID、交互 IO 锁存，以及 workspace 关闭被任务守卫拒绝后文档仍存活。Stop 完整失败注入、CAD detached 提交和双工作区 AIS offscreen 场景仍未覆盖。

## 5. 发布判断

当前状态为“审阅发现的 P0 实现缺口及可局部关闭的 P1 竞态已修复，静态/自动化基线绿色；剩余工作集中在 CAD detached 文档事务、完整安全故障注入、CAM immutable/multi-workspace 投影和入口 facade 化”。后续继续按 `todo.md` 的未完成项推进。

物理 ACS/GTN/激光、供应商 abort、Application Verifier/页堆、8 小时资源趋势和性能门限仍是独立发布门禁，不能由 SimulatorCMHP 或普通 CTest 代替。
