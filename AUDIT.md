# LaserCNC 全面代码与架构审计

审计日期：2026-07-30
审计基线：`main`，`3ae0fa8 feat: add initial Chinese localization` 加当前未提交工作区
结论：当前改动可以作为阶段性收口提交。质量预设（ACS+GTN、`/W4 /WX`）、all-off、ACS-only、GTN-only、real-laser、ASan 和 Visual Studio ACS+GTN 构建矩阵均已通过对应 CTest；其中包含真实 ACS `SimulatorCMHP`/`Simulator.prg` 初始化与设备线程会话回归。Process 唯一设备执行入口、剩余大型 facade 拆分、真机、交互内存和长稳验证仍未完成，当前不是生产发布版本。

## 1. 审计范围与方法

本次覆盖：

- `src/` 下的应用、core、view、CAD、CAM 和 Process 源码。
- `CMakeLists.txt`、全部 CMake preset、新增及既有 CTest 和维护脚本。
- `ARCHITECTURE.md`、`DELIVERY.md`、`todo.md`、`Readme.md` 的事实一致性。
- 当前工作区既有未提交改动；审计不回退、不覆盖其业务意图。

逐文件检查包含：

1. CMake 收录关系、成对头/实现文件引用和无消费者实现。
2. `core -> view/modules/app`、`view -> modules/app`、Process -> OCC 等依赖边界。
3. 淘汰文档 API、Process settings singleton、设备公共头兼容日志泄漏。
4. 迁移残留分支、空兼容函数、未使用节点类型、异常吞没和旁路日志。
5. 文件规模、命名规范、兼容格式、可选硬件编译边界和测试覆盖。

未在下文单列的文件未发现可独立证明的问题；这表示通过了本轮静态规则与引用检查，不等于已证明无逻辑缺陷。

## 2. 本次已清理

### 2.1 可证明的死实现

以下文件只有自身实现引用，没有工厂注册、构造调用、接口消费或测试引用，已从源码和 CMake 删除：

| 删除范围 | 原问题 |
| --- | --- |
| `core/algorithms/cad/sketch_constraints.*` | 占位式自由度估算器从未接入草图系统，结果不能代表真实约束求解。 |
| `modules/cad/sketch/sketch_serializer.*` | 未接入工程持久化、撤销或草图管理链路的平行序列化格式。 |
| `modules/process/controllers/acs_motion_controller_adapter.*` | 与实际 `ACSMotionControl`/sink 路径并存但从未实例化的第二套 ACS 控制器。 |
| `modules/process/controllers/gtn_motion_controller_adapter.*` | 与实际 `GTNMotionControl`/buffered sink 路径并存但从未实例化的第二套 GTN 控制器。 |
| `modules/process/toolpath/process_tool_matcher.*` | 无调用方且始终返回默认工具参数，可能掩盖未绑定工具。 |

共删除 10 个文件、884 行源码。

### 2.2 项目树迁移残留

机台节点已经迁移到独立 `WidgetMachineTree`，但工程树仍保留四种永远不会生成的节点类型及其分支。已清理：

- `MachineRoot`、`MachineAxis`、`MachineShape`、`MachineUnassignedGroup`。
- `isMachineProjectNode()` 永远返回 `false` 的兼容函数。
- 工程树中的机台选择、显隐、右键解绑和轴清空死分支。
- 已失效的 `AxisName` item role 与 `ProjectExplorerNode::axisName`。
- `selectProjectExplorerEntries(..., cadOnly)` 的伪双域参数，现只处理 Workpiece；机台选择由独立机台面板处理。

这次清理恢复了“工程树负责 Workpiece/CAM，机台树负责 Machine”的 UI 所有权边界。

### 2.3 异常与日志边界

已将以下吞没或旁路异常统一写入 `lcnc::Logger`：

- CAM 刀路 TOML 和历史切割计划解析。
- Process 工程工具快照、流程文件与 ToolFactory 恢复。
- Tool 字段转换。
- Process 设置命令原来的 `qWarning`。
- ULTRON 十六进制字段转换。

### 2.4 架构门禁

`scripts/check_architecture.ps1` 新增：

- `core/algorithms` 禁止依赖 QWidget/QAction/QDialog、文档、GuiDocument 和 Kernel。
- Process 设备公共头禁止暴露 `MessageModule`/`process_log_compat`。
- Process 禁止重新引入 `ProcessSettingsService::current()`。

### 2.5 本阶段结构、格式与测试收口

- `DeviceCommandQueue` 增加唯一命令 ID 和 `Succeeded / Failed / Superseded / Cancelled / Shutdown / TimedOut` completion；同 key 合并时新命令取得新 ID，被替换命令只完成一次并返回 `Superseded`。
- 引入执行线程私有的 `ProcessDeviceRuntime` 和 `ProcessRunCoordinator`；停机顺序调整为停止接收普通命令、取消并等待任务、停止监控、提交 Stop 安全输出、断开设备、关闭执行线程。
- Process 目录和文件迁移为 snake_case，删除 `MessageModule`、兼容日志层和 legacy workflow service 别名。
- MainWindow 的 workspace、工程树和视图状态职责已下沉到三个 controller；CAM 增加 `ToolpathGenerationService`，提交结果前校验输入 revision；CAD 算法统一抛出参数/OCC 异常并由模块边界记录和转换。
- 桌面和库仅接受项目/CAM v4、当前 workflow schema 与 Process settings schema v2；离线升级器和应用层迁移、双读双写逻辑已删除。
- 新增队列、运行状态、CAD 异常、CAM 数据/排序/生成、快照并发、工程树控制器、当前 schema 和 SimulatorCMHP SDK 测试，并加入启动冒烟与统一质量脚本。

## 3. 当前架构评价

### 稳固边界

- Kernel、统一工程文档、MachineWorkspace、workspace-bound GuiDocument 的所有权方向清晰。
- Process 未包含 OCC 类型，只消费 `ToolpathExportSnapshot`。
- CAD/CAM/Process 的模块启动顺序和反向停止由 ModuleRegistry 管理。
- `.lcnc` v4 使用 staging + 原子替换；应用层只接受当前格式并明确拒绝历史包。
- 设备命令队列、工作流线程、TaskManager 和分级轮询已经形成可测试的异步骨架。

### 仍需收口的结构风险

| 优先级 | 文件/范围 | 问题 | 建议 |
| --- | --- | --- | --- |
| P0 | `modules/process/runtime/process_device_coordinator.h`、`system/service.*`、`process_module.cpp` | 静态扫描仍有 41 处 runtime 外的设备锁或控制器/激光器指针访问，队列尚不是唯一 SDK 入口。 | 真机验证前不得移除防御锁；分阶段把剩余访问迁入设备执行上下文，并保留 Stop 优先级、超时隔离和对象保活语义。 |
| P0 | ACS/GTN/真实激光路径 | 构建矩阵和 SimulatorCMHP SDK 测试不能证明物理设备停机、急停、断开和超时安全。 | 完成故障注入、100 次连接/断开与加工停止循环、输出安全检查。 |
| P1 | `modules/cam/cam_module.cpp` | 约 6,606 行；toolpath generation 已抽离，但加工面、机台标定和显示投影仍集中。 | 继续按 machining-face pipeline、machine calibration、display projection 拆 service。 |
| P1 | `modules/process/process_module.cpp` | 约 2,876 行；run coordinator 已抽离，但连接、预检、监控和 UI 投影仍耦合。 | 继续下沉 connection、preflight 和 status service。 |
| P1 | `modules/cad/cad_module.cpp` | 约 2,297 行，CAD facade 与多类建模会话仍偏重。 | 将文档 IO、草图/特征和选择刷新继续委托给正式 controller/service。 |
| P1 | Process 公开设备 API | snake_case 文件迁移已完成，但 `Service::motionControl()`、`laserDevice()` 和 `lockDeviceAccess()` 仍被业务层使用。 | 与唯一 executor 迁移同步删除旧 API，不保留兼容别名。 |
| P1 | real-laser 编译策略 | ACS+GTN 质量预设已通过 `/W4 /WX`，但 real-laser 仍有旧厂商协议适配器告警。 | 逐 target 修正告警并记录必要豁免，使 real-laser 也可启用 `/WX`。 |

## 4. 文件级热点

本轮超过 1,000 行的 11 个文件均已检查。除供应商 `bdaqctrl.h` 外，主要热点为：

- `cam_module.cpp`：领域职责过多，是当前最高维护复杂度。
- `main_window.cpp`：三个 controller 已抽离，但 Ribbon、状态与残余接线仍较多。
- `process_module.cpp`：安全关键 facade 仍过大，拆分必须保持设备租约和关闭顺序。
- `cad_module.cpp`：应继续利用已有 `services/`，避免新增内联业务。
- `gtn_motion_control.cpp`、`acs_motion_control.cpp`：供应商语义密集，修改需对应 SDK/真机验证。
- `laser_toolpath.cpp`：多种板材/管材候选与安全回退集中，不能用简单删分支方式“瘦身”。
- `dialog_options.cpp`、`gui_document.cpp`、`widget_cad_task_panel.cpp`：已达到下一轮拆分阈值。

## 5. 验证证据

已完成：

- 独立 Ninja 子生成树：all-off（16/16）、ACS-only（17/17）、GTN-only（16/16）、ACS+GTN quality（17/17）、real-laser（17/17）和 ASan ACS+GTN（17/17）均构建并通过 CTest。
- Visual Studio/MSBuild ACS+GTN Debug 构建和 17/17 CTest 通过；启动冒烟首次发现的 `zlib1.dll` 部署缺失已通过 POST_BUILD 修正。
- 质量预设对项目源码启用 `/W4 /WX`；第三方和供应商头按 external policy 处理。
- SimulatorCMHP 测试使用真实 `acsc_OpenCommSimulator()` 与部署的 `Simulator.prg`，完成 100 次设备线程会话命令及同线程断开/销毁。
- `scripts/check_architecture.ps1 -Root .`、旧格式/API/Process-OCC 扫描和 `git diff --check` 通过。
- 工程 v4 staging/原子替换、缺资源和旧版本拒绝，队列 completion、CAM stale result、状态机、快照并发及 UI controller 均已有自动化回归。

未完成：

- SimulatorCMHP 100 次完整进程级重连及全动作循环；供应商 RPC 释放窗口会在连续数轮后拒绝新句柄，不能用纯软件 Simulator 替代。
- ACS/GTN/激光真机验证和不可中断供应商调用故障注入。
- Application Verifier、页堆、受控 ASan GUI 和 8 小时资源趋势。
- 大模型取消响应和 GUI 批提交性能门限。

## 6. 发布判断

当前状态是“源码清理完成一批、架构静态门禁通过、ACS+GTN Debug 与自动化测试通过”。这足以作为下一轮集成测试基线，不足以证明：

- 无内存泄漏或线程竞态。
- 供应商 SDK 阻塞时仍能满足停机时限。
- 真机输出在所有异常路径都已安全复位。
- 可标记生产发布。

后续执行顺序见 `todo.md`。
