# LaserCNC 全面代码与架构审计

审计日期：2026-07-30
审计基线：`main`，`040b089 feat(cam): add staged machining face pipeline` 加当前未提交工作区
结论：架构门禁、ACS+GTN Debug 构建和 7 项 CTest 已通过；当前可继续集成测试，但尚不具备生产发布所需的真机、长稳和内存验证证据。

## 1. 审计范围与方法

本次覆盖：

- `src/` 下 346 个 C/C++ 文件，共约 7.8 万行。
- `CMakeLists.txt`、全部 CMake preset、6 个测试源文件和维护脚本。
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

## 3. 当前架构评价

### 稳固边界

- Kernel、统一工程文档、MachineWorkspace、workspace-bound GuiDocument 的所有权方向清晰。
- Process 未包含 OCC 类型，只消费 `ToolpathExportSnapshot`。
- CAD/CAM/Process 的模块启动顺序和反向停止由 ModuleRegistry 管理。
- `.lcnc` v4 使用 staging + 原子替换，桌面加载与离线升级边界明确。
- 设备命令队列、工作流线程、TaskManager 和分级轮询已经形成可测试的异步骨架。

### 仍需收口的结构风险

| 优先级 | 文件/范围 | 问题 | 建议 |
| --- | --- | --- | --- |
| P0 | `modules/process/runtime/process_device_coordinator.h`、`System/Service.*`、`process_module.cpp` | `DeviceCommandQueue` 已承接主要调度，但递归租约仍是最终 SDK 串行边界，设备队列尚不是唯一 SDK 入口。 | 真机验证前不得移除租约；分阶段把剩余直接访问迁入唯一设备执行上下文，并保留 Stop 优先级和对象保活语义。 |
| P0 | ACS/GTN/真实激光路径 | 本次只有 ACS+GTN 编译与无硬件测试，无法证明真机停机、急停、断开和超时安全。 | 完成故障注入、100 次连接/断开与加工停止循环、输出安全检查。 |
| P1 | `modules/cam/cam_module.cpp` | 6,382 行，生成、机台、标定、渲染协调和持久化投影集中。 | 按 machining-face pipeline、toolpath generation、machine calibration、display projection 拆 service。 |
| P1 | `app/main_window.cpp` | 约 2,700 行，工作区、工程树、视图、Ribbon 和状态同步集中。 | 按 workspace presenter、project explorer controller、view state controller 拆分。 |
| P1 | `modules/process/process_module.cpp` | 约 2,600 行，facade、设备生命周期、预检、监控和 UI 投影耦合。 | 优先下沉 connection/session、preflight、poll projection 和 run-state coordinator。 |
| P1 | `modules/cad/cad_module.cpp` | 2,193 行，CAD facade 与多类建模会话仍偏重。 | 将导入/导出、草图、特征和选择刷新继续委托给现有 services。 |
| P1 | 45 个 Process legacy 文件 | `Connect/Laser/MotionControl/Setting/System/Tool` 中仍有 PascalCase 文件和目录。 | 与接口迁移一起分批重命名，避免只改文件名却保留旧 API。 |
| P1 | `core/algorithms/cad/*` | 纯算法内部捕获 `Standard_Failure` 并转错误字符串，与“算法抛出、调用方统一记录”的规则不完全一致。 | 统一一种异常契约；不要在纯算法层引入 UI 或全局 Kernel。 |
| P1 | 编译策略 | 项目代码当前没有统一 `/W4` 或等价告警基线。 | 先建立告警清单与豁免，再分 target 提升到 `/W4`；不应一次性对供应商头启用。 |
| P2 | 兼容读取 | CAM JSON、workflow tree、旧工具字段仍有兼容路径，部分没有明确删除版本。 | 每条兼容路径绑定 schema、fixture、升级器和删除版本，桌面端避免长期双读/双写。 |

## 4. 文件级热点

本轮超过 1,000 行的 11 个文件均已检查。除供应商 `bdaqctrl.h` 外，主要热点为：

- `cam_module.cpp`：领域职责过多，是当前最高维护复杂度。
- `main_window.cpp`：UI 事件编排过多；本轮已先删除机台树迁移残留。
- `process_module.cpp`：安全关键 facade 仍过大，拆分必须保持设备租约和关闭顺序。
- `cad_module.cpp`：应继续利用已有 `services/`，避免新增内联业务。
- `GTNMotionControl.cpp`、`ACSMotionControl.cpp`：供应商语义密集，修改需对应 SDK/真机验证。
- `laser_toolpath.cpp`：多种板材/管材候选与安全回退集中，不能用简单删分支方式“瘦身”。
- `dialog_options.cpp`、`gui_document.cpp`、`widget_cad_task_panel.cpp`：已达到下一轮拆分阈值。

## 5. 验证证据

已完成：

- 使用 `cmake --fresh --preset acs-gtn` 将误配的 Visual Studio 生成树恢复为仓库规定的 Ninja Multi-Config。
- `cmake --build --preset acs-gtn-debug --parallel 16`：通过，成功生成 `x64/Debug/LaserCNC.exe`。
- `ctest --test-dir build --build-config Debug --output-on-failure`：7/7 通过。
- `scripts/check_architecture.ps1 -Root .`：通过。
- CMake 孤儿 `.cpp`：0。
- 未被其他翻译单元消费的成对头/实现：0。
- 淘汰文档 API、Process OCC include、`ProcessSettingsService::current()`：0。
- `git diff --check`：无空白错误；仅有 Git 的 LF/CRLF 工作区提示。

未完成：

- all-off、ACS-only、GTN-only、ASan、real-laser 构建矩阵。
- GUI 启动/退出冒烟与交互回归。
- ACS/GTN/激光真机验证。
- Application Verifier、页堆、受控 ASan GUI 和 8 小时资源趋势。

## 6. 发布判断

当前状态是“源码清理完成一批、架构静态门禁通过、ACS+GTN Debug 与自动化测试通过”。这足以作为下一轮集成测试基线，不足以证明：

- 无内存泄漏或线程竞态。
- 供应商 SDK 阻塞时仍能满足停机时限。
- 真机输出在所有异常路径都已安全复位。
- 可标记生产发布。

后续执行顺序见 `todo.md`。
