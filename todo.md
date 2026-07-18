# LaserCNC 审计与改进计划

审计日期：2026-07-17。审计基线：`main` 工作树，约 6.4 万行 C++ 头文件/实现，CMake 编译清单、分层 include、异步任务、对象所有权、异常处理、兼容代码、文档和构建配置均已纳入检查。

## 已在本轮完成

- [x] 删除 5 份阶段性根目录文档和 93 份过时的自动生成 `.qoder/repowiki` Markdown，建立唯一架构事实源 `ARCHITECTURE.md`。
- [x] 删除未编译且无引用的旧 Process UI、License、Expression、SignalSource、TCP/UDP/HTTP server 和 legacy adapter/types。
- [x] 删除已编译但没有任何运行时入口的平行 communication framework、旧 `ProcessSettings` 壳以及未接入的 `ProcessRuntime/ProcessStateMachine`。
- [x] 删除 `Service` 中已移除设备的 no-op 兼容 API和无引用显示状态字段。
- [x] 修复 `TaskManager` 退出时不等待 worker、未释放任务实体的问题，并使 `waitForDone(timeoutMs)` 真正遵守超时。
- [x] 为 `TaskProgress` 的跨线程 `QString/callback` 状态增加互斥保护。
- [x] 修复 Process 硬件状态轮询和环境监控在析构时可能继续使用裸控制器指针的 use-after-free 风险。
- [x] 移除 `LogModule::instance()` 的确定性堆泄漏。
- [x] 将 Logger 关闭移到所有 GUI、模块、任务和 Kernel 对象析构之后。
- [x] 在 QApplication 析构前显式停止并等待旧 `MessageModule` 线程，消除静态析构顺序风险。
- [x] 删除只为旧 CMake 清单保留的空 `commands_cad.cpp`，由 CMake 显式登记其 Qt 元对象头。
- [x] 修复 `LCNC_WITH_REAL_LASER` 未加入真实激光实现源文件的问题；明确 Qt SerialPort 为当前继承链的必需依赖。
- [x] 修复 Process 分域 TOML 只由 `devices.toml` 触发默认写入的缺陷；每个域现可独立校验、备份损坏文件并原子重建。
- [x] Ninja/MSVC Debug 构建通过，隐藏窗口启动、主窗口正常关闭和退出码 0 已验证。

## P0：发布前必须完成

### 1. 设备访问单线程化

现状：连接/断开、回零、加工、150 ms 状态轮询和 500 ms 环境监控可能从不同 worker 同时调用 `MotionControl` 及供应商 SDK。当前没有可证明的线程安全协议。

- [ ] 建立唯一 `ProcessDeviceCoordinator` 或专用设备线程，所有 ACS/GTN/激光/IO 命令排队串行执行。
- [ ] 读操作和写操作统一经过同一入口，禁止 UI/monitor/workflow 直接取得 `MotionControl*`。
- [ ] 急停使用最高优先级命令并提供同步确认；安全输出复位失败必须进入 Error/EmergencyStop。
- 验收：并发连接、轮询、回零、加工和停止压力测试无竞态；TSan 不适用于当前 MSVC/Qt 组合时，使用供应商模拟器与调用序列日志证明串行性。

### 2. 任务取消与关机协议

现状：`TaskManager` 已能安全等待，但取消仍是协作式；多个硬件/文件任务不检查 `isAbortRequested()`，供应商调用阻塞时退出可能无限等待。

- [ ] 所有长任务保存 task id，模块 `stop()` 先请求取消再等待自己的任务。
- [ ] 循环和阶段边界检查取消；硬件 SDK 配置超时和 abort API。
- [ ] 将“正在连接/断开/回零”纳入正式状态机，禁止状态交叉。
- [ ] 设计超时后的安全降级：停止发新命令、关闭输出、记录错误，不释放仍被 SDK 使用的对象。
- 验收：在导入、CAM 生成、连接、回零各阶段强制关闭应用，进程可控退出且无 UAF/死锁。

### 3. 模块异常边界

现状：`ModuleRegistry` 的 init/start/stop 只在部分路径捕获 `std::exception`，非标准异常可能越过顶层导致进程终止；若模块 init 中途失败，失败模块自身没有统一 rollback 合同。

- [ ] init/start/stop 全部补齐 `catch (...)` 并记录模块 id。
- [ ] 明确 `stop()` 必须能清理“只 init 未 start”的部分状态，且幂等、noexcept。
- [ ] `main()` 增加顶层异常边界，确保 Logger flush 和安全输出收尾。
- 验收：故障注入覆盖 CAD/CAM/Process 每个 init/start 阶段，均能反向清理并产生明确日志。

### 4. 内存与崩溃验证门禁

现状：仓库没有自动化测试目标、ASan/Application Verifier 配置或长稳测试；静态审计与成功构建不能证明不存在泄漏。

- [ ] 增加 MSVC ASan preset，先覆盖 PureSimulation、工程开关、CAM 生成/清空和退出。
- [ ] 增加 Application Verifier/页堆脚本，覆盖 Qt/OCC/供应商 DLL 场景。
- [ ] 建立 8 小时循环：新建/打开/关闭工程、显示/隐藏机台、生成/重算、仿真加工、退出重启。
- [ ] 对 OCC `Handle`、QObject parent tree、后台 future 和供应商句柄分别记录资源基线。
- 验收：ASan 0 error；Application Verifier 0 heap/handle error；长稳测试私有工作集和句柄数无持续单调增长。

### 5. 硬件关闭与日志生命周期

现状：`ProcessModule::stop()` 不执行同步设备断开；控制器由函数内 static 对象持有，GTN 析构未明确断开。旧 `MessageModule` 已在 QApplication 析构前显式停止，但双日志/弹窗线程框架本身仍应移除。

- [ ] 在 Kernel shutdown 前完成同步 safe-stop、断激光、断控制器和监控退出。
- [ ] 取消函数内 static 控制器，将实例所有权交给设备 coordinator。
- [ ] 移除 `MessageModule`/旧三日志系统，统一到 `lcnc::Logger` + 主线程通知服务。
- [ ] 禁止硬编码 `D:/Log`，日志目录统一从应用配置派生。
- 验收：真实/模拟控制器重复连接 100 次、正常退出和异常退出均无残留线程、句柄或输出状态。

## P1：结构稳定性

### 6. 收口 Process 旧框架

- [ ] 将 `Service`、`MotionControl`、`LaserDevice`、`ToolFactory` 的 PascalCase/全局状态接口封装成现代 service 接口。
- [ ] 移除 `ProcessSettingsService::current()`、`DT::*` 等隐藏全局依赖，改为构造注入。
- [ ] `ProcessModule` 只保留 facade、生命周期和信号转发；连接、监控、执行、状态分别下沉。
- [ ] 删除 `IProcessFacade` 中已标记 deprecated 的同步连接别名和 `ProcessCuttingJob::order`。
- 验收：`process_module.cpp` 小于 800 行；硬件 SDK 头只出现在 option-gated 私有 `.cpp`。

### 7. CMake 与可选 SDK 隔离

现状：依赖根目录使用个人绝对路径默认值，`lcnc_common` 广泛暴露 `3rd/include_3rd`，ACS/BDAQ 的旧实现仍存在无条件编译/链接行为。

- [ ] 提供 `CMakePresets.json`，个人路径放入不提交的 `CMakeUserPresets.json`。
- [ ] ACS、GTN、BDAQ、真实激光分别建立私有 adapter target；开关关闭时不解析其任何供应商头。
- [ ] 构建矩阵验证 `all-off`、`GTN`、`ACS`、`real-laser` 和合法组合。
- [ ] 对本轮补齐的 `LCNC_WITH_REAL_LASER=ON` 单独配置并编译验证。
- 验收：干净机器可只凭 preset 配置；每个开关的声明、源文件、include、lib、runtime DLL 完全一致。

### 8. 统一工程文档 API

现状：Workpiece 和 CAM 已物理合并，但仍同时存在 `projectDocument()`、`sourceDocument()`、`workspaceGuiDocument()` 等旧别名，文档规范与代码长期漂移。

- [ ] 选择明确名称（建议 `projectDocument()` + EntityKind/domain API），移除伪装成独立文档的别名。
- [ ] `GuiDocument` 的默认 source document 参数改为显式 `DocumentId/domain`。
- [ ] 机台域只通过 `MachineWorkspace` 暴露，ProjectManager 不再提供模糊所有权接口。
- 验收：同一概念只有一个公开名称；所有权可从类型和 API 直接判断。

### 9. 拆分超大 facade/UI

当前热点：`cam_module.cpp` 约 3600 行、`main_window.cpp` 约 2400 行、`cad_module.cpp`/`process_module.cpp` 约 1800 行、`dialog_options.cpp` 约 1200 行。

- [ ] CamModule 拆为 generation/solve/layer/lead-in/machine-placement coordinators。
- [ ] MainWindow 拆为 workspace view controller、project explorer controller、panel/ribbon composer。
- [ ] CadModule 拆分 document IO、modeling session 和 view synchronization。
- [ ] 选项对话框按页面拆分独立 widget/model。
- 验收：模块 facade 不含大段算法和临时 UI；单文件职责可用一句话描述。

### 10. 自动化架构检查

- [ ] 增加脚本检查 core/view 反向 include、Process OCC 类型、禁用 API 和未列入 CMake 的 `.cpp`。
- [ ] CMake/CI 执行 `git diff --check`、Debug 构建和架构脚本。
- [ ] 为 `LcncProjectPackage`、`CamDataManager`、contour order、状态机和 TaskManager 增加单元测试 target。
- 验收：违反分层或新增孤儿 `.cpp` 时 CI 直接失败。

## P2：兼容与可维护性

### 11. 建立兼容代码退出策略

当前仍有 `.lcnc` v1/v2、`process_cutting_plan.toml`、CAM JSON、旧 workflow tree 和旧字段读取。它们涉及用户数据，不应在没有迁移方案时直接删除。

- [ ] 统计现场最低项目版本，确定最后支持版本。
- [ ] 提供一次性离线升级工具，将旧工程升级到 v3 当前 schema。
- [ ] 建立每个旧版本的只读 fixture；禁止新格式继续双写 legacy 字段。
- [ ] 到期后删除旧读取器、旧字段、别名和迁移分支。
- 验收：兼容代码均有来源版本、测试样本、截止版本和删除 issue。

### 12. 工程自包含与可追溯

- [ ] 将项目使用的工具参数快照写入 `.lcnc`，避免只按全局工具名解析导致历史工程漂移。
- [ ] manifest 记录软件版本、机台构型摘要、配置 schema 与刀路算法版本。
- [ ] 保存采用明确的临时文件 + 原子替换，并验证中断保存不会破坏旧包。
- 验收：工程复制到另一台机器后可还原相同工具参数并检测不兼容机台配置。

## 每轮验证清单

- [ ] `git diff --check`
- [ ] core/view 分层 include 扫描为 0
- [ ] Process OCC 类型扫描为 0
- [ ] 无未列入 CMake 且非明确 optional 的 `.cpp`
- [ ] Debug 构建通过
- [ ] 新建、打开、切换、关闭工程 smoke
- [ ] CAM 生成、显式重算、图层/轮廓状态 smoke
- [ ] Process PureSimulation 启动/暂停/继续/停止/急停 smoke
- [ ] 涉及硬件时完成安全输出和断开检查
