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

现状：`ProcessDeviceCoordinator` 租约已成为 `Service` 的唯一供应商 SDK 串行边界；连接/断开、回零、加工、150 ms 状态轮询、500 ms 环境监控、设置下发和 legacy 步骤服务均持有同一租约。控制器从函数内 static 改为 `Service` 生命周期的 `unique_ptr`，并在 worker 内重新取得控制器，避免重连后使用旧指针。ACS/GTN 已补齐安全关闭路径（ACS 打开失败不再传入无效句柄、断开有 30 秒上限；GTN 清理失败仍继续尝试 `GTN_Close`）。

- [x] 建立 `ProcessDeviceCoordinator` 串行访问租约，并接入现有设备调用路径。
- [x] 安全输出复位返回确认结果；失败记录通道并进入 Error/EmergencyStop。
- 2026-07-18 验证：`git diff --check`、core/view 分层扫描、Process OCC 扫描、MSVC/Ninja Debug 构建、独立启动/退出冒烟均通过。

- [ ] 将租约式 coordinator 升级为专用设备线程/命令队列，并从公开调用点移除 `MotionControl*` 借用。
- [ ] 为 ACS、GTN 和仿真器建立可复现的并发连接、轮询、回零、加工、停止调用序列压力测试。
- 验收：并发连接、轮询、回零、加工和停止压力测试无竞态；TSan 不适用于当前 MSVC/Qt 组合时，使用供应商模拟器与调用序列日志证明串行性。

### 2. 任务取消与关机协议

现状：`TaskManager` 已能安全等待，但取消仍是协作式；多个硬件/文件任务不检查 `isAbortRequested()`，供应商调用阻塞时退出可能无限等待。

- [x] CAD（工程打开、STEP/STL 导入、STEP 导出）、CAM（机台加载）和 Process（连接、断开、回零）全部 `TaskManager` 任务均由所属模块持有 task-id；每个阶段检查取消，`stop()` 请求取消并最多等待 10 秒。
- [x] Process 后台任务捕获 `shared_ptr<Service>`；取消超时后停止发新命令和安全输出复位，但不会销毁仍可能被 SDK 使用的控制器对象。
- [x] Process 以 `Connecting` / `Disconnecting` / `Homing` 设备操作状态拒绝交叉连接、断开和回零。
- 2026-07-18 验证：Debug 构建、独立启动/退出冒烟通过。

- [ ] 所有长任务保存 task id，模块 `stop()` 先请求取消再等待自己的任务。
- [ ] 循环和阶段边界检查取消；硬件 SDK 配置超时和 abort API。
- [ ] 将“正在连接/断开/回零”纳入正式状态机，禁止状态交叉。
- [ ] 设计超时后的安全降级：停止发新命令、关闭输出、记录错误，不释放仍被 SDK 使用的对象。
- 验收：在导入、CAM 生成、连接、回零各阶段强制关闭应用，进程可控退出且无 UAF/死锁。

### 3. 模块异常边界

现状：`ModuleRegistry` 的 init/start/stop 只在部分路径捕获 `std::exception`，非标准异常可能越过顶层导致进程终止；若模块 init 中途失败，失败模块自身没有统一 rollback 合同。

- [x] `ModuleRegistry` 的 init/start/stop 全部捕获 `std::exception` 与未知异常并记录模块 id；init 或 start 失败均反向调用所有已初始化模块（含失败模块自身）的 `stop()`。
- [x] `main()` 建立顶层异常边界；bootstrap 失败不再经会抛异常的 `LCNC_CRIT` 越过 main，而是记录错误、执行日志/消息服务关闭并返回非零。
- 2026-07-18 验证：Debug 构建、独立启动/退出冒烟、Process OCC 扫描通过。

- [ ] init/start/stop 全部补齐 `catch (...)` 并记录模块 id。
- [ ] 明确 `stop()` 必须能清理“只 init 未 start”的部分状态，且幂等、noexcept。
- [ ] `main()` 增加顶层异常边界，确保 Logger flush 和安全输出收尾。
- 验收：故障注入覆盖 CAD/CAM/Process 每个 init/start 阶段，均能反向清理并产生明确日志。

### 4. 内存与崩溃验证门禁

现状：仓库没有自动化测试目标、ASan/Application Verifier 配置或长稳测试；静态审计与成功构建不能证明不存在泄漏。

- [x] 提交 `CMakePresets.json` 的 `asan` 预设；所有项目 C++ 及源码构建的 QuaZip 使用一致的 MSVC ASan STL 注解。2026-07-18 已配置、全量构建并启动/退出通过。
- [x] 增加 `scripts/collect_runtime_baseline.ps1`（私有工作集、工作集、句柄、线程 CSV）和 `scripts/application_verifier.ps1`（仅显式 `-Enable/-Disable` 配置 Heap/Handle 检查）。短时 ASan smoke 已采集 3 个稳定样本。
- [ ] 仍需执行 8 小时循环、Application Verifier 实测与 QObject/OCC Handle/vendor handle 的领域专用基线；未达到长期发布门禁前不得宣称无泄漏。

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

本轮优先稳定性：`Service::shutdownDevices()` 统一激光停止、红光停止、激光断开、运动停止、缓冲停止和控制器断开；Process 模块停机与 Service 析构复用该顺序。Cam/MainWindow/CAD/Options 的大规模拆分延后至稳定性门禁完成后。

2026-07-18：将回零顺序、轴定义比较与仿真轴坐标计算迁至 `runtime/process_axis_utilities`，`ProcessModule.cpp` 从 2231 行降至 2106 行；该工具不访问 UI 或设备 SDK。

本轮：`Service`、`MotionControl`、`LaserDevice`、`LDFactory` 与 `ProcessParameterRegistry` 均改为显式注入 `ProcessSettingsService`；`SimulatorCMHP`、ACS、GTN 和全部激光适配器构造时均传入同一设置实例。`LDFactory` 已从静态设备对象改为 `Service` 成员，流程步骤注册表在模块初始化注入设置、停止时清空借用引用；节点编辑 UI 通过注册表读取受控配置。`ProcessSettingsService::current()` 与静态指针已删除。Process 初始化顺序为“设置初始化成功后再构造 Service”，成员析构顺序保证借用设置的 Service 先于设置服务销毁。`DT::*` 及 PascalCase 兼容接口仍待继续收口。2026-07-18：默认 GTN Debug、独立 ACS Debug 以及 ASan Debug 构建和各自 3 项 CTest 通过。

2026-07-18（连接与轮询安全收口）：`SimulatorCMHP` 明确归属 ACS，连接顺序恢复为打开 ACS Simulator、停止缓冲、加载源码受控并由 CMake 部署的 `Simulator.prg`、执行 `AfterOpenComm()`；all-off 仅以 `Simulator` 身份进入 PureSimulation。启用 ACS/GTN 时 Pure 默认关闭，ACS、GTN、SimulatorCMHP 断连或连接失败均进入 Error，`MotionSinkFactory` 和激光器工厂不再静默回退仿真。控制器状态/安全 IO/串口外设分别使用 150 ms、500 ms、2 s 的专用单线程池，`QSerialPort` 已迁出 GUI 线程；ACS 加工期间坐标通知限流为 10 Hz 并使用 `QPointer` 防止退出后 UAF。已验证 all-off Debug、ACS+GTN Debug、两套 4 项 CTest、架构扫描，以及 ACS+GTN GUI 6 秒启动和干净退出。ASan 干净构建与 4 项 CTest 通过；ASan GUI 可运行到主窗口，但关闭时被系统注入的 `SogouPY.ime`/`SogouTSF.ime` 报告第三方 heap-use-after-free，调用栈不经过项目代码，需在禁用该输入法注入的发布测试环境复验。未做真机验证。

同日：`ToolFactory` 未命中查询不再写入隐式空工具，重复索引使用 `insert_or_assign` 覆盖旧参数，避免项目/设置切换后保留陈旧工具项。新增 `ProcessRuntimeConfiguration` 并经 ProcessModule → Service → MotionControl 注入；GTN/ACS 的标准轴、扩展轴、仿真选择、旧 IO 列表缓存和 ACS 权限兼容状态均已脱离 `DT::*`。`BASE` 不会作为可用运动轴传入设备层。2026-07-18：新增 `lcnc_process_runtime_configuration_test`，覆盖轴归一化、伪轴过滤、扩展轴匹配、仿真和权限状态；Debug 构建、4 项 CTest、架构检查和 Process 调用扫描通过。`DataType.h` 的无调用方 `DT` 静态兼容壳已删除，保留枚举、数据结构和精度常量。

2026-07-18：`MotionControl.h` 与 `LaserDevice.h` 已移除对 `MessageModule.h` 的公共包含；实际提示宏移入各设备 `.cpp`。`MessageModule::WriteLog` 和旧设备日志宏均路由至 `lcnc::Logger`；`LogModule.h/.cpp` 已删除，硬编码三日志路径消失。Debug 构建和 `architecture_checks` 通过。`MessageModule` UI 队列的后续替换仍未完成。

同日：`ProcessModule` 的监控、轮询和主面板 IO 配置改为显式使用其 `m_settingsService` 注入成员，模块实现中 `ProcessSettingsService::current()` 已清零。

- [ ] 将 `Service`、`MotionControl`、`LaserDevice`、`ToolFactory` 的 PascalCase/全局状态接口封装成现代 service 接口。
- [x] 移除 `ProcessSettingsService::current()` 隐藏全局依赖，改为构造/生命周期注入。
- [x] 将 `DT::*` 的轴掩码、扩展轴、IO 列表和权限状态迁移至显式 Process 运行时配置对象；已确认 ACS/GTN 的调用方不再读取 DT。
- [x] 删除 `DataType.h` 中已无调用方的 `DT` 兼容类型壳，并保留仍在使用的枚举定义。
- [ ] `ProcessModule` 只保留 facade、生命周期和信号转发；连接、监控、执行、状态分别下沉。
- [x] 删除 `IProcessFacade` 中已标记 deprecated 的同步连接别名和 `ProcessLayerJob::order`；UI 统一使用异步 `connectAllDevices()` / `disconnectAllDevices()`，命令命名同步为 Devices。
- 验收：`process_module.cpp` 小于 800 行；硬件 SDK 头只出现在 option-gated 私有 `.cpp`。

### 7. CMake 与可选 SDK 隔离

现状：依赖根目录使用个人绝对路径默认值，`lcnc_common` 广泛暴露 `3rd/include_3rd`，ACS/BDAQ 的旧实现仍存在无条件编译/链接行为。

- [x] 提交 `CMakePresets.json`（all-off `debug`、`asan`、`acs`、`gtn`）及本机路径模板 `CMakeUserPresets.json.template`。
- [x] `SimulateCMHPMotionControl` 已改为不含供应商 SDK 的本地状态控制器；ACS 源、文本 sink、adapter、include 和 import library 仅在 `LCNC_WITH_ACS=ON` 时加入。2026-07-18 已完整配置、编译和链接 all-off、ACS、GTN 三个 `LaserCNC.exe`；all-off 已完成 8 秒启动冒烟。
- [ ] BDAQ 与真实激光仍需独立 adapter target/构建矩阵；个人绝对路径默认值仍待移至 user preset。

- [ ] 提供 `CMakePresets.json`，个人路径放入不提交的 `CMakeUserPresets.json`。
- [ ] ACS、GTN、BDAQ、真实激光分别建立私有 adapter target；开关关闭时不解析其任何供应商头。
- [x] 将 `SimulateCMHPMotionControl` 改为控制器无关的仿真实现，使 ACS 基类、头文件和 import library 在 all-off 构建中完全消失。
- [ ] 构建矩阵验证 `all-off`、`GTN`、`ACS`、`real-laser` 和合法组合。
- [ ] 对本轮补齐的 `LCNC_WITH_REAL_LASER=ON` 单独配置并编译验证。
- 验收：干净机器可只凭 preset 配置；每个开关的声明、源文件、include、lib、runtime DLL 完全一致。

### 8. 统一工程文档 API

现状：Workpiece 和 CAM 已物理合并。2026-07-18 已删除 `ProjectWorkspace::projectDocument()`、`GuiApplication`/CAD/CAM/AppContext 的 `workspaceGuiDocument()`，并将 `GuiDocument::sourceDocument()` 改为 `document()`；带 workspace id 的视图切换信号明确为 `activeWorkspaceDocumentChanged`，避免和单文档信号重载混淆。

- [x] 清除 `projectDocument()`、`workspaceGuiDocument()`、`sourceDocument()` 公开别名；活动视图统一为 `activeGuiDocument()`。
- [ ] `GuiDocument` 的默认 source document 参数改为显式 `DocumentId/domain`。
- [ ] 机台域只通过 `MachineWorkspace` 暴露，ProjectManager 不再提供模糊所有权接口。
- 验证：2026-07-18 `rg "projectDocument\\(|workspaceGuiDocument\\(|ensureProjectDocument\\(|sourceDocument\\(" src` 仅命中 `ProjectWorkspace` 构造形参名；Debug 构建和 8 秒启动冒烟通过。
- 验收：同一概念只有一个公开名称；所有权可从类型和 API 直接判断。

### 9. 拆分超大 facade/UI

当前热点：`cam_module.cpp` 约 3600 行、`main_window.cpp` 约 2400 行、`cad_module.cpp`/`process_module.cpp` 约 1800 行、`dialog_options.cpp` 约 1200 行。

- [ ] CamModule 拆为 generation/solve/layer/lead-in/machine-placement coordinators。
- [ ] MainWindow 拆为 workspace view controller、project explorer controller、panel/ribbon composer。
- [ ] CadModule 拆分 document IO、modeling session 和 view synchronization。
- [ ] 选项对话框按页面拆分独立 widget/model。
- 验收：模块 facade 不含大段算法和临时 UI；单文件职责可用一句话描述。

### 10. 自动化架构检查

现状：2026-07-18 新增 `scripts/check_architecture.ps1`，并以 CTest 的 `architecture_checks` 注册。它检查 core/view 分层 include、Process OCC include、淘汰文档 API 和未在 `CMakeLists.txt` 登记的 `.cpp`。`cmake --preset debug && ctest --test-dir build/debug --output-on-failure && cmake --build --preset debug` 已通过。

- [x] 将分层、Process OCC、禁用 API、孤儿 `.cpp` 检查接入 CTest。

- [ ] 增加脚本检查 core/view 反向 include、Process OCC 类型、禁用 API 和未列入 CMake 的 `.cpp`。
- [ ] CMake/CI 执行 `git diff --check`、Debug 构建和架构脚本。
- [~] 为 `LcncProjectPackage`、`CamDataManager`、contour order、状态机和 TaskManager 增加单元测试 target。已新增 `lcnc_project_package_test`，覆盖 v4 工具快照 round-trip 与缺快照拒绝；新增 `lcnc_task_manager_test`，覆盖协作取消、超时等待和异常任务失败；CamDataManager、轮廓排序与状态机待补。
- 验收：违反分层或新增孤儿 `.cpp` 时 CI 直接失败。

## P2：兼容与可维护性

### 11. 建立兼容代码退出策略

桌面端现在只接受当前 v4 `.lcnc`；`lcnc_project_upgrade` 是唯一允许读取 v1/v2/v3 和 `process_cutting_plan.toml` 的入口，并会以 v4 重写输出。CAM JSON 与旧 workflow tree 的读取路径仍待移除。

- [ ] 统计现场最低项目版本，确定最后支持版本。
- [x] 提供 `lcnc_project_upgrade <input.lcnc> <output.lcnc> [--tools <tools.toml>]`，将旧工程升级到 v4 当前 schema；输入输出不得相同。升级器保留源 `tools.toml`；缺失时必须显式提供快照。
- [~] 建立每个旧版本的只读 fixture；禁止新格式继续双写 legacy 字段。`lcnc_project_package_test` 已在临时目录生成 v1/v2/v3 结构 fixture（无 `tools.toml`，迁移时显式补充），并实际调用 `lcnc_project_upgrade.exe` 回归；仍缺现场历史工程样本。
- [ ] 到期后删除旧读取器、旧字段、别名和迁移分支。
- 剩余风险：结构 fixture 已覆盖 v1/v2/v3，但尚无现场历史工程样本，未能完成真实历史包的升级回归；离线升级无法从 v1/v2 的全局配置名反推出项目专属参数快照，现已要求 `--tools` 显式输入；运行时快照必须包含 OCCT 对应的 TBB DLL。
- 验收：兼容代码均有来源版本、测试样本、截止版本和删除 issue。

### 12. 工程自包含与可追溯

本轮验证（2026-07-18）：主 `build` 全量编译通过；`scripts/check_architecture.ps1 -Root <repo>` 与 `ctest --test-dir build --output-on-failure` 通过。`lcnc_project_package_test` 验证 v4 工具快照 round-trip、缺 `tools.toml` 拒绝、staging 原子替换、失败保存后既有包 SHA-256 不变，并生成 v1/v2/v3 结构 fixture 后实际调用 `lcnc_project_upgrade.exe --tools`，验证其输出能由 v4 桌面加载；其首次失败暴露了 Win32=32 句柄冲突，现已通过在压缩前析构 `QTemporaryFile` 修复。工具测试还发现 `--tools` 曾将 TOML 文本误作文件路径，现已改为流式解析。机台指纹不匹配现会进入 session 安全门禁，真实加工拒绝、仿真允许。`build/LaserCNC.exe` 启动 12 秒仍存活后主动停止。工具快照序列化已扩展为 `Tool::SetFromTable()` 覆盖字段的可逆集合，并修复复制时遗漏的模拟量与脉冲参数。

- [x] 桌面端将项目使用的工具参数完整快照写入包内 `tools.toml`；v4 保存/加载均验证该资源存在，加载时项目快照优先恢复到 `ToolFactory`。
- [x] v4 manifest 记录软件版本、机台构型 SHA-256 指纹、配置 schema 与刀路算法版本。
- [x] 打开工程时比较机台构型指纹；不匹配会发出 UI 警告并标记 session，真实加工预检拒绝启动，仿真仍可用于检查工程。
- [x] `.lcnc` 归档先写同目录 staging 文件，Windows 使用 `MoveFileExW(..., MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)` 原子替换；压缩或替换失败时旧包保留。
- [ ] 通过故障注入验证保存中断不会破坏旧包。
- [ ] 用真实 v1/v2/v3 包验证离线升级后的 `tools.toml`、跨机恢复与机台指纹提示。
- 验收：工程复制到另一台机器后可还原相同工具参数并检测不兼容机台配置。

## 每轮验证清单

- [ ] 排查 `build/debug/LaserCNC.exe` 的 Qt6Cored.dll `0xc0000005` 启动崩溃；主 `build/LaserCNC.exe` 已通过 8 秒启动 smoke，二者运行时部署配置不一致。

- [x] 当前 ASan preset 重新配置、构建并通过全部 CTest；修复 CTest 的 `0xc0000135`，CMake 现自动部署 MSVC ASan runtime 与 OCCT TBB。`build/asan/LaserCNC.exe` 12 秒启动 smoke 存活，未生成 ASan 错误日志；45 秒基线在启动约 10 秒后私有工作集/句柄/线程稳定。资源脚本会在异常退出时仍输出 CSV，并支持私有工作集/句柄增长阈值。8 小时资源趋势与 Application Verifier/页堆仍待执行：本机 `appverif.exe` 非交互 CLI 超时且未留下配置，须在可交互现场环境执行。

- [ ] `git diff --check`
- [ ] core/view 分层 include 扫描为 0
- [ ] Process OCC 类型扫描为 0
- [ ] 无未列入 CMake 且非明确 optional 的 `.cpp`
- [ ] Debug 构建通过
- [ ] 新建、打开、切换、关闭工程 smoke
- [ ] CAM 生成、显式重算、图层/轮廓状态 smoke
- [ ] Process PureSimulation 启动/暂停/继续/停止/急停 smoke
- [ ] 涉及硬件时完成安全输出和断开检查
