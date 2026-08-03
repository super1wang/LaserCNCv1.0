# LaserCNC 当前交付状态

复核日期：2026-08-03

## 结论

当前代码已完成一轮阶段性结构、当前格式和自动化测试收口。2026-08-03
对今日 23 个提交及全项目架构复审后，quality CMake/Ninja ACS+GTN Debug
（`/W4 /WX`）与 23/23 CTest 通过；随后优先修复 Stop/E-stop 安全事务、队列
completion、轮询跨代、CAM ID/context 和 CAD 部分关闭/保存缺口，并重新验证。

本轮已优先关闭审阅中的 Stop/E-stop、队列 completion、轮询跨代、CAM ID/context
和 CAD 部分关闭/保存问题；剩余 CAD detached 事务、完整安全失败注入和多工作区
投影回归仍必须关闭。除真机安全、供应商阻塞故障、ASan/Application Verifier 和
长时间资源趋势外，`AUDIT.md` 的剩余 P1 问题仍是发布前条件。

## 已实现并验证

- 删除 5 组无消费者的平行/占位实现，共 10 个文件。
- 清理工程树中已迁移到独立机台树的节点类型、选择、显隐和菜单死分支。
- TOML、工具、流程和设置命令异常统一写入 `lcnc::Logger`。
- 架构脚本新增 pure-algorithm、设备公共头和 settings 注入门禁。
- CMake/Ninja 与 Visual Studio/MSBuild 已分别固定到 `build-cmake/` 和
  `build-vs/`，禁止继续使用旧共享 `build/`。
- 应用格式已收紧为项目/CAM v4：删除离线升级器、v1/v2/v3 XCAF 和 CAM
  回退、旧 workflow tree 双读写、CAM JSON 自动迁移及旧工具字段宽容读取。
- DeviceCommandQueue 已具备可追踪 completion：优先级、同级 FIFO、按 key
  合并、显式取消、超时和关闭丢弃均有确定结果；同 key 合并时新旧命令保持
  独立 ID 且每个已接受命令只完成一次。
- 新增由设备队列线程调用的 `ProcessDeviceRuntime` 与基础 `ProcessRunCoordinator`，
  并修正模块停机顺序；预检已下沉到 `ProcessPreflightService`，普通切割的
  sink 生命周期和轮廓前二次安全门禁均通过 typed runtime 执行。
- runtime 外原始设备指针/锁入口已删除；架构扫描禁止重新引入
  `motionControl()`、`laserDevice()` 或 `lockDeviceAccess()` 业务调用。
- 连接/断开和状态轮询已分别迁入 `ProcessConnectionService`、
  `ProcessStatusService`；后者涵盖控制器、外设和安全监控生命周期调度。
- 工作流当前 schema 的文档所有权、文件读写与变更通知已迁入
  `ProcessWorkflowService`；流程树和 `MainWindow` 通过该服务契约接线，
  `ProcessModule` 不再直接解析或保存流程 TOML。
- CAD 新建、保存、关闭、STEP 导出及 STEP/IGES/STL/BREP 读取已迁入
  `CadDocumentIoService`；该服务的 detached import、文档级取消和原子提交仍待修复，
  不能将当前状态视为文档 IO 事务已收口。
- MainWindow 已将工作区、工程树和视图状态下沉为三个 controller；CAM 已
  抽出带 revision 校验的 `ToolpathGenerationService`；CAD 算法异常统一在
  module/service 边界记录和转换。
- 新增 `scripts/run_quality_gates.ps1`，统一空白检查、架构/旧格式扫描、配置、构建和 CTest 入口。
- 两条 ACS+GTN Debug 路线均成功生成 `x64/Debug/LaserCNC.exe`。
- 2026-08-03 复审时质量树 `build-cmake-quality/` 的 CTest 为 23/23 通过，其中
  `lcnc_simulator_cmhp_sdk_integration_test` 使用真实
  `acsc_OpenCommSimulator()` 与部署的 `Simulator.prg`，并验证 100 次设备
  线程会话命令和同线程断开/销毁。这不替代 ACS/GTN/真实激光物理硬件验证。
- 已实测该 SDK 的进程级 `CloseComm`→`OpenCommSimulator` 重启有多秒释放窗口，
  且连续数轮后会拒绝新句柄；因此“100 次进程级重连”仍是供应商 SDK 环境待验证项，
  未将其误记为自动化通过。
- 独立 Ninja 构建矩阵已验证 all-off（16/16 CTest）、ACS-only（17/17，含
  SimulatorCMHP SDK）与 GTN-only（16/16）。GTN-only 首次链接出现一次测试
  可执行文件短暂占用，确认无残留进程后重试通过；这不是编译或测试失败。
- real-laser 配置已构建并通过 17/17 CTest，ASan（ACS+GTN）构建并通过
  17/17 CTest。real-laser 仍暴露旧厂商协议适配器的 `/W4` 告警，故该配置
  尚未达到 `/WX` 收口；ASan 通过亦不替代 Application Verifier 和 8 小时资源趋势。
- Visual Studio/MSBuild ACS+GTN Debug 已构建并通过 17/17 CTest。首次冒烟
  发现共享应用输出未部署 `zlib1.dll`（`0xc0000135`），现已将该 DLL 绑定到
  `LaserCNC` POST_BUILD；修复后完整 CTest 通过。

完整问题、证据和发布判断见 [AUDIT.md](AUDIT.md)。

## 复审发现但尚未修复

- CAD worker 仍直接修改目标文档，STEP/IGES 仍缺 detached、generation-checked 原子提交。
- `MachiningFacePipelineService` 仍暴露 mutable entries，且尚缺双工作区 AIS offscreen 回归。
- 新增 Process service 文本尚未进入中文翻译 catalog。

这些问题的精确位置、影响和回归要求见 [AUDIT.md](AUDIT.md) 与 [todo.md](todo.md)。

## 仅静态或构建验证

- Process OCC-free、core/view 分层、淘汰 API、settings singleton、CMake 源文件收录。
- DeviceCommandQueue 与 ProcessDeviceRuntime 的线程边界；静态扫描确认业务层
  原始设备锁和设备指针调用为 0，`ProcessDeviceCoordinator` 仅留在 runtime 内部。
- 旧格式拒绝和设备队列 completion：`lcnc_project_package_test`、
  `lcnc_device_command_queue_test` 已通过。
- 预检 generation/取消/不可变报告与普通切割断连、状态读取、故障、电机未创建、
  轴失使能门禁分别由专用单元测试覆盖；本次证据不等同物理设备故障注入。
- connection/status service 的基本异步回调、幂等启停和停止后不再主动调度由独立
  CTest 覆盖；快速 stop/start 与旧 in-flight completion 尚未覆盖。
- real-laser 源码已构建并通过 CTest，但旧厂商协议适配器仍有项目 `/W4`
  告警，尚未达到该配置的 `/WX` 收口。

## 仍需外部环境/真机验证

- SimulatorCMHP 的真实 ACS SDK 100 次完整连接、建轴、使能、轮询、回零、
  加工、暂停、Stop、急停和断开循环；本轮已完成真实 SDK 初始化及 100 次
  设备线程会话命令，但没有用它冒充完整进程级重连验证。
- ACS、GTN 和真实激光器的物理硬件连接、回零、加工、暂停、Stop、急停和断开。
- 不可中断供应商调用的超时、对象保活与关机故障注入。
- Application Verifier/页堆、受控 ASan GUI 和 8 小时资源趋势。
- 大模型取消响应与 GUI 批提交性能门限。

实施顺序见 [todo.md](todo.md)，架构事实见 [ARCHITECTURE.md](ARCHITECTURE.md)。
