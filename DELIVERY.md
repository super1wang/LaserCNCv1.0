# LaserCNC 异步任务模型交付说明

交付复核日期：2026-07-23

## 结论

本轮完成了通用任务模型、Process 设备命令队列、独立工作流线程、耗时 CAM
命令后台化和连接态轮询生命周期收口。当前工作区已通过 ACS+GTN Debug 构建、
5 项 CTest、架构脚本与空白检查，可作为集成测试候选提交。

真机并发、供应商阻塞调用故障注入、受控 ASan GUI、Application Verifier 和
8 小时长稳仍是发布门禁；在这些验证完成前不标记为生产发布完成。

## 已完成范围

### 统一任务模型

- `TaskManager` 使用 `TaskSpec`、`TaskPriority`、`TaskExecutionStatus` 和
  `TaskSnapshot` 描述通用后台任务。
- 任务支持可见性、作用域、进度、步骤、协作取消、失败原因和有界等待。
- CAD/CAM 的一次性耗时工作继续使用通用线程池；Process 设备调用使用专用
  单线程执行策略，两者共享同一优先级语义。

### Process 设备执行策略

- `DeviceCommandQueue` 使用唯一设备线程串行执行已迁移的供应商调用。
- 优先级固定为 `Stop > Workflow > Interactive > Normal > Polling`。
- 控制器状态和外设轮询按 key 合并，避免定时器产生积压。
- 连接成功后启动轮询；断开、模块停止或连接失败后停止调度。
- 手动相对/绝对运动、连续点动、轴使能、IO 输出、设置下发、回零、加工预检、
  工作流步骤和安全停机均不再在 GUI 线程调用供应商 API。
- 断开仍作为带阶段进度与取消检查的一次性 `TaskManager` 生命周期任务执行，
  并受 `ProcessDeviceCoordinator` 串行租约保护。

### 独立工作流线程

- `ProcessWorkflowExecutor` 拥有独立、常驻的单线程池，不与 CAM、文件导入或
  通用任务池竞争。
- 工作流调用设备队列时使用仅次于 Stop 的 Workflow 优先级。
- 暂停和停止通过协作式 token 在步骤/轮廓边界生效；停止与急停均提交最高
  优先级安全停机命令。
- PureSimulation 的 `QTimer` 仅在其所属 Qt 线程操作，工作流线程通过阻塞式
  元对象调用跨越线程边界。

### CAM 耗时命令

- 全局刀路生成和当前轮廓重算使用
  “GUI 快照 → TaskManager 后台 OCC/IK → GUI 校验提交”。
- 后台阶段报告步骤和进度，并在工件、源形状、刀路版本或生成参数变化时丢弃
  陈旧结果。
- 轮廓/工件阶段与离散后的阶段边界检查取消；文档、XCAF、渲染和项目脏状态
  只在 GUI 线程提交。
- Process 工作流只读取 CAM 提供者缓存的 OCC-free 快照，不跨线程访问
  `CamModule`、XCAF 或视图对象。

### 生命周期与停机

- 模块停止先取消所属 TaskManager 任务，再停止轮询和工作流，随后关闭设备
  队列；只有所有借用线程退出后才销毁/断开设备运行时。
- 超时时保留可能仍被 SDK 使用的 Service 与控制器对象，不在活动调用下释放。
- 设备轮询只在连接态运行；断开前停止上层调度并执行安全输出复位。
- DeviceCommandQueue 析构和 ProcessWorkflowExecutor 析构均等待各自工作线程
  退出，避免后台线程越过所有者生命周期。

## 构建与验证

- 日常构建：`cmake --preset acs-gtn` +
  `cmake --build --preset acs-gtn-debug --parallel 16`
- 生成器：Ninja Multi-Config，唯一生成树为 `build/`
- Debug 运行目录：`x64/Debug`
- CMake 最低版本：3.21（链接器 launcher 从 3.21 起可用）
- 2026-07-23 复核：
  - ACS+GTN Debug 构建通过。
  - `ctest --test-dir build --build-config Debug --output-on-failure`：5/5 通过。
  - `scripts/check_architecture.ps1 -Root .`：通过。
  - Process OCC 扫描：0 命中。
  - `git diff --check`：通过，仅有预期的 LF/CRLF 提示。

## 未完成发布门禁

剩余事项及执行顺序以 `todo.md` 为准，核心风险包括：

1. ACS、GTN、SimulatorCMHP 的真机/模拟器并发与 Stop 优先级压力测试。
2. 供应商不可中断调用的超时、abort 和保留对象故障注入。
3. 工作流、PureSimulation 线程亲和、CAM 陈旧结果和取消延迟的自动化回归。
4. 受控 ASan GUI、Application Verifier/页堆与 8 小时资源趋势。
5. Process facade 与剩余 legacy/SDK 边界的后续结构收口。
