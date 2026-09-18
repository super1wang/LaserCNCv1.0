# B2.S1 实施记录

2026-09-17；基于用户确认 R1 审阅通过。仅实施 S1，未提交、未推送。

## 代码范围

- `PreparedDeviceProgram` 私有构造、只读发布：冻结 FinalMotionPlan、完整轴布局、controller context/qualification、按轮廓绑定的 Tool 值、已提交 Process/IO/profile 配置、倍率、配方内容 revision 与 runEpoch。
- `consumeExactPlan` 按 CAM block/knot/fence 次序交付；entryBoundary 不重复发点；-1 fence 在入边前交付；结构标记不隐式开关光/停车；取消或失败丢弃未封口程序。
- 真实 NormalCutting 入口改走准备程序；旧 points 路径只允许显式 PureSimulation。硬件 sink 默认拒绝 exact-plan 接口，不适配回旧点循环。
- 缺失/变更配方、Raw/stale plan、无效轴布局/运动参数、未解析补偿、范围截取与断点重放均拒绝。配置在 owner 线程捕获，不将 Tool 指针交给运行程序。
- ToolFactory 新增受锁保护的显式配方值快照；合成兼容 default 被排除，保存/恢复保留其非执行来源。修复 Tool 赋值遗漏平滑字段。
- 碰撞采用已发布 context 的策略；Disabled/Optional 不要求 proof；Required 检查绑定 planHash、完整安全证书与必要安全资源。

## 保留的边界

- ACS/GTN production qualification 仍 Unavailable/revision 0；测试中的 Qualified 仅为构造 fixture。
- S2 尚未实施：实际 controller lowering、设备当前资格/映射与起始位置 admission 必须在启用硬件 exact-plan sink 前完成。没有真实设备启动或机床合格结论。
- 显式 PureSimulation 保留兼容点回放，不算 exact-plan 仿真证据；精确仿真属于后续 B3。
- 新接口复用 DeviceCommandQueue；未改写已有 Stop/fault/recovery。S1 的无重放保护不等于已完成 S3 的全部 Group/pause 生命周期资格。
- S1 定向测试覆盖值冻结和消费 trace；不宣称完成 B2 Stage Gate V-017..V-023。

## 验证

- Ninja Debug 全量构建通过；定向回归 14/14，12.47 秒。
- Ninja ASan 全量构建通过；同组回归 14/14，20.74 秒，无 sanitizer 报告。
- `scripts/check_architecture.ps1` 与 `git diff --check` 通过。
- 回归：process_cutting_safety（含新增准备程序用例）、process_preflight_service、process_workflow、process_current_schema、device_command_queue、device_wait、command_list_start、confirmed_motion_stop、controller_recovery_sequence、group_motion_filter、process_device_shutdown_sequence、toolpath_snapshot_concurrency、cam_motion_plan_contract、project_package。
- 原始 CTest 证据：`build-cmake/Testing/Temporary/LastTest.log`、`build-cmake-asan/Testing/Temporary/LastTest.log`。
