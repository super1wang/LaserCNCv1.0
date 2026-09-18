# B2.S1 实施记录

初版日期：2026-09-17；收口更新：2026-09-18。R1 已由用户确认通过。
初版已提交为 `0b6b3ae0177b550c657b69e0a02fe2e5c38007c1`。
首次收口 F01/F02 已提交为 `27a867413544a4323e366440a16a83af8fb3c3a8`。
二次审计 F03 的增量修复及验证记录见下文；历史提交与本次交付分开记录。

## 代码范围

- `PreparedDeviceProgram` 私有构造、只读发布：冻结 FinalMotionPlan、完整轴布局、controller context/qualification、按轮廓绑定的 `FrozenToolExecutionRecipe`、已提交 Process/IO/profile 配置、倍率、配方内容 revision 与 runEpoch。
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

## 初版验证（0b6b3ae）

- Ninja Debug 全量构建通过；定向回归 14/14，12.47 秒。
- Ninja ASan 全量构建通过；同组回归 14/14，20.74 秒，无 sanitizer 报告。
- `scripts/check_architecture.ps1` 与 `git diff --check` 通过。
- 回归：process_cutting_safety（含新增准备程序用例）、process_preflight_service、process_workflow、process_current_schema、device_command_queue、device_wait、command_list_start、confirmed_motion_stop、controller_recovery_sequence、group_motion_filter、process_device_shutdown_sequence、toolpath_snapshot_concurrency、cam_motion_plan_contract、project_package。
- 原始 CTest 证据：`build-cmake/Testing/Temporary/LastTest.log`、`build-cmake-asan/Testing/Temporary/LastTest.log`。

## F01：完整执行配方

- `frozen_tool_execution_fields.inc` 定义允许执行的有序字段集合；声明、Tool 捕获、canonical 编码和有限值检查共用此清单。`device-run-recipe-v2` 使用 `frozenToolExecutionRecipeHash()`，不依赖 `Tool::toTable()`。
- 身份包含工具名、来源、profile schema。已知配置单位：lineVelocity 为 mm/s，laserFrequency 为 kHz，pulseWidth 为 μs，快门延时为 s，平滑时间为 ms、平滑系数无量纲；其余旧配置的设备相关参数仍为 requested values，S2 必须资格确认其单位映射。
- 捕获拒绝非中性的轴置位/移动、路径延长、几何补偿、Z 联动、伺服高度、飞切、盲刻、打点、native arc/PD 特殊模式及其参数。S2 的工具入参只有显式 DTO；已提交配置副本中的 `Setting.Tool` 被移除，prepare 也拒绝该旧表旁路。
- 允许的数值逐字段检查 NaN/Inf；真实执行要求正的 line feed/acc/jerk。更改任意允许字段或工具/来源/schema 均改变 identity。合成 default 仍由 ToolFactory 的显式来源规则排除。

## F02：同一次运行内的暂停

- `PreparedExecutionSection` 按连续 CAM contour 分组，绑定 ordinal、contourId、首尾 block 下标/ID/hash、planHash、runEpoch；暂停边界必须已关光，拒绝非连续重复轮廓，不改动 block/knot/fence。
- `prepareExactSection` 仅编码当前 section；`startExactSection` 只能启动该 section；`isExactSectionRunning` 确认该 section 完成。旧的整程序 Start 不再用于 exact 路径。
- `executePreparedSections` 是 NormalCutting 与 fake-sink 测试共用的编排代码。每段启动前在工作流 checkpoint 等待，并在设备队列中再次检查 Pause/Stop；正在执行的段可完成，下一段等待 Resume。准备期间/Start 排队期间的 Pause 同样阻止启动。
- 同一 live run 始终使用同一 PreparedDeviceProgram、plan/context/recipe identity 和 runEpoch。只对尚未 Start 的 section 延迟准入；任何 Start 失败均返回，不重试。保留 checkpoint 的新调用拒绝恢复；失败后的 safe-stop/SDK 所有权仍由既有队列及 NormalCutting 清理路径处理。
- `consumeExactSection` 只遍历对应原始 blocks，entryBoundary 不作为重复指令发出。此合同不包含 S2 编码或 S3 Group 生命周期资格。

## 首次收口验证（27a8674）

- Ninja Debug 全量构建通过；11 项直接回归全部通过（8.04 秒），其中 process_cutting_safety 包含 F01 逐字段检查、原 S1 用例与 F02 六种并发运行场景。
- Ninja ASan 全量构建通过；同组回归 11/11（20.44 秒），无 sanitizer 报告。
- 架构检查与 `git diff --check` 通过。仅测试头命名空间整理后，补跑 Debug process_cutting_safety；未扩展 B1/B2 Stage Gate。
- 初版日志已被本轮同路径日志替换；上文初版数字为历史记录。

## F03：生产加载的圆弧兼容镜像

- `Tool::SetFromTable` 将 line acceleration/jerk 镜像至 arc 字段；冻结只接受精确相等的镜像，仍拒绝非零 arcVelocity、独立 arcAcc/arcJerk 及非有限值。
- 不新增 frozen arc 字段，不改变 DTO、identity schema 或 S2 接口；lineAcceleration/lineJerk 仍是唯一执行权威。
- 正向回归使用真实 TOML table → SetFromTable → ToolFactory 显式配方快照 → freeze → offline PreparedDeviceProgram。负向覆盖三个 arc 字段的独立值、NaN/Inf；保留 F01 全字段与 F02 六种暂停/无重放场景。

### 二次收口验证（2026-09-18）

- Ninja Debug 与 ASan 的 process_cutting_safety / process_current_schema 目标构建通过；定向 CTest 分别 2/2（3.76 秒）、2/2（7.96 秒），无 sanitizer 报告。
- 架构检查与 `git diff --check` 通过；本次未重复全量构建或 B2 Stage Gate。
- 原始日志仍为两棵生成树的 `Testing/Temporary/LastTest.log`，现记录本次定向测试；首次收口 11/11 为历史证据。

## 收口结论

F01 = CLOSED；F02 = CLOSED；F03 = CLOSED；C3 = CLOSED。
`B2.S1 = PASS`，`B2.S2_RELEASE = YES`（开发准入）。
本轮止于 S1 收口，S2/S3 尚未实施；真实 controller qualification 仍为 Unavailable/revision 0。
本次交付不执行提交或远端推送。
