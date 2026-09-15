# Repository Facts — super1wang/LaserCNCv1.0 @ 4254db59696193963392e2946132cf614c8def26

> 这些是规划使用的仓库事实。Terra 执行每个 WP 前必须在本地工作区重新验证；若 HEAD 不同但事实仍兼容，可继续；若公共合同/责任发生实质变化，触发 ESCALATE。

## FACT-01 Architecture Boundary

`ARCHITECTURE.md` / `AGENTS.md` 规定 CAM 是轮廓顺序、运动计划、物理轴坐标和碰撞验证快照的权威；Process 不得包含 OCC，消费 OCC-free snapshot。

## FACT-02 Current Export Contract

`src/modules/cam/contracts/toolpath_export_dto.h`

- `ToolpathExportPoint` 同时包含几何点、machine XYZ/R1/R2、layout axes、`tcpMcs`；
- `ToolpathExportSnapshot` 包含 contours、points、machine layout、solver identity、collision safety、travel plan、`CamMotionPlanSnapshot`。

## FACT-03 Current Motion Plan

`src/core/project/cam/collision_validation_contracts.h`

`CamMotionPlanSnapshot` 当前主要由：

- `QVector<CamMotionNode> nodes`；
- `edgeCertificates`；
- `collision`。

尚未表达 block-level primitive / motion class / optimization proof。

## FACT-04 Process Uses Committed CAM Snapshot

`ProcessToolpathService::refreshCommittedExecutionSnapshot()` 只读取 CAM 已提交结果，Process 无 API 重排/重解。

## FACT-05 Current Cutting Loop Still Uses Export Points

`NormalCuttingManager::executeContour()` 当前遍历 `row.data.points`，逐点调用 `commandSink.lineTo()`。

因此 v3 必须把“Final Motion Plan 真相”真正接到执行链，不能只新增一个无人消费的 DTO。

## FACT-06 Current RTCP Lowering

`GtnBufferedCommandSink::lineTo()`：

- RTCP active 时发送 `{tcpMcsX, tcpMcsY, tcpMcsZ, r1, r2}`；
- 同时用 CAM predicted physical axes 调 `ValidateGroupRtcpTarget()`；
- 然后进入 `GroupLineTo()`。

## FACT-07 Current Group Move

`GTNMotionControl::GroupLineTo()` 使用 `GTN_MoveLinearAbsolute`，`endVelocityMode=0`，并有 near-zero target filter。

## FACT-08 GTN LookAhead/Smoothing Already Exists

`InitFiveAxisGroup()` 已读取并设置：

- Group motion/orientation constraints；
- per-axis motion constraints；
- `fGroupSmoothTime` / `fGroupSmoothK`；
- `iGroupLookAheadNum` / `fGroupLookAheadTime` / `fGroupLookAheadRadiusRatio`；
- rotary velocity reference ratios。

`ProcessParameterRegistry` 已暴露对应设置。

## FACT-09 Current Group Lifecycle

`GtnBufferedCommandSink` 目前一个 program 结束后释放 Group；`resetProgram()` 也是 ownership boundary。

## FACT-10 Existing Rotary 4-Axis Solver

`toolpath_kinematics_solver.cpp` 已有 `RotaryTube4AxisSolver`，并已有 rotary equivalent-near continuity 逻辑。

因此 v3 不应把“4轴求解”当成全新能力从零重写；重点是统一 optimizer、物理降维验证和 execution lowering。

## FACT-11 Existing Full Table 5-Axis Solver

`Table5AxisSolver` 使用 continuous table IK 并把 relative linear axes 与 rotary 解组合为 `SolvedMachinePose`。

## FACT-12 RTCP Reference Has Explicit Semantics

`tableRtcpReferencePoint()` 明确：RTCP command TCP 是 table-zero machine reference point；去除 current workpiece carrier motion，保留 setup。它不是 posed machine-world TCP，也不是直接物理 XYZ。

所以降维必须分析 physical axes/forward kinematics，不能直接根据 `tcpMcs` 字段判断物理 XYZ 是否参与。

## FACT-13 Current Collision Gate Coupling

`process_cutting_safety.cpp::camExecutionBlockReason()` 当前对 machine package/job overlay 的检查受 safety flags 约束；但随后会**不依赖 Disabled/Required 三态策略**地检查 `motionPlan.collision.complete/Pending` 与 `collision.blocksExecution(...)`，并在真实机且 `safety.enabled` 时继续检查 edge certificates。

v3 `CollisionVerificationMode=Disabled` 必须把 collision-specific eligibility 从 plan/runtime readiness 中拆开，明确绕开 collision completeness/proof 门禁，而不是假造 safe proof；controller/device/axis/kinematics/plan completeness 等非碰撞门禁仍必须保留。

## FACT-14 Current Certificates Assume Linear Node Interpolation

`continuous_motion_certificate_builder.cpp::interpolateNode()` 对 `CamMotionNode.axes`、TCP、normal 做线性插值。未来 primitive/RTCP block 若改变插补语义，必须由统一 evaluator 驱动证书；不能继续默认所有段都是同一线性空间。

## FACT-15 Machine Kinematics Exists Independently of Geometry

`MachineKinematics` 保存 axis defs、parent topology、limits、WPC mounts、transforms，可在没有 machine shape 的条件下表达运动学。机台几何是碰撞/显示资产，不是 IK 数学本身。

## FACT-16 Safety Infrastructure Must Be Preserved

Process 已有单线程 `DeviceCommandQueue`、Stop 优先级、设备健康检查、GTN fault/release 语义。碰撞关闭不得绕开这些机制。

## FACT-17 Current Public Repository Baseline

Remote `main` 在规划生成时仍指向 `4254db59696193963392e2946132cf614c8def26`。


## FACT-18 Current Architecture Defines Production Collision as Fail-Closed

当前远端 `ARCHITECTURE.md` 明确把生产碰撞链定义为 `.lmsp/.lmsi + Job Overlay + residual geometry + continuous edge certificates`，并要求 Pending / Indeterminate / BoundaryUnknown / stale / missing / `complete=false` 阻断真实加工。

因此 v3 的 `CollisionVerificationMode::Disabled` 是**显式 commissioning 策略扩展**，不是对现有事实的重新解释。B0 必须在代码、配置、UI/日志和架构文档中同步落地；生产默认继续保持 `Required`，除非产品策略另行批准。

## FACT-19 Public Remote Build Surface Is Incomplete

本次远端 root snapshot 可见 `AGENTS.md`、`ARCHITECTURE.md`、`BUILD.md`、`src/`、`docs/` 等，但未暴露规划文档所描述的完整 root build/test/dependency surface（例如 root `CMakeLists.txt` / presets / tests / scripts / 3rd）。

因此规划审阅只能证明 source/contract/architecture compatibility，不能声明 public snapshot 已完成 C++ build、CTest、vendor SDK link 或 HIL。B0 必须以完整本地工作树重新建立 build baseline。

## FACT-20 Execution Recipe Must Be Frozen With Prepared Program

当前 NormalCuttingManager 的准备/执行路径仍会在运行准备阶段解析工具，并存在 sanitized/default fallback 语义。对于 v3 的 immutable FinalMotionPlan，这可能导致“同一 planHash、不同实际工艺值”。

因此 B2 必须把有效 tool/process/IO/profile values 冻结进 `PreparedDeviceProgram`（或等价单一准备结果），真实加工缺失/陈旧 recipe 必须阻断，而不是静默回退默认值。
