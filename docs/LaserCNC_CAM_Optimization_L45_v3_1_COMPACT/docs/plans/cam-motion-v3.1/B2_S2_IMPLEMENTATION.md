# B2.S2 实施记录

基线：`398b4e0`，B2.S1 二次收口由用户确认审阅通过。本次范围为 S2 编码及软件检查点，S3 Group/LookAhead 生命周期另行实施。

## 代码

- `GtnLoweringProfile` 冻结 Group 槽位→physical-layout index/name/role→GTN controller axis、物理限位、模式、资格来源/revision、逐 MotionClass 连续插补/hold/feed 能力、跨圈语义及进给参考轴/比例。默认 Unavailable/revision 0，无生产资格来源时不生成有效编码。
- profile 内容 hash 必须等于已发布 controller qualification 的 capabilityFingerprint，来源/revision/mode 及完整 qualification hash 同时匹配。`DeviceRunRecipe` 升级 `device-run-recipe-v3` 并包含此 profile；编译后不读取可变 UI/Tool 配方。
- `GtnEncodedProgram` 先检查并编码整份有限运行，失败不返回部分结果；按原 section 保存只读 `GtnEncodedSection`。PhysicalAxes 精确映射物理轴，RTCP 仅使用 selected knot 的 table-zero reference TCP 与原始旋转轴坐标。不做 IK、归一化、近零过滤、重采样或候选切换。
- 保留每个 knot（含重复值）、entry fence、激光状态变更及 requiredStop。entryBoundary 用于校验，不重复发点。userTag 在整份运行内唯一，可追溯 block/hash/knot/contour/source/departure identity；encodingHash 绑定 plan/context/recipe/runEpoch/profile 与指令内容。
- 支持显式 XYZ mm、带资格比例的混合 mm 度量、单旋转轴表面进给到 deg/s 的半径转换。nominal mm/min 显式除以 60；倍率只作用于速度；rapid 使用独立冻结速度及 rapid acceleration/jerk，缺失即拒绝。混合轴无度量、零参考距离、非有限值、越限、未资格 cell 均拒绝。
- GTN sink 的 `prepareExactSection` 验证映射与运行身份，缓存整份不可变主机编码并选择当前 section；Stop 拒绝准备。RTCP 调用已有 `ValidateGroupRtcpTarget`，每个目标与 entry boundary 都检查；端点成功不替代 continuous-interpolation qualification。

## 检查与边界

- 新回归并入 `lcnc_process_cutting_safety_test`：乱序五轴映射、PhysicalAxes/RTCP trace、跨 ±360° 多圈原值、重复点、fence/section/tag、冻结后修改、资格/映射/限位/单位负例、整份运行失败原子性及无 legacy Start fallback；原 S1 全字段/暂停测试保留。
- SDK 的 `TGroupMoveParameter.orientationDir` 默认描述短路径，因此不能凭数值未修改推断圈数语义；独立 `absoluteRotaryTurnsQualified` 默认关闭。测试资格仅存在于 fixture。
- 这里的封口是主机编码封口，尚非 SDK CommandListDataEnd。`startExactSection` 明确拒绝，S3 负责当前控制器资格/位置/Group 状态 admission、冻结 profile/IO 的 SDK 映射与读回、finite-list 提交及 Start/Stop/release/no-replay 生命周期。不得借用旧 `GroupLineTo` 的过滤/Tool fallback 或旧 `startProgram`。
- 当前生产 controller 仍 Unavailable/revision 0；没有资格服务、SDK 实机验证或硬件启动。native circle/cylinder 无编码路径。保持现有 DeviceCommandQueue 单一设备入口。
- B2 Stage Gate V-017..V-023 未执行；本次产物供人工 review，不代表整批 B2 或机床放行。

## 验证

- Ninja Debug：LaserCNC 与相关测试构建通过；定向 CTest 5/5（11.25 秒）。
- 套件：process_cutting_safety、process_current_schema、device_command_queue、command_list_start、group_motion_filter。
- Ninja ASan：LaserCNC 与同组测试构建通过；定向 CTest 5/5（9.73 秒），无 sanitizer 报告。
- 架构检查与 `git diff --check` 通过。原始日志为两棵生成树的 `Testing/Temporary/LastTest.log`。
- S2 软件编码检查点通过，已记录在本地提交；硬件提交仍由上述 S3/资格边界关闭。
