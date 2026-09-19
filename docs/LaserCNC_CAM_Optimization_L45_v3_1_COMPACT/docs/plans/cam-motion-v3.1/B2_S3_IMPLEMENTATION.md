# B2.S3 实施记录

基线：`58b627f`，用户确认 S2 审阅通过。范围为 Group / LookAhead / dynamics 生命周期；本次代码供人工 review，不自动提交或推送。

## Review 入口

- `runtime/gtn_exact_session.{h,cpp}`：同一状态机驱动 SDK adapter 和故障测试。顺序为 admission → acquire → 全运行编码 → 每批最多 32 条语义指令 → DataEnd → 一次性 Start → poll → 确认停止和释放。DataEnd pending 最多 256 次，每次返回现有队列；不增加设备线程。
- `device/motion_control/gtn_exact_backend.cpp`：冻结轴映射、运动学、轴/path/orientation 约束、dvMax、Smooth、LookAhead、参考轴比例及数字 IO。读取当前资格、编码器起点、轴/IO/标定身份；不同即拒绝。profile→encoder 同步、确认停止、反馈故障和显式 recovery 复用现有 GTN 实现。
- `runtime/gtn_lowering_profile.h`：新增资格绑定 Group profile；内容全部参与 capability fingerprint。requested/effective/source/unit/revision 记录在参数日志；可读参数逐项比对。SDK 无 LookAhead getter，明确记录 SDK-accepted/readback-unavailable，不冒充硬件读回。
- `runtime/gtn_buffered_command_sink.*`、`coordinated_motion_command_sink.*`、`exact_section_execution.*`：接通协调包装层转发；有限填表在既有 DeviceCommandQueue 中执行，Stop 可在批次之间运行；旧版 motion/IO 入口不得改写 exact session。
- Start 前重新 admission；任何不确定返回都安全输出、确认 Stop、尽可能释放并 latch，禁止 replay。释放失败保留轴归属，不在析构或 reset 中隐式重试恢复。安全输出在确认 Stop 前后各执行一次，防止运行列表再次开光。
- 各 section 完成后释放，再重新建立下一组；不跨段复用未验证的 Group。重复点、旋转圈数和 fence 保留，不调用旧 GroupLineTo 的过滤、Tool fallback 或 Process IK。

## 自动化证据与边界

`tests/gtn_exact_session_test.h` 并入 `lcnc_process_cutting_safety_test`，包含：两种模式正常生命周期、多段释放/重建、冻结后修改、最终 admission 失效、部分 acquire/append/DataEnd/poll 失败、Start 不确定、释放失败保留归属、禁止重放、DataEnd 有限重试、32 条填表边界 Stop、实际队列与协调包装层转发。测试资格仅存在于 fixture。

V-017～V-023 软件层映射：

| 验证项 | 本次证据 |
|---|---|
| V-017 exact plan / V-018 frozen recipe | 累计 prepared_device_program 与 session 测试；legacy 方法调用即断言失败 |
| V-019 RTCP / V-020 PhysicalAxes | 累计 lowering 测试；两种 Full5D 测试命令轨迹，保留重复点和多圈旋转值 |
| V-021 unsupported/no fallback | 累计模式、资格、cell、单位、映射负例；非中性未映射设备参数拒绝 |
| V-022 ownership/queue/recovery | 实际队列 Stop 插队、两段确认释放、失败保留 ownership；原 confirmed-stop/recovery/shutdown 回归 |
| V-023 indeterminate Start | fake backend Start 失败最多调用一次，safe-stop/latch，无自动 replay |

两种模式轨迹可在 `build-cmake/Testing/Temporary/LastTest.log` 与 ASan 对应日志中搜索 `S3 test-only`。这是主机语义指令轨迹，不是实际 SDK/设备执行 trace。

生产 `currentExactQualification()` 仍返回 Unavailable/revision 0，没有伪造 service/version；real admission 保持关闭。取得正式资格后还需验证 SDK/固件 IO、旋转圈数、连续插补、Group/LookAhead 实机行为。数字 IO/定时以外的非中性激光设备参数当前明确拒绝。GUI、控制器模拟器、物理机床本次均未运行；B2 实机 Stage Gate 和 R2 人工审阅仍待完成。

## 验证结果

- Ninja Debug 与 ASan：LaserCNC、相关目标构建通过。
- 定向 CTest 套件：process_cutting_safety、process_current_schema、device_command_queue、device_wait、command_list_start、confirmed_motion_stop、controller_recovery_sequence、process_device_shutdown_sequence、group_motion_filter、process_workflow。
- Debug 10/10（2.19 秒）；ASan 10/10（13.55 秒），无 sanitizer 报告。无全量 B1 重跑。
- 架构检查与 diff 空白检查通过。
- 新增多段 fixture 初次因缺少 canonical entry edge 被契约拒绝；补齐 entry/source/fence 后通过。测试断言改为 stderr，避免无人值守测试弹出 CRT 对话框。
