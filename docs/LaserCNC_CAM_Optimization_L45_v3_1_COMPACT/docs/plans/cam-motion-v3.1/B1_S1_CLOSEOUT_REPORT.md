# B1.S1 收口报告（v3.2.1）

日期：2026-09-16。范围仅限 S1 的编译输入冻结、源几何采样和旧求解失效；未实现 S2 优化、控制器资格认证或实体机放行。

## 结论

> 2026-09-17 复审更正：下述为 3722c12 的历史报告，不再作为放行依据。复审判定 PASS_WITH_PATCH：计算前冻结、最终区间角度证明、源覆盖/屏障完整性及硬预算仍需修复。修复与重新验证结果见 `B1_S1_REVIEW_FIX_REPORT.md`。

**本报告历史结论已撤回；最新修补及验证结论以 `B1_S1_REVIEW_FIX_REPORT.md` 为准。**

ACS/GTN 尚无 ControllerMotionMode qualification service 或记录。生产捕获明确使用 `Unavailable`、revision `0`、空 capability fingerprint；历史 `PhysicalAxes` 仅是 requested value。该状态进入确定性 context hash；不将 SDK、轴名/角色、编译或普通测试当作资格。RTCP motion plan finalization 和 evaluator binding 均拒绝未资格快照。qualification-dependent candidate admission 仍关闭。

## C1：冻结来源

- 复用 CAM owner 的 `ToolpathGenerationStamp`，加入 workspace generation、源刀路 revision、轮廓顺序、已求解机器配置 fingerprint（其配置序列包含拓扑、软限位及运动参数）、setup revision、tool/process、采样/优化策略来源、controller qualification snapshot、dynamics semantics、collision mode 与 interpolation model version。
- `MotionCompilationInput` 在发布路径持有上述上下文和参数 requested/effective/source/unit/revision/available；结果接纳前重新比较 owner generation 与 context identity，失败或取消均拒绝。
- 没有新增控制器资格权威，也没有把未资格值转成 qualified effective mode。工艺硬屏障来源当前为空并显式标记 unavailable；真实工艺来源接入仍属于后续工作。

## C2/C3：几何与旧解

- OCC deflection seed 继续保留，源边/源参数上补弦偏差、切向角和表面法向角的区间细分；深度、最小参数跨度及采样倍率有界。预算不足标记 `geometrySamplingComplete=false`，不放宽容差，也不允许权威求解。
- 源边 seam、lead-in anchor 与显式 process hard barrier 均作为采样结点保留；硬屏障贯穿导出节点、运动块身份和项目包回读。闭合、反向、裁剪边有直接断言。
- 几何序列替换、起点重排和轮廓遍历重排清除旧 `MachineCoord`/`SolvedMachinePose`；旧项目无采样完成标记时保留显示点，但必须重采样。

## 验证

- `acs-gtn-debug` Ninja 全目标构建通过。一次无运行进程的 `LNK1104` 重试后通过；未清理生成树。
- 完整 CTest：**52/52 通过**（含架构、项目包、CAM 流程、运动计划、Process 回归）。
- 增补后的 `lcnc_cam_lead_in_test` 单独重建并通过：strict duplicate、same-position seam、closed order、reversed parameter、trimmed line/arc、曲线弦偏差、anchor、tangent/normal refinement、process barrier、budget exhaustion、旧解失效。
- `lcnc_cam_motion_plan_contract_test` 覆盖来源身份及未资格 RTCP 拒绝；`lcnc_cam_algorithm_pipeline_test` 覆盖 mutation-after-capture/stale/cancel；`lcnc_project_package_test` 覆盖 incomplete 与 hard barrier 回读。
- `git diff --check` 通过。软件测试不等于 ACS/GTN qualification、SDK/仿真器语义认证、碰撞生产资格或实体机验收。

## S2 边界

允许开展 authoritative IK、Raw/Optimized Full5D、rotary unwrap、numerical denoise 等 controller-independent 工作。任何依赖 ControllerMotionMode qualification 的 RTCP exact semantics、candidate admission 或控制器实际运动均继续拒绝，直至存在正式、版本化的资格来源。
