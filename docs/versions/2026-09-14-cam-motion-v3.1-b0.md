# CAM Motion v3.1 B0 — 基础合同与连续运动求值器

## 交付范围

- 基线：本地 `main@815ceb8`；规划基线为 `LaserCNCv1.0@4254db5`。用户指定的 `LaserCNCv1.1` 远端在 2026-09-14 查询成功但没有可见引用，因此本轮以完整本地工作区为事实基线，没有覆盖或回退本地历史。
- 实现提交：`6b11aeb feat(cam): 完成运动重构B0基础合同`。
- 当前分支：`codex/cam-motion-v3.1-b0`。

## 已实现

1. 冻结 `MotionClass`、`ControllerMotionMode`、`CollisionVerificationMode` 三个正交维度。旧 `collisionDetectionEnabled=true` 配置确定迁移为 `Required`，新格式持久化 `collisionVerificationMode`。
2. 将现有 `CamMotionPlanSnapshot` 扩展为唯一 FinalMotionPlan：块内保存物理轴 knot、世界 TCP、RTCP MCS reference TCP、source span、process fence、feed/tolerance 语义；上下文、块和计划使用确定性 SHA-256 身份。旧 `nodes` 只由块派生，并绑定 `derivedFromPlanHash`。
3. 新增碰撞无关的 `ContinuousMotionEvaluator`。PhysicalAxes 先插补物理轴再调用运动学；RTCP 必须提供经资格确认的模型；缺少保守区间 bound 时拒绝用端点猜测连续等价。
4. Process 将路径/运行准备与碰撞资格拆分：Disabled/Optional 不因缺少机台 STEP、`.lmsi`、Job Overlay、Pending 或证书而阻断；Required 保持失败关闭。控制器连接、状态、故障、轴使能、运动学、限位与 Stop 门禁未被削弱。
5. Disabled 证书路径在创建 worker/几何上下文前直接返回 `Disabled` 证明状态，查询计数保持 0；UI 与 Process 日志明确标记未获碰撞安全认证。
6. 异步碰撞证明只允许附加到相同 revision、`contextHash` 和 `planHash`，不匹配时替换完整快照而不修改旧运动块。

## 构建与 Stage Gate

- 配置：Qt 6.9.1、OpenCASCADE 8.0.1、ACS=ON、GTN=ON、Coal=ON、BDAQ=OFF、real-laser=OFF。
- `acs-gtn-debug` Ninja 全目标构建通过；一次 `lcnc_process_workflow_test.exe` LNK1104 瞬时占用在确认无残留进程后原目标重试通过。
- B0/直接影响矩阵：Process preflight、Process cutting safety、travel path、无模型 CAM flow、workpiece collision flow、machine collision contract、Coal backend、FinalMotionPlan contract、foundation/config/backend-zero、snapshot concurrency、architecture、translation 均通过。
- 机台碰撞测试首次因旧夹具未声明新策略而失败；夹具显式设为 `Required` 后同一用例重跑通过，未更改生产阈值或把 Pending 伪装为安全。
- 最终核心复跑 5/5；无模型 CAM flow 1/1；机台碰撞 contract 1/1。

## 未覆盖与下一门禁

- 本轮没有执行 VS/Release/ASan 全矩阵、GUI 人工检查、GTN/ACS 实机运动、激光、HIL、长稳或生产碰撞验收。
- B0 只建立合同与未优化的 exact-knot block；Full5D 优化、重采样、5D merge、DOF reduction、候选选择和原子后台编译属于 B1。
- B1 已由本次 B0 收口复审放行，但尚未开始实现。

## R0 `PASS_WITH_PATCH` 收口

- F01：后续语义 Block 通过 `entryBoundary` 唯一拥有跨 Block 的 canonical edge；该边界不进入 legacy `nodes` 投影。`sourceSpans` 与 start fence 使用已定义的 `-1` entry 索引，Block/Plan v2 哈希覆盖边界、来源与 fence 语义。
- F02：`ContinuousMotionEvaluator` 只接受由 finalized plan 一次性生成的 `BoundMotionEvaluationContext`；绑定校验 plan/context/model identity，求值拒绝非成员或已变更 Block，bound 拒绝非有限、逆序和负误差区间。
- F03：initial approach 的 Disabled 路径不调用碰撞 certifier，也不再要求无实际用途的 workpiece shape；Optional 保留首个运动学合法候选并仅附加诊断，碰撞/Unknown 不改变候选。
- F04：Required 的 Process 与 initial-approach 门禁显式只接受 `CertifiedSafe`，`Disabled`、`Invalid`、`BoundaryUnknown`、`Blocked` 均不能取得 Required 执行资格。
- F05：仅在新字段缺失时执行 legacy bool 迁移；显式非法或空 `collisionVerificationMode` 保留 invalid 状态并按 Required 方向失败关闭，不再静默降级 Disabled。
- B1 handoff：当前 `workspaceGeneration`、`setupCalibrationHash`、controller capability、tool/process 与 dynamics 字段仍是 B0 transition/baseline identity；B1 Workstream 7 必须从真实 immutable compilation context 捕获，非空 hash 不代表控制器能力已获 qualification。

收口验证：`acs-gtn-debug` 下 Debug `LaserCNC` 与直接影响目标构建通过；B0-CLOSE 定向矩阵 12/12 通过，包括 architecture、translation、motion plan、foundation/config、Process cutting/workflow、travel path、no-model CAM flow、workpiece collision 与 machine collision contract。

```text
R0 = PASS
B1_RELEASE = YES
```

本结论只放行 B1 开发，不代表 GUI、GTN/ACS 实机、激光、HIL、长稳或 B4 生产碰撞资格通过。
