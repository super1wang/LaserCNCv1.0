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
- B1 在 Astra R0 给出 `PASS` 或关闭 `PASS_WITH_PATCH` 前不开始。
