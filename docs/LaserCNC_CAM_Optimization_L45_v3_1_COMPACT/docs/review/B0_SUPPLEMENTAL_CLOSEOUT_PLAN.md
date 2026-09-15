# LaserCNC CAM Motion v3.1 — B0 补充收口执行规划（R06/R07）

> 执行目标：关闭 B0 补充审阅发现的两个策略入口缺口
> 当前分支：`codex/cam-motion-v3.1-b0`
> 当前审阅 HEAD：`e2cbf7902e801e295fdd44305047a390e281a00c`
> 批次性质：**单提交级小型收口，不重做 B0**
> 正式验收次数：**1 次 targeted Stage Gate + 1 次 Astra R0 复审**

---

# 1. Goal

只关闭：

```text
R06 Optional travel path hidden gate
R07 Disabled automatic collision-backend preparation
```

修复完成后：

```text
R0 = PASS
B1_RELEASE = YES
```

本批次不得扩大到：

```text
B1 Full5D optimizer
B1 DOF reduction
B2 Process/GTN lowering
B3 HIL
B4 production collision qualification
```

---

# 2. Frozen Invariants

必须继续满足：

```text
INV-01 FinalMotionPlan is the single writable motion truth
INV-02 MotionClass / ControllerMotionMode / CollisionVerificationMode are orthogonal
INV-03 collision policy does not choose CAM trajectory
INV-04 Disabled has zero automatic collision backend construction/query
INV-05 Optional is diagnostic only
INV-06 Required remains fail-closed
INV-07 path validity remains independent from collision diagnostic validity
INV-08 real path/IK/limit errors still block all modes
INV-09 controller/fault/axis-enable/Stop remain unchanged
INV-10 no second DeviceCommandQueue / SDK bypass
```

---

# 3. C0.6 — Separate Travel Path Failure from Collision Diagnostic Failure

## Goal

保证 Optional collision diagnostics 不污染 `TravelPlanSnapshot::isPathReady()`。

---

## Repository Facts

当前 travel plan 同时存在：

```text
TravelPlanSnapshot::failureReason
TravelPlanSnapshot::collision.failureReason
```

但 `attachTravelPlan()` 在部分碰撞失败分支中会：

```text
plan.collision.failureReason = ...
plan.failureReason = plan.collision.failureReason
```

`isPathReady()` 又使用：

```text
failureReason.empty()
```

因此 collision diagnostic 可以伪装成 path failure。

---

## Required Behavior

建立明确 ownership：

### Path-level failure

仅用于：

```text
invalid / missing solved endpoint
IK failure
route construction failure
axis/coord representability failure
invalid transition semantics
```

写入：

```text
travelPlan.failureReason
```

所有模式均阻断。

### Collision-level failure

仅用于：

```text
collision config incomplete
machine package unavailable
overlay unavailable
diagnostic Unknown
collision diagnostic result
proof pending/stale
```

写入：

```text
travelPlan.collision.*
fullEnvironmentVerificationPending
collision-specific status
```

并由策略决定：

```text
Disabled:
  ignored / not scheduled

Optional:
  diagnostic only
  must not make isPathReady() false

Required:
  collision readiness/proof gates execution
```

---

## Implementation Sequence

1. 审计 `attachTravelPlan()` 中所有对 `plan.failureReason` 的写入；
2. 将“路径失败”和“碰撞失败”分类；
3. collision-only failure 不再写 path-level failure；
4. 检查：
   - cached travel restore
   - full environment verification result merge
   - async collision completion
   - UI/log projection
   是否会重新污染 `travelPlan.failureReason`；
5. 保持 `TravelPlanSnapshot::isPathReady()` 只表达运动路径是否可执行；
6. Required collision gate 继续从 collision state/proof 判断。

---

## Forbidden Solutions

```text
- Optional 时统一 clear failureReason
- Disabled 时统一 force path ready
- 将 IK/path failure 移到 collision.failureReason
- 修改 isPathReady() 让所有 failureReason 都不阻断
- Process 自己重新判断哪种 failure 是 collision
```

责任必须在 CAM 生成/快照层已经分清。

---

## Affected Surface

优先：

```text
src/modules/cam/toolpath/cam_module_toolpath.cpp
src/core/project/cam/travel_plan_contracts.h
src/modules/cam/integration/cam_service_adapters.cpp   // if merge semantics affected
tests/*travel*
tests/*process*
tests/*no_model*
```

如无需修改 header，不要为了形式新增 API。

---

## Tests

### Positive

```text
two valid contours
valid solved coordinates
Optional
collision package/overlay missing
=> travelPlan.isPathReady() == true
=> Process collision-wise not blocked
```

### Required negative

同一路径：

```text
Required
collision package/overlay missing
=> execution blocked
```

### Path negative

```text
Disabled + invalid endpoint/IK
=> path not ready

Optional + invalid endpoint/IK
=> path not ready
```

### Regression

```text
existing no-model CAM flow
existing travel path regression
existing Process cutting safety
```

---

## Done Condition

```text
Optional collision diagnostic can never change path-ready state
unless it also exposed a real non-collision path error.

Required fail-closed unchanged.
```

---

# 4. C0.7 — Disabled Must Not Auto-Prepare Collision Backend

## Goal

确保 Disabled 是真正的：

```text
zero automatic collision backend construction/query
```

不仅是 certificate builder 零查询。

---

## Repository Facts

当前正常 travel rebuild 会：

```text
if verification pending:
    scheduleFullEnvironmentVerification()
else:
    scheduleCollisionSafetyDomainPreparation()
```

`scheduleCollisionSafetyDomainPreparation()` 当前可继续：

```text
buildCollisionGeometry
buildCoalCollisionPairs
Job Overlay build
```

即使 effective mode 为 Disabled，只要已有机台安全资源。

---

## Required Behavior

### Automatic scheduling rule

在所有自动 collision preparation/verification 调度入口：

```text
CollisionVerificationMode::Disabled
  -> return before creating task
```

至少覆盖：

```text
scheduleCollisionSafetyDomainPreparation
scheduleFullEnvironmentVerification
continuous certificate scheduling entry
automatic overlay preparation triggered by motion regeneration
```

已经有 certificate builder 的 Disabled short-circuit 可以保留作为第二层防护，但不能依赖它阻止更早的 geometry/backend preparation。

### Policy transitions

从 Optional/Required 切到 Disabled 时：

```text
pending collision preparation task:
  abort if supported

late completion:
  generation/policy check prevents publish into new Disabled state
```

不要求本批次删除已有 cache 文件或清空所有旧诊断数据。

核心要求是：

```text
Disabled 下不再启动新的自动 collision work
且旧工作不能改变新的 committed motion eligibility
```

---

## Important Boundary

不得误伤 CAM 自身正常 OCC 使用。

以下仍允许：

```text
OCC source geometry
primitive recovery
toolpath geometry
kinematics
workpiece transforms
preview
```

禁止的是：

```text
collision geometry build
Job Overlay automatic build
Coal collision pair build
collision certificate/proof automatic build
```

---

## Implementation Sequence

1. 找出所有自动碰撞任务调度入口；
2. 将 `effectiveVerificationMode` / frozen policy 作为第一层条件；
3. Disabled 提前返回；
4. 审计 mode change 时已有 task 的取消/代际保护；
5. 不新建新的 scheduler / task system；
6. 复用现有 TaskManager cancellation / generation guard。

---

## Forbidden Solutions

```text
- 只在 buildContinuousMotionCertificates() return
  但前面仍构造 collision geometry
- 通过 “machine package not ready” 间接避免调度
- Disabled 时仍后台预热 collision，只是不让 Process 看
- 新增第二套 collision scheduler
```

---

## Affected Surface

优先：

```text
src/modules/cam/toolpath/cam_module_toolpath.cpp
src/modules/cam/collision/cam_module_collision.cpp
existing collision task/cancel helpers
tests/cam_motion_foundation_test.cpp
relevant no-model / collision scheduling tests
```

---

## Tests

必须新增一个**机台资源存在**的 Disabled 场景：

```text
mode = Disabled
machine package ready
machine geometry available
overlay missing or stale
toolpath/travel rebuild
```

断言：

```text
collision prep task scheduled == 0
collision geometry build calls == 0
Coal pair build calls == 0
certificate work == 0
```

再验证：

```text
Optional:
  diagnostic preparation may occur
  but cannot block valid path

Required:
  preparation/proof remains required
```

---

## Done Condition

```text
Disabled motion generation has zero automatic collision work,
regardless of whether machine collision resources happen to exist.
```

---

# 5. One Targeted Stage Gate

C0.6 + C0.7 一起完成后，只进行一次 Gate。

建议矩阵：

```text
G1 Optional + valid multi-contour + missing collision env
G2 Required + same missing collision env
G3 Disabled + existing machine/package + stale overlay
G4 Disabled + true path failure
G5 Optional + true path failure
G6 existing no-model CAM flow
G7 existing Process cutting safety
G8 directly impacted travel/collision foundation tests
```

Exit：

```text
[ ] G1 PASS
[ ] G2 PASS
[ ] G3 PASS
[ ] G4 PASS
[ ] G5 PASS
[ ] G6 PASS
[ ] G7 PASS
[ ] G8 PASS

[ ] Optional does not poison travel path readiness
[ ] Disabled schedules zero automatic collision work
[ ] Required still fail-closed
[ ] real path failure still blocks
[ ] no new motion truth
[ ] no new device queue
[ ] no unresolved ESCALATE
```

---

# 6. Status / Evidence Update

当前仓库已经提前记录：

```text
B0 = Astra PASS
R0 PASS
B1 Released
```

在补充审阅关闭前，应改为事实状态：

```text
B0 = Stage Gate Passed / Supplemental Patch
R0 = PASS_WITH_PATCH
B1 = Blocked pending R06/R07
```

修复完成并通过本 Gate 后，再更新为：

```text
B0 = Astra PASS
R0 = PASS
B1 = Released
```

不要由执行模型自行宣布 Astra PASS。

应由 Astra/本阶段 review 明确给出。

---

# 7. Compact Completion Report

执行模型完成后只输出：

```text
HEAD / commit
R06 changed area
R07 changed area
G1..G8 result
remaining known deferred optimization
Astra R0 review request
```

不要重复完整 B0 规划。

---

# 8. Astra R0 Supplemental Review Prompt

```text
审阅 codex/cam-motion-v3.1-b0 最新 HEAD。

这是 B0 R0 的补充收口复审。
上一轮 F01–F05 已接受，本轮只确认 R06/R07 以及其直接回归。

必须检查：

R06
- TravelPlan path failure 与 collision diagnostic failure 是否真正分离；
- Optional 在多轮廓 travel 中即使碰撞资源缺失/Unknown，也不会让有效路径 isPathReady=false；
- Required 仍因相同 collision failure fail-closed；
- 真正的 IK/path/endpoint failure 在 Disabled/Optional 下仍阻断。

R07
- Disabled 即使已经加载 machine geometry / package，也不会启动自动 Overlay、collision geometry、Coal pair、certificate/proof work；
- Optional/Required 行为没有被误伤；
- mode transition 的 stale task 不会回写新的 Disabled committed state。

同时确认：
- FinalMotionPlan single truth unchanged；
- no second DeviceCommandQueue；
- no Process replanning；
- B1 可以在不重新设计 B0 contract 的情况下开始。

输出：
PASS
PASS_WITH_PATCH
BLOCK

若 PASS，明确写：

R0 = PASS
B1_RELEASE = YES

不要要求 B1 optimizer、GTN 实机、HIL 或 B4 production qualification 来阻塞本次补充收口。
```

---

# 9. Release Rule

本文件通过后仅表示：

```text
B0/R0 complete
B1 CAM Motion Compiler development released
```

不表示：

```text
real-machine qualification
laser process qualification
production collision-safe release
```

这些仍属于后续 B3/B4。
