# LaserCNC CAM Motion v3.1 — B0 收口执行规划

> 规划类型：R0 `PASS_WITH_PATCH` 收口
> 目标分支：`codex/cam-motion-v3.1-b0`
> 审计 HEAD：`fcc3d173dc82faa8ae7d60d7ea46593cc1615081`
> 执行策略：**单一收口批次、最少验收、集中开发、一次 Stage Gate**
> 完成后：Astra R0 复审，通过即放行 B1
> 不包含：B1 Full5D optimizer、B2 Process/GTN cutover、B3 HIL、B4 production collision qualification

---

## 1. Goal

用一个紧凑收口批次关闭 B0 审计发现的 5 个 contract gap，使 B1 可以安全依赖：

```text
FinalMotionPlan
ContinuousMotionEvaluator
CollisionVerificationMode semantics
```

不得重新设计 B0 主架构，不得扩大为新一轮完整重构。

---

## 2. Execution Mode

执行模型采用连续开发模式：

```text
实现 C0.1 + C0.2
  -> Soft Check S0

继续实现 C0.3 + C0.4 + C0.5
  -> 一次 B0-CLOSE Stage Gate

通过
  -> 更新状态/证据
  -> Astra R0 Review
  -> PASS 后进入 B1
```

禁止每个小修改单独生成长报告。

正常输出只需要：

```text
changed files
important semantic decision
targeted test result
blocker if any
```

---

# 3. Frozen Invariants

整个收口过程中必须保持：

```text
INV-01 FinalMotionPlan 仍是唯一可写 motion truth
INV-02 MotionClass / ControllerMotionMode / CollisionVerificationMode 正交
INV-03 Disabled 不需要 machine geometry/package/overlay/certificate
INV-04 Optional 只诊断，不改变 path selection / execution eligibility
INV-05 Required fail-closed
INV-06 controller/fault/axis-enable/limits/Stop 不受 collision mode 削弱
INV-07 evaluator 不依赖 collision backend
INV-08 Process/consumer 不重新 IK / replan / simplify
INV-09 不新增第二 SDK queue
INV-10 B1 之前 canonical continuous motion edge coverage 必须闭合
```

---

# 4. C0.1 — 冻结 Canonical Edge Ownership / Block Boundary

## Goal

修复跨 `CamMotionBlock` 阶段边界丢失连续求值语义的问题。

## Inputs

```text
CamMotionPlanSnapshot
CamMotionBlock
CamMotionNode
MotionProcessFence
ContinuousMotionEvaluator
attachMotionPlan()
finalizeMotionPlan()
```

## Repository Facts

当前 `attachMotionPlan()` 在 phase/contour/rapidPhase 变化时新建 block；新 block 不自动包含上一 block 终点。

当前 evaluator 只使用当前 block 的 knots。

## Required Behavior

必须定义：

```text
每条 canonical motion edge
  belongs-to exactly one evaluable interval
```

至少覆盖：

```text
LeadIn -> Cutting
Cutting -> Rapid
Rapid phase -> Rapid phase
Rapid -> LeadIn/Cutting
```

Block boundary 同时必须保留：

```text
phase
process fence
laser semantics
source span
hash identity
```

## Implementation Decision

执行模型可在以下两类实现中选择最小兼容方案：

### Preferred A — Explicit entry boundary

Block 增加不会被 consumer 当成第二 execution node 的 entry boundary / predecessor state。

优点：

- 无重复 projection node；
- edge ownership 明确；
- process phase 与 edge 语义更清楚。

### Allowed B — Shared boundary knot

相邻 block 共享一个 canonical boundary knot。

但必须同时解决：

- legacy node projection 去重；
- fence 语义；
- phase metadata；
- hash；
- edge count。

若两种方案都与现有 contract 冲突，触发 ESCALATE，不得让 optimizer 自己猜 predecessor。

## Affected Surface

优先：

```text
src/core/project/cam/collision_validation_contracts.h
src/core/project/cam/final_motion_plan.cpp
src/core/algorithms/cam/continuous_motion_evaluator.*
src/modules/cam/toolpath/cam_module_toolpath.cpp
tests/cam_motion_plan_contract_test.cpp
```

## Forbidden

```text
- evaluator 偷读 global previous node
- optimizer 回退到 pointsByContourId 补边
- Process 以后再修
- 一个 edge 同时属于两个 block
- 用 duplicated legacy node 形成第二 motion truth
```

## Tests

新增 canonical edge coverage test：

```text
sequence:
LeadIn P0
Cutting P1
Cutting P2
Retract P3
Traverse P4
Approach P5
Cutting P6
```

断言：

```text
source edge count == covered edge count
every edge covered once
no duplicate coverage
phase/fence boundaries stable
plan identity changes when boundary semantics change
```

## Done

C0.1 完成条件：

```text
edge coverage = exact
contract test PASS
no consumer inference required
```

---

# 5. C0.2 — Harden ContinuousMotionEvaluator Contract

## Goal

让 B1 可以安全使用 evaluator 做 adaptive resampling / merge / equivalence。

## Required Behavior

### Model binding

建立一次性绑定语义：

```text
plan interpolationModelVersion
==
evaluation context model version
```

建议采用：

```text
bindEvaluationContext(plan/context, callbacks)
  -> validated immutable BoundMotionEvaluationContext
```

单点 `evaluate()` 不重复做全 plan hash。

### Bound validation

`bound()` 必须拒绝：

```text
NaN / Inf
min > max
negative error
invalid u
unsupported interpolation
model mismatch
callback says valid but structure invalid
```

应验证：

```text
all physical axis bounds finite
all TCP bounds finite
min <= max
position/orientation error finite and >= 0
```

### Preserve current semantics

PhysicalAxes：

```text
interpolate physical axes
-> derive kinematics/TCP
```

禁止：

```text
interpolate physical axes independently
AND interpolate world TCP independently
```

RTCP：

```text
only qualified callback/model
```

## Affected Surface

```text
src/core/algorithms/cam/continuous_motion_evaluator.*
src/core/project/cam/collision_validation_contracts.h  // only if identity field/API needed
tests/cam_motion_plan_contract_test.cpp
```

## Forbidden

```text
- 每 evaluate() 重算完整 plan hash
- missing bound 时用 endpoints 猜
- 自动 clamp 非法 bound 伪装成合法
```

## Tests

```text
model mismatch -> fail
NaN min/max -> fail
inverted interval -> fail
negative error -> fail
u outside [0,1] -> fail
missing bound callback -> fail
existing nonlinear midpoint test -> pass
valid physical-axis bound -> pass
```

---

# Soft Check S0

C0.1 + C0.2 完成后，只跑：

```text
cam_motion_plan_contract_test
与 evaluator/final-motion-plan 直接关联的 build target
```

如果通过，直接继续 C0.3。

失败才停下来诊断。

不需要 Astra，不需要全量 regression，不需要阶段报告。

---

# 6. C0.3 — Optional Initial Approach Must Be Diagnostic Only

## Goal

消除 Optional 对 initial approach candidate selection 的隐藏影响。

## Required Behavior

将 initial approach 分为：

```text
A. motion candidate validity
   kinematics
   axis limits
   target agreement
   route semantics

B. collision result
   Disabled: none
   Optional: diagnostic
   Required: gate
```

策略：

```text
Disabled:
  accept first motion-valid candidate
  no collision certification dependency

Optional:
  accept same motion candidate as non-collision logic
  attach diagnostic if available
  collision/unknown must not select another candidate

Required:
  candidate must obtain Required proof
  fail closed otherwise
```

## Affected Surface

```text
src/modules/cam/toolpath/cam_module_toolpath.cpp
src/modules/cam/contracts/i_cam_initial_approach_planner.h
tests/cam_motion_plan_contract_test.cpp
and/or focused initial-approach test
```

## Forbidden

```text
- Optional Collision -> search a different safety-Z
- Optional Unknown -> fail the path
- fake Safe
- skipping kinematic/limit checks
```

## Tests

对完全相同的 initial approach inputs：

```text
Disabled -> candidate A
Optional + diagnostic Safe -> candidate A
Optional + diagnostic Collision -> candidate A
Optional + diagnostic Unknown -> candidate A
Required + Safe -> candidate A
Required + Collision/Unknown -> blocked
```

只比较 path candidate identity/waypoints，不要求本次 HIL。

---

# 7. C0.4 — Required Certificate Eligibility Must Be Strict

## Goal

确保 Required 不接受 `Disabled` certificate。

## Required Behavior

冻结：

```text
Required edge eligibility:
state == CertifiedSafe
```

不要复用“Disabled 也可执行”的 policy-agnostic helper 作为 Required 判断。

建议 API：

```text
certificate.executionEligibleFor(mode)
```

或者在 Required gate 显式比较 `CertifiedSafe`。

选择最小影响方案即可。

## Affected Surface

```text
src/core/project/cam/collision_validation_contracts.h
src/modules/process/runtime/process_cutting_safety.cpp
src/modules/cam/contracts/i_cam_initial_approach_planner.h
tests/process_cutting_safety_test.cpp
tests/cam_motion_plan_contract_test.cpp
```

## Tests

```text
Required + Disabled cert -> block
Required + Invalid -> block
Required + BoundaryUnknown -> block
Required + Blocked -> block
Required + all CertifiedSafe -> pass

Disabled + no cert -> collision-wise pass
Optional + no/failed cert -> collision-wise pass
```

---

# 8. C0.5 — Reject Invalid Explicit Collision Mode

## Goal

消除配置 typo 导致 silent Disabled downgrade。

## Required Behavior

区分：

```text
field missing
field valid
field invalid
```

语义：

```text
missing:
  legacy migration

valid:
  explicit value

invalid:
  configuration invalid / fail-closed
```

禁止：

```text
invalid new string
-> legacy false default
-> Disabled
```

## Affected Surface

```text
src/modules/cam/settings/cam_config.cpp
src/modules/cam/settings/cam_config.h
tests/cam_motion_foundation_test.cpp
```

## Allowed Implementation

可选择：

```text
- parser 返回 optional/result + invalid flag
- MachineProfile 保存 configuration-valid state
- load 时记录 config error 并令 Required/activation fail-closed
```

要求行为明确，不强制 private API 名称。

## Tests

```text
legacy true -> Required
legacy false -> Disabled

"disabled" -> Disabled
"optional" -> Optional
"required" -> Required

"requried" -> invalid/fail-closed
"" explicitly present -> invalid/fail-closed
```

---

# 9. B1 Handoff Note — Context Placeholders

本项不是 B0 收口 blocker，不要在此重新实现 B1 compiler。

但必须在代码注释/状态文档里明确：

当前 B0 context 中部分字段是 transition/baseline identity，不是最终 production authority，例如：

```text
workspaceGeneration
setupCalibrationHash
controllerMode
controllerCapabilityHash
toolProcessHash
dynamicsSemanticHash
```

B1 Workstream 7 必须从真实 immutable compilation context 捕获这些来源。

不得因为字段“已有非空 hash”而认为 controller capability 已经 qualified。

---

# 10. Stage Gate — B0-CLOSE

所有代码完成后只执行一次正式收口验证。

## Required targeted suite

至少：

```text
cam_motion_plan_contract_test
cam_motion_foundation_test
process_cutting_safety_test
initial-approach related contract/regression
no-model CAM flow
directly impacted travel path tests
```

若仓库已有对应 CTest label，可按 label 批量运行。

## Regression rule

只跑直接影响矩阵，不要求：

```text
VS Release full matrix
ASan full suite
GTN/ACS real hardware
laser
HIL
long soak
B4 production collision qualification
```

## Exit Criteria

```text
[ ] C0.1 PASS
[ ] C0.2 PASS
[ ] C0.3 PASS
[ ] C0.4 PASS
[ ] C0.5 PASS

[ ] Disabled remains zero collision prerequisite
[ ] Optional remains diagnostic only
[ ] Required remains strict fail-closed
[ ] non-collision safety gates unchanged
[ ] FinalMotionPlan remains single motion truth
[ ] evaluator remains collision-independent
[ ] canonical edges are completely represented
[ ] no unresolved ESCALATE
[ ] targeted regression PASS
```

---

# 11. Completion Evidence

批次结束只输出一个简洁证据摘要：

```text
commit range
changed files / symbols
F01..F05 closure result
targeted tests
known deferred items
Astra R0 request
```

不要逐任务生成独立 evidence 文件。

建议更新：

```text
docs/versions/2026-09-14-cam-motion-v3.1-b0.md
docs/.../STATUS-MATRIX.md
```

状态变为：

```text
B0 Stage Gate Passed
R0 Review Pending
```

Astra 复审确认后：

```text
B0 Astra PASS
B1 Not Started / Released
```

---

# 12. Astra R0 复审提示词

```text
审阅当前 codex/cam-motion-v3.1-b0 最新 HEAD。

这是 B0 PASS_WITH_PATCH 收口复审，不重新规划 B0。

重点确认：

1. canonical motion edges 是否全部且唯一属于可求值 interval；
2. ContinuousMotionEvaluator 是否绑定正确 model identity，
   并拒绝非法 conservative bounds；
3. Optional 是否完全退出 initial approach path selection/gating；
4. Required 是否只接受 CertifiedSafe certificate；
5. invalid explicit collisionVerificationMode 是否不会 silent downgrade；
6. 原有 Disabled no-model flow 与 Required fail-closed 是否保持；
7. 是否产生新的第二 motion truth、SDK queue 或隐藏 fallback；
8. B1 是否可以在不修改 B0 核心合同的情况下开始。

输出只能为：
PASS
PASS_WITH_PATCH
BLOCK

若 PASS：
明确写：
R0 = PASS
B1_RELEASE = YES

不要要求 B1/B2/B3/B4 范围的验收来阻塞本次 B0 收口。
```

---

# 13. Final Release Rule

完成本文件不是生产放行。

它只决定：

```text
B0/R0 complete
     ↓
B1 CAM Motion Compiler development may start
```

最终生产放行仍由 B3/B4 的 commissioning / collision / HIL / process qualification 决定。
