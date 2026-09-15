# Motion Parameters / Policy Ownership — v3.2.1 FINAL

> 适用范围：CAM Motion v3.2.1，覆盖 B1 参数所有权、FinalMotionPlan 身份/失效规则，以及 B1 → B2 的参数交接。  
> 目标：避免参数重复归属、隐式默认值、UI 值冒充设备有效值，以及 B1/B2 各自维护一套不一致的运动参数。  
> 本文是参数语义与 ownership 的权威文档，不负责重复定义 Geometry / Pose / DOF 算法。

---

# 1. Parameter Domains

参数按职责分为四类：

```text
A. CAM Optimization Policy
B. Execution Policy
C. Controller Expert Parameters
D. Commissioning-Locked Parameters
```

四类参数必须保持 ownership 清晰。

任何新参数在加入前必须先确定：

```text
owner
requested value
effective value
unit
source
revision
hash/invalidation scope
```

禁止为了方便在 CAM、Process、GTN adapter、UI 中复制同一个参数定义。

---

# 2. CAM Optimization Policy

CAM Optimization Policy 只控制 B1 trajectory compiler 的优化行为。

## 2.1 核心模式

```text
trajectory.optimizationMode = Off | Conservative | Full
```

### Off

语义：

```text
不做 path-changing optimization
只允许 contract-validity / mandatory numerical normalization
不做 DOF reduction
不做 relaxed orientation smoothing
不做 process-tolerance Z-hold
```

允许：

```text
finite-value normalization
必要的 canonical representation
不改变物理加工意义的严格数值整理
```

不允许：

```text
以减少 knot 为目标的 merge
使用非零 process tolerance 改变路径
ReducedDOF candidate selection
process Z-hold
```

### Conservative

语义：

```text
只允许 zero-process-tolerance / numerically-equivalent 优化
```

允许：

```text
rotary unwrap
numerical orientation denoise
strict-equivalent adaptive resampling
strict-equivalent continuous merge
strict-equivalent DOF reduction
numerical Z canonicalization
```

禁止：

```text
消耗 relaxed process tolerance
改变真实 surface normal / contour geometry
process-tolerance Z-hold
```

### Full

语义：

```text
包含 Conservative
+ 在明确的 geometry/process tolerance 下启用 bounded optimization
```

允许：

```text
bounded orientation smoothing
bounded adaptive resampling
bounded 5D merge
DOF reduction（enableDofReduction=true）
process Z-hold（enableLaserZHold=true 且 whitelist/process envelope 完整）
```

`Full` 不代表可以自动放大 tolerance，也不代表可以跳过 continuous proof。

---

## 2.2 B1 Optimization Parameters

至少包括：

```text
trajectory.optimizationMode
trajectory.orientationToleranceDeg
trajectory.maxRotaryStepDeg
trajectory.enableDofReduction
trajectory.enableLaserZHold
trajectory.geometryToleranceSource
trajectory.positionToleranceSource
trajectory.processToleranceSource
trajectory.resampling.maxDepth
trajectory.resampling.minParameterSpan
trajectory.resampling.maxKnotMultiplier
trajectory.merge.maxCandidateSpan / equivalent search budget
trajectory.cost policy
trajectory.rotary reversal / soft-limit weights or tuple policy
```

首版不要求这些字段全部暴露为 UI 配置项；但参与结果的值必须有明确 source 和 revision。

---

# 3. Tolerance Ownership

## 3.1 Position / Geometry Tolerance

不得由 B1 planning package 凭空定义新的 machining tolerance。

position / chord / contour error 必须来自：

```text
existing authoritative CAM tolerance
explicit process recipe
explicit commissioning recipe
```

B1 只能读取并冻结，不得自行扩大。

如果没有 authoritative nonzero tolerance：

```text
只能执行 numerical-equivalent / zero-process-tolerance optimization
```

---

## 3.2 Orientation Tolerance

```text
trajectory.orientationToleranceDeg
```

默认语义：

```text
0 或未配置
=> 只允许 numerical-equivalent denoise
```

只有显式非零 process/angular tolerance 才允许：

```text
bounded orientation smoothing
```

并且必须：

```text
沿 material arc length 验证
保留 endpoint
保留 process fence
保留 entryBoundary semantics
```

---

## 3.3 Rotary Step

```text
trajectory.maxRotaryStepDeg
```

这是 hard bound。

不得为了满足 knot count、merge ratio 或 controller command count：

```text
自动提高 maxRotaryStepDeg
```

---

## 3.4 Z-Hold Tolerance

分为两类：

### Numerical Z Canonicalization

不使用 relaxed process tolerance。

只在 physical Z variation 为 numerical noise 时允许。

### Process-Tolerance Z Hold

只有：

```text
trajectory.enableLaserZHold = true
```

并且存在真实 process envelope 时才允许。

所需 authority 至少包括：

```text
focus / standoff tolerance
contour position tolerance
orientation tolerance
calibration error bound
runout / control error bound
supported beam-material intersection model
```

缺失或 ownership 不明确：

```text
reject candidate
or ESCALATE
```

不得自行填默认 process Z tolerance。

---

# 4. Execution Policy

Execution Policy 与 CAM Optimization Policy 正交。

包括：

```text
ControllerMotionMode = PhysicalAxes | RTCP
CollisionVerificationMode = Disabled | Optional | Required
```

这两个维度不属于 Geometry/DOF optimizer 自由选择的参数。

---

## 4.1 ControllerMotionMode

`ControllerMotionMode` 是 B1 candidate admission 的输入。

B1 必须在 FinalMotionPlan publication 前确认：

```text
candidate
+
selected ControllerMotionMode
+
controller capability/qualification revision
```

是否可以 exact / qualified 表达。

不兼容：

```text
reject candidate
```

禁止：

```text
B1 publish candidate
-> B2 lowering 再改成另一条 motion candidate
```

Lowering 只能编码已经发布的 selected motion。

---

## 4.2 CollisionVerificationMode

```text
Disabled
Optional
Required
```

B1 optimizer 不依赖 collision backend。

三种模式都不能改变 B1 的 Geometry / Full5D / DOF candidate selection。

B1 中 CollisionVerificationMode 只作为 frozen execution-policy identity 进入 compilation context。

### Disabled

```text
不要求 machine STEP / package / overlay / certificate
不构造/查询自动 collision backend
界面明确“碰撞验证关闭 / 未碰撞认证”
```

### Optional

```text
diagnostic only
不阻断 B1 motion compilation
不参与 trajectory/candidate selection
```

### Required

```text
FinalMotionPlan 发布后由 collision verifier 做 fail-closed certification
```

B1 不因为 Required 模式改变 motion candidate。

---

# 5. Feature Flags

## 5.1 DOF Reduction

```text
trajectory.enableDofReduction
```

### false

```text
不产生/不选择 reduced motion candidate
最终仍使用 Optimized Full5D
```

### true

允许：

```text
SingleAxis
Coordinated2D
Coordinated3D
Reduced4D
```

参与 candidate generation/admission。

前提仍然是：

```text
continuous physical equivalence
controller-mode admissibility
```

---

## 5.2 Laser Z Hold

```text
trajectory.enableLaserZHold
```

### false

```text
禁止 relaxed process Z-hold
```

但不影响：

```text
strict numerical Z canonicalization
```

### true

只允许：

```text
whitelist + explicit process envelope
```

首版不要求通用 constrained process IK。

---

# 6. Controller Expert Parameters

以下参数 ownership 继续归现有 controller/GTN settings，不在 B1 新建副本：

```text
Group motion/orientation constraints
per-axis vel/acc/dec/jerk/dvMax
Group smooth time / k
lookAheadNum
lookAheadTime
radiusRatio
command velocity reference axes / rotary ratios
RTCP validation tolerance / stride
Group/list/axis mapping configuration
```

B1 可以读取这些参数的 effective semantics，用于：

```text
candidate admissibility
dynamics proxy
controller capability qualification
```

但不能成为新的 owner。

---

# 7. Commissioning-Locked Parameters

以下参数属于 commissioning authority：

```text
axis scale / resolution / direction
machine kinematic topology
rotation centers / axis vectors
setup / calibration
controller / firmware / SDK identity
hard limits
soft limits
```

B1 只能冻结其 revision/fingerprint/value snapshot。

B1 不得：

```text
自动修改
自动校正
为通过 optimizer 测试而替换
```

如果 commissioning 参数变化导致 motion semantics 变化：

```text
FinalMotionPlan stale
=> recompile
```

---

# 8. Immutable Compilation Input

B1 worker 开始前，owner thread 必须冻结所有影响 path/candidate/identity 的值。

至少包括：

```text
workspaceGeneration
sourceToolpathRevision
contourOrder/source geometry revision
machineKinematicsHash/revision
axis layout
soft limits
setupCalibrationHash/revision
toolProcessHash/revision
optimizationPolicyHash
ControllerMotionMode
controllerCapabilityHash/revision
dynamicsSemanticHash/revision
CollisionVerificationMode
interpolationModelVersion
```

后台 worker 禁止读取：

```text
mutable UI settings
mutable config
live controller feedback
current capability state
current process recipe
```

作为本次编译的新 authority。

---

# 9. FinalMotionPlan / Context Identity Rules

以下变化 MUST invalidate FinalMotionPlan 或改变 context identity。

## 9.1 Must Affect FinalMotionPlan / Context Identity

- source geometry / source revision；
- contour order；
- workspace generation；
- machine kinematic topology；
- axis layout / relevant soft limits；
- setup / calibration；
- path-affecting tool/process values；
- optimizationMode；
- path-affecting optimizer tolerances；
- maxRotaryStepDeg；
- enableDofReduction；
- enableLaserZHold（当会影响 candidate set 时）；
- ControllerMotionMode（当语义/compatibility 不同）；
- controller capability/qualification revision；
- interpolation model/version；
- dynamics semantics（当影响 candidate admissibility/trajectory shape）；
- candidate cost/tie-break policy revision。

---

## 9.2 Parameters That May Not Require CAM Geometric Recompile

有些 controller runtime values 可能不改变 B1 geometry/motion semantics，例如纯执行层调优。

它们可以不使 FinalMotionPlan stale，但如果用于实际 run，则必须在 B2：

```text
PreparedDeviceProgram
```

中冻结、记录和 hash。

---

# 10. PreparedDeviceProgram Handoff Rules

B1 只发布 controller-independent FinalMotionPlan + frozen compilation context。

B2 构建 `PreparedDeviceProgram` 时至少还要冻结：

```text
FinalMotionPlan planHash
MotionCompilationContext contextHash
ControllerMotionMode
controller capability/qualification revision
effective controller smooth/lookahead/profile settings
tool/process recipe used for run
axis/group mapping
laser/gas IO recipe
run epoch / execution identity
```

原则：

> 即使某 controller runtime parameter 不需要 CAM geometric recompile，只要它影响本次真实运行的插补/动态语义，就必须在 PreparedDeviceProgram 中冻结和可审计。

如果某 runtime tuning 改变 B1 evaluator/certifier 假定的 interpolation semantics：

```text
它不再只是 runtime tuning
```

必须使相应 plan/model/proof stale。

---

# 11. Requested / Effective / Source / Unit / Revision

任何 materially affecting B1/B2 行为的参数，都必须可记录：

```text
requested
effective
source
unit
revision
```

示例：

```text
requested orientationToleranceDeg = 0.05
effective orientationToleranceDeg = 0.05
source = processRecipe.orientationToleranceDeg
unit = degree
revision = recipe:42
```

禁止仅记录 UI 输入值，然后假设设备或编译器实际使用了该值。

---

# 12. Parameter Provenance in Optimizer Report

每次 B1 compile report 至少保留：

```text
optimizationMode
position tolerance + source
orientation tolerance + source
maxRotaryStepDeg
enableDofReduction
enableLaserZHold
ControllerMotionMode
controller capability revision
dynamics semantics revision
interpolation model version
cost policy revision
```

这样：

```text
A/B test
bug reproduction
planHash explanation
commissioning comparison
```

都有可追溯依据。

---

# 13. optimizationMode Differential Contract

相同 source/context 下：

## Off

预期：

```text
无 path-changing optimization
无 ReducedDOF
无 relaxed smoothing
无 process Z-hold
```

## Conservative

预期：

```text
只出现 numerical-equivalent 变化
```

## Full

预期：

```text
在 explicit tolerance 内可以发生 bounded path reduction/merge/resampling
```

测试中必须验证三种模式行为差异不是 UI-only。

---

# 14. Determinism / Cost Policy

candidate selection 必须 deterministic。

首版推荐：

```text
hard constraints
  ↓
lexicographic cost tuple
```

例如：

```text
requiredStops
estimatedTimeProxy
softLimitRisk
rotaryReversalCount
normalizedAxisTravel
outputKnotCount
normalizedError
stableCandidateOrdinal
```

如果采用 weighted score，必须冻结：

```text
unit
normalization
default weight
tie epsilon
stable final tie-break
```

并进入：

```text
optimizationPolicyHash
```

禁止 candidate winner 依赖：

```text
QHash/QSet iteration order
worker completion order
pointer/address
unstable floating comparison
```

---

# 15. Compute Budget Parameters

B1 adaptive refinement / merge / candidate search 必须有明确预算，例如：

```text
max refinement depth
min parameter interval
max output knot multiplier
max candidate span
max candidate count
cancellation interval/checkpoint
```

预算耗尽：

```text
keep original/shorter optimized block
reject candidate
```

禁止：

```text
increase tolerance
skip proof
publish partial result
```

预算参数属于 optimization policy，若会改变结果必须进入 policy hash。

---

# 16. UI Rules

## Collision Disabled

必须显示：

```text
碰撞验证关闭
未碰撞认证
```

不得显示：

```text
Certified Safe
```

不得因为：

```text
missing machine collision model/package
```

显示为 execution error。

但以下非碰撞安全状态不能被隐藏：

```text
controller disconnect
axis fault
axis disabled
kinematic invalid
soft-limit violation
Stop/E-stop
```

---

## Full5D Optimizer No Material Change

应显示：

```text
Optimized / no material change
```

不得显示成：

```text
optimizer failed
```

---

## Controller Mode Change

如果改变当前 plan candidate compatibility：

```text
mark stale
request recompile / reselection
```

不得静默重编码为另一条 motion candidate。

---

# 17. Current Priority Commissioning Profile

当前优先 commissioning profile：

```text
trajectory.optimizationMode = Full
trajectory.enableDofReduction = true
trajectory.enableLaserZHold = false
collision.verificationMode = Disabled
controller.motionMode = PhysicalAxes or RTCP
  only when that cell is qualified
```

该 profile 的目的：

```text
先验证 CAM motion compiler / controller execution
不强制引入 production collision qualification
```

但仍保留所有非碰撞安全门禁。

---

# 18. B1 → B2 Ownership Boundary

## B1 Owns

```text
geometry normalization
source fidelity
authoritative IK solve result
Optimized Full5D
DOF candidate generation
controller-mode/capability admission
selected FinalMotionPlan
MotionCompilationContext
optimizer report
```

## B2 Owns

```text
PreparedDeviceProgram
FinalMotionPlan lowering/encoding
GTN group/list/native instruction selection
controller profile freeze
device command packaging
run epoch
execution-time stale checks
```

## B2 MUST NOT

```text
re-IK
replan
re-simplify
DOF reduce
change selected motion candidate
```

---

# 19. ESCALATE Conditions

必须停止并升级设计判断，如果：

```text
1. 找不到某个 materially affecting parameter 的唯一 authority；
2. controller capability 没有可 revision/hash 的 authoritative source；
3. process tolerance ownership 不明确或互相冲突；
4. 为实现 requested optimization 必须创建新的未知 machining tolerance；
5. 某 controller runtime parameter 实际改变 evaluator/certifier interpolation semantics；
6. B1 candidate 只能依靠 B2 lowering 修改运动才能执行；
7. Commissioning locked parameter 被 optimizer 试图自动修正；
8. B0 FinalMotionPlan/context 无法表达本参数的 invalidation semantics。
```

---

# 20. Invariants

- PAR-INV-01：一个 materially affecting parameter 只能有一个 authority。
- PAR-INV-02：requested 值不等于 effective 值，二者必须可区分。
- PAR-INV-03：所有 path-affecting optimizer 参数进入 compilation identity。
- PAR-INV-04：controller capability revision 参与 candidate admission identity。
- PAR-INV-05：runtime tuning 若改变 interpolation semantics，则必须使相关 plan/model/proof stale。
- PAR-INV-06：B1 不复制 GTN/controller expert parameter ownership。
- PAR-INV-07：B1 不修改 commissioning-locked 参数。
- PAR-INV-08：无 authoritative tolerance 时不得自行创造 relaxed optimization budget。
- PAR-INV-09：同 input/context/policy 必须 deterministic。
- PAR-INV-10：B2 只 lowering/encoding，不重新选择运动。
