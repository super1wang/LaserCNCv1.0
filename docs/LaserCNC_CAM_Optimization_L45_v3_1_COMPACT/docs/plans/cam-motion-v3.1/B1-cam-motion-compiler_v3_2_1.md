# B1 — CAM Motion Compiler v3.2.1（补充优化版）

> 目标：在保持原 B1 验收节奏不变的前提下，补齐 B0 收口后暴露出的执行语义，并把 B1 细化到足够高能力模型连续开发，而不是拆成大量微型 Work Unit。  
> 验收节奏保持：**B1.S1 / B1.S2 / B1.S3 三个 Soft Checkpoint + 1 次 B1 Stage Gate + 1 次 Astra R1 Review**。  
> B0 合同为冻结输入；除非触发 `ESCALATE`，B1 不重新设计 B0。

---

## 1. Goal

实现完整 CAM-side trajectory compiler：

```text
冻结编译输入
  -> Geometry normalization / primitive recovery
  -> authoritative complete IK solve
  -> Raw solved Full5D
  -> Optimized Full5D
  -> adaptive resampling / 5D merge
  -> physical DOF candidate generation
  -> restricted Z candidate
  -> controller-mode/capability admission
  -> deterministic selection
  -> rebuild derived semantics
  -> atomic FinalMotionPlan publication
```

B1 仍然替代旧 WP03–WP10；不新增额外正式验收批次。

---

## 2. B1 批次前置：冻结编译输入

在任何后台优化计算开始之前，CAM owner thread 必须捕获一个不可变的 B1 compilation input。该步骤是整个 B1 的前置动作，不单独形成新的验收节点。

至少冻结：

```text
workspace/generation
source toolpath revision
contour order/source geometry revision
OCC Wire/Edge/parameter source identity
machine kinematic topology
physical axis layout / soft limits
setup/WPC revision
calibration revision
tool/process recipe and motion-affecting values
optimization policy + requested/effective/source/unit/revision
ControllerMotionMode
controller capability/qualification revision
dynamics/profile semantics used by admission
interpolation model/version
CollisionVerificationMode snapshot
```

B0 中为占位/过渡用途生成的 hash 或 revision 不得因为“非空”而被视为最终生产 authority。

后台 worker 只能消费该 detached immutable input；不得中途读取可变 UI、可变配置、实时控制器位置或当前 controller capability。

---

## 3. Workstream 1 — Geometry normalization and source fidelity

### 目标

生成最终供 authoritative IK solver 消费的 ordered geometry samples + primitive metadata。

### 规则

- 去除 strict numeric duplicate；
- 修正 finite normal/tangent 数值噪声；
- 保留 closed contour winding/order；
- 保留 seam、corner、process fence、lead-in attachment；
- source edge / curve parameter 映射必须可追溯；
- OCC Wire/Edge/parameter interval 是第一 authority；
- line/circle/arc/trimmed primitive 优先直接读取 OCC；
- ellipse 允许识别但不要求 native lowering；
- BSpline/Bezier 保留原曲线语义；
- 只有 source geometry 缺失时才允许 point fitting，并记录 residual/provenance/tolerance；
- IK 前 resampling 只使用 chord / tangent / normal / process-feature criteria；
- 不依赖 controller/collision，也不做 MotionClass 判定。

### Hard barriers

B1 首版禁止任何优化跨越：

```text
contour boundary
LeadIn -> Cutting
Cutting -> Rapid
Retract -> Traverse
Traverse -> Approach
laser/process event
required stop
semantic OCC seam/corner
B0 entryBoundary ownership boundary
```

### Geometry → IK 冻结边界

如果 Geometry Optimizer 对 sample sequence 发生了任意：

```text
insert
delete
move
resample
primitive sample replacement
```

则之前的 machine coordinates / solved poses 全部视为 stale。

Workstream 2 必须从最终 WS1 geometry sequence 调用现有 authoritative complete IK solve，禁止复用旧 solved axes。

### Soft checkpoint B1.S1

一次性运行 accumulated geometry suite：

```text
duplicate/noise
closed contour/winding
seam/corner
reversed/trimmed line/arc/circle
false primitive fit
source mapping
lead-in attachment
fit provenance
bounded geometry resampling
```

通过后直接继续，不做 Astra review。

---

## 4. Workstream 2 — Authoritative complete IK solve + Optimized Full5D foundation

### 目标

把 WS1 最终几何序列转换成连续物理 solved path，并建立 Full5D 优化基线。

### Required behavior

- 使用现有 authoritative solver；
- 复用现有跨轮廓/有序序列的 branch continuity 语义；
- 不在 optimizer 中实现第二套 global IK；
- 输出 physical-layout solved axes；
- 同步得到 world TCP / process direction / reference TCP 所需的 authoritative semantics；
- periodic rotary 采用 unwrapped physical angle；
- 保留 turn count 和 direction；
- 不做 `%360`；
- soft-limit 不能通过等价角归一化规避；
- orientation tolerance 为 0/缺失时，只允许 numerical-equivalent denoise；
- 非零 orientation process tolerance 才允许 bounded smoothing，并且必须沿 material arc length 验证；
- 每个 simultaneous five-axis path 都必须产生显式 `Optimized Full5D` reference。

如果 Full5D optimizer 没有 materially better 结果：

```text
changed = false
```

仍然视为成功。

---

## 5. Workstream 3 — Adaptive pose resampling

使用 B0 冻结的 `ContinuousMotionEvaluator`，并绑定本次 frozen compilation context。

目标设计的误差/细分依据如下。S2 首版仅 rotary step 是生产硬细分条件；
TCP/orientation 区间 bound 为基础设施，velocity/acceleration/reversal 为 audit-only，不能据此宣称生产资格：

```text
TCP positional deviation
orientation deviation
rotary step
rotary velocity demand proxy
axis acceleration/reversal proxy
```

要求：

- preserve monotonic source parameter；
- preserve hard barriers；
- `maxRotaryStepDeg` 是 hard bound；
- 新增 knot 的 physical axes / world TCP / reference TCP / process direction 必须由 authoritative evaluator/kinematics 生成；
- 不得分别线性插值这些缓存字段；
- removed/inserted spans 保留 concise max-error evidence。

### Compute budget

必须有明确实现上限，例如：

```text
max refinement depth
minimum parameter interval
maximum generated-knot multiplier/per-block cap
cancellation checkpoints
```

预算耗尽：

```text
keep shorter/original optimized block
```

或拒绝当前候选。

绝不通过增大 tolerance 来满足 knot-count 目标。

---

## 6. Workstream 4 — 5D continuous merge / typed blocks

尝试合并连续 Full5D knots 时，必须使用相同 shared evaluator 验证整个 candidate interval。

至少检查：

```text
source/internal/adaptive parameters
position deviation
orientation deviation
axis soft limits
rotary continuity
active-axis semantics
entryBoundary ownership
process fences / hard barriers
```

Endpoint-only equality 不足以证明可 merge，尤其是 RTCP semantics。

Unsupported/Unknown conservative bound：

```text
保留更短 Optimized Full5D blocks
```

不得回退 raw sampled path。

### Soft checkpoint B1.S2

一次性运行 Full5D numeric suite：

```text
359 -> 361
351 -> 711
near-limit unwrap
orientation numerical noise
explicit nonzero smoothing budget
adaptive refine/decimate
hard max rotary step
midpoint counterexample
entryBoundary/fence non-crossing
budget exhaustion
determinism repeat
```

通过后继续。

---

## 7. Workstream 5 — Physical DOF candidate generation

候选只能来自 `Optimized Full5D` physical motion。

候选类型：

```text
SingleAxis
Coordinated2D
Coordinated3D
Reduced4D
Optimized Full5D
```

### Active-axis analysis

每个物理轴至少分析：

```text
total span
monotonicity
max local delta
dynamic contribution
freeze residual at canonical value
```

必须使用 physical axes，不允许基于 raw RTCP TCP 数值是否变化来判定。

### Continuous equivalence

候选至少在：

```text
source sample parameters
internal knots
adaptive interpolation parameters
```

比较：

```text
candidate physical axes
  -> forward kinematics
  -> TCP/orientation/process semantics

vs

Optimized Full5D reference
```

### 必须保留的反例

1001 个每步很小但累计显著的运动，不能因为 local delta 小就冻结该轴。

### Controller admissibility

selected `ControllerMotionMode` + capability 是否能 exact/qualified 表达 candidate，必须在 FinalMotionPlan publication 前检查。

不支持：

```text
reject candidate
```

Lowering/Process 不允许后来换成另一条运动候选。

---

## 8. Workstream 6 — Restricted laser Z-hold

### 8.1 Numerical Z Canonicalization

B1 首版必须完成。

如果物理 Z 只有 numerical noise，可在 strict equivalence 下固定 canonical Z。

这不使用 relaxed process tolerance。

### 8.2 Process-tolerance Z Hold

架构保留，但首版范围收窄。

默认：

```text
trajectory.enableLaserZHold = false
```

只有显式开启并且存在完整 process envelope 的 whitelist case 才允许产生 candidate。

必须具备：

```text
focus/standoff bounds
contour position tolerance
orientation tolerance
calibration/runout/control error bounds
supported beam/material intersection model
continuous feasible physical-Z interval
no independent height-follow conflict
no mid-cut jump
```

缺失或 ownership 不明确：

```text
reject candidate
or ESCALATE
```

B1 不要求为了“完成 Z-hold”发明通用 constrained IK 或未知工艺模型。

---

## 9. Workstream 7 — Controller admission / deterministic selection / atomic publication

### optimizationMode

冻结为：

#### Off

- 不做 path-changing optimization；
- 只做 contract validity / 必要 numerical normalization；
- 不做 DOF reduction；
- 不做 relaxed smoothing；
- 不做 process Z-hold。

#### Conservative

- 只允许 zero-process-tolerance / numerically equivalent 优化；
- rotary unwrap；
- numerical denoise；
- strict-equivalent resampling/merge/reduction；
- numerical Z canonicalization。

#### Full

- 包含 Conservative；
- 目标能力：有显式 process/geometry tolerances 与连续区间证明时可做 bounded resampling/merge；
- S2 首版：生产 `boundPhysicalAxes` 尚不可用，Full 保留 Conservative 等价子集并记录确定性拒绝原因；位置/姿态容差细分仅基础设施，测试 callback 不是生产资格；
- S2 非零 smoothing 仍拒绝并保留 strict reference，动态 proxy 仅审计；生产 policy 仍 Off，controller 为 Unavailable/revision 0；
- `enableDofReduction=true` 时允许 reduction；
- `enableLaserZHold=true` 且 whitelist/process envelope 完整时才允许 process Z-hold。

### Deterministic selection

先执行 hard constraints，再使用 deterministic cost。

首版推荐 lexicographic tuple：

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

禁止 tie-break 依赖：

```text
QHash/QSet iteration order
worker completion order
pointer/address
unstable floating comparison
```

### Derived semantic rebuild

任何 path-changing optimization 后，必须从 evaluator/kinematics 重建：

```text
world TCP
process/beam direction
reference TCP / tcpMcs
source mapping
feed/duration evidence
source spans
fences
tolerance proof
optimizer metrics
```

不得保留与新 physical motion 不一致的旧缓存。

### Atomic publication

owner thread 最终重新验证：

```text
generation
source revision/order
geometry revision
context/controller mode
capability revision
policy hash
cancel/stale state
```

通过后，一次性发布：

```text
FinalMotionPlan
MotionCompilationContext
plan/context/block hashes
optimizer report
```

Late collision proof 只能 attach 到 exact identity，不能改 motion。

### Soft checkpoint B1.S3

一次性运行：

```text
true C-only / U+C
false C-only
1001 cumulative tiny increments
numerical Z canonicalization
process Z-hold qualified/rejected/disabled
unsupported mode/capability
deterministic tie
same input -> identical planHash
stale/cancel
entryBoundary preservation
derived-field consistency
Off/Conservative/Full differential
```

---

## 10. B1 Stage Gate

R1 收口实施：结构 fence 与实际激光变更/required stop 分离；公共 provider 通过冻结输入任务
完成 Full5D/Reduction，只发布匹配 current authority、配置代际、实际顺序和 travel key 的完整记录。
同 identity 重读复用编译结果；碰撞附件按 exact planHash 更新。证据与首版范围见
[B1_R1_CLOSEOUT_REPORT.md](B1_R1_CLOSEOUT_REPORT.md)。R1 人工复核前 B2_RELEASE 保持 NO。

仍只进行一次正式 Stage Gate。

运行：

```text
V-007..V-016
+ directly impacted CAM/kinematics regressions
+ B1.S1/S2/S3 accumulated suites
```

必须输出代表性 before/after metrics：

```text
input/output knots
merge ratio
max position/orientation deviation
rotary travel/reversal before/after
max rotary delta
selected/rejected candidate counts
estimated command count
compile time
planHash determinism
```

硬条件：

```text
collision backend calls = 0
false tolerance relaxation = 0
raw sampled fallback after valid Full5D = 0
stale/cancel partial publication = 0
same input/context/policy hash mismatch = 0
```

### Exit Criteria

- [ ] immutable compilation input 在后台开发前冻结；
- [ ] geometry source fidelity/topology/fences preserved；
- [ ] geometry change 会强制 authoritative re-solve；
- [ ] every simultaneous path has Optimized Full5D；
- [ ] evaluator 驱动 resampling/merge；
- [ ] rotary turns/direction survive；
- [ ] B0 entryBoundary/process fence preserved；
- [ ] path-changing optimization 后 derived semantics rebuilt；
- [ ] reduced candidates continuously admissible；
- [ ] numerical Z canonicalization strict-equivalent；
- [ ] relaxed Z-hold whitelist-only or remains off；
- [ ] controller mode/capability admission before publication；
- [ ] Off/Conservative/Full semantics correct；
- [ ] deterministic selection；
- [ ] budget exhaustion never expands tolerance；
- [ ] atomic stale-safe publication；
- [ ] zero collision dependency/backend call；
- [ ] no unresolved ESCALATE。

完成后请求 Astra R1。

B2 只在 R1 PASS 或 PASS_WITH_PATCH 完全收口后开始。
