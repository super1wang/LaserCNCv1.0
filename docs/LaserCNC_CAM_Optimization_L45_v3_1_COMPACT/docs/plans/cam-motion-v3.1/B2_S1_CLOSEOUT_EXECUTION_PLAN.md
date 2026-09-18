# LaserCNC CAM Motion v3.2.1 — B2.S1 收口执行规划

> 目标分支：`codex/cam-motion-v3.2.1-b1`  
> 审计 HEAD：`0b6b3ae0177b550c657b69e0a02fe2e5c38007c1`  
> 当前状态：`B2.S1 = PASS_WITH_PATCH`  
> 目标状态：`B2.S1 = PASS`，随后自动进入 B2.S2  
> 正式 B2 Stage Gate：**本轮不执行**

---

## 1. Scope

本轮只关闭：

```text
C1 — frozen Tool execution recipe identity
C2 — exact-plan pause boundary contract
C3 — direct regressions / documentation truth
```

禁止扩展到：

```text
PhysicalAxes lowering
RTCP lowering
controller native interpolation
Group reuse
LookAhead
indeterminate Start
B2 Stage Gate
HIL / real machine qualification
production collision qualification
```

---

## 2. Frozen Invariants

```text
INV-B2S1-01 FinalMotionPlan is the only real motion source
INV-B2S1-02 PreparedDeviceProgram is immutable after prepare
INV-B2S1-03 Process never performs IK / resample / reduction / candidate switch
INV-B2S1-04 real controller failure never falls back to legacy point loop
INV-B2S1-05 PureSimulation is explicit compatibility mode only
INV-B2S1-06 run recipe must have complete content identity
INV-B2S1-07 no synthetic/default Tool recipe may enter a real program
INV-B2S1-08 controller qualification remains fail-closed
INV-B2S1-09 collision policy comes from committed FinalMotionPlan context
INV-B2S1-10 existing DeviceCommandQueue remains the only device command authority
INV-B2S1-11 Pause never becomes automatic replay
INV-B2S1-12 Stop always dominates Pause
```

---

## 3. C1 — Freeze an Explicit Tool Execution Recipe

### Goal

让 B2.S2 可以安全相信 `PreparedDeviceProgram.recipe` 已包含其允许使用的全部 runtime semantics，任何运行语义变化都会改变 identity。

### C1.1 Define the executable subset

新增推荐类型：

```cpp
struct FrozenToolExecutionRecipe
{
    QString toolName;

    double lineVelocity;
    double lineAcceleration;
    double lineJerk;

    // laser / gas / IO / qualified profile fields actually consumed later
};
```

字段以 S2/S3 真正消费的运行语义为准。不要把整个 legacy `Tool` 自动定义成 execution contract。

### C1.2 Classify legacy path-changing fields

明确处理：

```text
bMovePos*
fMovePos*
fExtend*
geometry offset fields
Z-linkage trajectory semantics
servo-height path alteration
other fields that can alter FinalMotionPlan trajectory
```

exact-plan 模式必须二选一：

```text
A. already resolved by CAM -> must be neutral at Process boundary
B. unsupported -> prepare rejects
```

禁止 B2 lowering 再解释这些字段。

### C1.3 Canonical content hash

建立：

```text
frozenToolExecutionRecipeHash()
```

或者等价 deterministic encoder。

要求：

```text
- deterministic field order
- every executable field participates
- unit/type semantics are fixed
- every numeric executable value is finite
- material tool/source identity is included
```

`DeviceRunRecipe` identity 改用这一结构。

不要继续依赖 `Tool::toTable()` 作为完整 runtime identity，除非正式把它重新定义为完整 execution canonical representation，并覆盖全部执行字段。

### C1.4 Value capture

owner thread：

```text
ToolFactory / settings
    ↓
FrozenToolExecutionRecipe values
    ↓
DeviceRunRecipe
    ↓
PreparedDeviceProgram
```

PreparedDeviceProgram 不得保留：

```text
Tool*
ToolFactory*
mutable settings pointer
```

### C1.5 Negative validation

real run 至少拒绝：

```text
missing recipe
synthetic/default recipe
NaN / Inf
zero/negative required feed-dynamics values
unsupported path-changing legacy semantics
recipe hash mismatch
```

---

## 4. C2 — Freeze Exact-Plan Pause Contract

### Goal

在 B2.S2 开始 controller lowering 前，明确 controller program 必须遵守的 host pause semantics。

### C2.1 Introduce execution section identity

PreparedDeviceProgram 需要输出稳定 section boundaries。

首版推荐：

```text
one CAM contour = one pause-safe host section
```

每个 section 至少绑定：

```text
section ordinal
contourId
first / last block identity
planHash
runEpoch
```

不要求本轮设计复杂 streaming protocol。

### C2.2 Keep prepare separate from Start

保持当前良好边界：

```text
prepareExactProgram()
= encode / stage only
```

不要在 prepare 阶段 Start。

S2 后续可扩展为：

```text
prepare exact section/program
seal
start section
query section completion
```

但不得重新暴露 legacy point-loop。

### C2.3 Pause state machine

冻结：

```text
Running
  └─ Pause requested
       ├─ current active section may finish
       ├─ next section MUST NOT start
       └─ enter Paused

Paused
  └─ Resume
       └─ continue next unexecuted section
```

必须保持同一个：

```text
PreparedDeviceProgram
planHash
contextHash
recipe revision
runEpoch
```

禁止重新 capture UI/settings。

### C2.4 Stop precedence

```text
Pause + Stop
=> Stop wins
=> safe stop path
=> no automatic replay
```

Stop/recovery 继续复用已有 DeviceCommandQueue 与 runtime。

### C2.5 Resume-point semantics

当前 `ProcessInterruptContext` 的通用 resume point 会保存 checkpoint。

exact-plan 路径必须区分：

```text
live same-run pause/resume
vs
workflow exited and later re-enters node
```

要求：

```text
same live run: may resume next exact section
new/re-entered run with old resume state: fail closed / fresh run required
```

不得恢复已经执行的 section。

---

## 5. C3 — Tests and Documentation

### Group T1 — Recipe identity

```text
T1-01 changing any executable recipe field changes DeviceRunRecipe hash
T1-02 NaN executable field rejects prepare
T1-03 missing explicit tool rejects
T1-04 fallback/synthetic recipe rejects
T1-05 unsupported path-changing legacy field rejects
T1-06 mutation after prepare does not change frozen recipe
```

### Group T2 — Pause

使用 fake exact consumer/sink：

```text
T2-01 two contours / two sections
T2-02 requestPause during section 1
T2-03 section 1 completes
T2-04 section 2 not started
T2-05 Resume starts section 2
T2-06 same runEpoch / planHash / recipe revision
T2-07 no replay of section 1
T2-08 Pause+Stop => Stop wins
T2-09 Pause before first section => no Start
```

### Group T3 — Existing regression

只跑直接受影响测试，例如：

```text
process_cutting_safety
process_workflow
process_preflight_service
device_command_queue
device_wait
command_list_start
confirmed_motion_stop
controller_recovery_sequence
process_device_shutdown_sequence
project_package
```

视实际修改再增加少量直接回归。

本轮不跑：

```text
B1 Gate
B2 Stage Gate V-017..V-023 full set
HIL
real controller
```

---

## 6. Documentation Truth

更新：

```text
B2_S1_IMPLEMENTATION.md
```

把“未提交、未推送”改成当前真实提交状态。

收口完成后追加：

```text
closeout commit
F01 result
F02 result
targeted Debug result
targeted ASan result
deferred S2/S3 boundaries
```

---

## 7. Exit Criteria

```text
[ ] executable Tool recipe has a complete canonical identity
[ ] no allowed runtime field can change without recipe hash changing
[ ] all executable numeric values are finite-validated
[ ] path-changing legacy Tool fields cannot alter FinalMotionPlan in Process
[ ] PreparedDeviceProgram remains immutable
[ ] exact-plan pause boundary is explicit
[ ] Pause cannot start the next section
[ ] Resume continues same program without replay
[ ] Stop overrides Pause
[ ] real path has no legacy/raw fallback
[ ] controller qualification remains fail-closed
[ ] collision policy behavior remains unchanged
[ ] DeviceCommandQueue remains single authority
[ ] targeted regressions pass
[ ] no unresolved S1 ESCALATE
```

全部满足：

```text
B2.S1 = PASS
B2.S2_RELEASE = YES
```

---

## 8. B2.S2 Handoff Contract

S2 只能消费：

```text
PreparedDeviceProgram
FrozenToolExecutionRecipe
MachineAxisLayout
FinalMotionPlan blocks
explicit pause-safe sections
frozen ControllerMotionMode / qualification snapshot
```

S2 负责：

```text
PhysicalAxes lowering
RTCP lowering
axis mapping
turn preservation
trace/userTag identity
feed/dynamics unit qualification
unsupported-cell rejection
```

S2 不允许重新决定：

```text
contour order
source path
IK
DOF
Z canonicalization
tool trajectory extension
legacy compensation
pause replay strategy
```

这些都应在进入 S2 前冻结。
