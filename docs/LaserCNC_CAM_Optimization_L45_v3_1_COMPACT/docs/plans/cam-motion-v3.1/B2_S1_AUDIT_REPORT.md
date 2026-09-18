# LaserCNC CAM Motion v3.2.1 — B2.S1 审计报告

> 仓库：`super1wang/LaserCNCv1.0`  
> 分支：`codex/cam-motion-v3.2.1-b1`  
> 审阅 HEAD：`0b6b3ae0177b550c657b69e0a02fe2e5c38007c1`  
> 提交：`feat(process): implement B2 S1 exact-plan preparation`  
> B1/R1 放行基线：`d1e82b9ba39eac7ee1c12e761ad411204f91d4ae`  
> 日期：2026-09-18  
> **审计结论：B2.S1 = PASS_WITH_PATCH**  
> **B2.S2_RELEASE = NO**

---

## 1. 审计目标

本轮只判断 B2.S1 是否满足既定目标：

```text
FinalMotionPlan
    ↓
Process exact-plan cutover
    ↓
immutable PreparedDeviceProgram
    ↓
real execution never falls back to legacy/raw points
```

以及以下冻结合同：

```text
- Process preserves CAM-authoritative block / contour / phase / fence order
- PreparedDeviceProgram freezes run-semantic input
- missing / stale real-run recipe blocks preparation
- Disabled / Optional collision does not require proof
- Required remains fail-closed
- Stop / fault / pause / ownership semantics remain valid
- no Process-side planning / IK / reduction / candidate reselection
```

本轮不要求实现：

```text
PhysicalAxes lowering
RTCP lowering
GTN Group / LookAhead lifecycle
HIL / real-machine qualification
B2 Stage Gate
```

---

## 2. 总体结论

当前提交已经建立了正确的 B2.S1 主体架构：

```text
real NormalCutting
    ↓
exportCommittedExecutionSnapshot()
    ↓
DeviceRunRecipe capture
    ↓
PreparedDeviceProgram::prepare()
    ↓
IMotionCommandSink::prepareExactProgram()
    ↓
existing DeviceCommandQueue
```

已完成的核心目标：

```text
- real path 不再静默回退 legacy point loop
- explicit PureSimulation 是唯一兼容 point-loop 路径
- PreparedDeviceProgram 按值冻结 FinalMotionPlan / axis layout / recipe
- stale / raw plan 拒绝
- missing/default Tool recipe 拒绝
- controller qualification 对真实执行 fail-closed
- collision policy 使用 committed FinalMotionPlan context
- Required proof 必须绑定 exact planHash
- S1 没有伪造 PhysicalAxes / RTCP lowering
```

但进入 B2.S2 前仍应关闭两个 P1 合同缺口：

```text
B2S1-F01  Tool execution recipe identity 不完整
B2S1-F02  exact-plan pause 语义尚未保持
```

因此：

```text
B2.S1 = PASS_WITH_PATCH
S2_RELEASE = NO
```

---

## 3. 已接受部分

### A1 — Exact FinalMotionPlan cutover

**PASS**

`NormalCuttingManager::run()` 在非 PureSimulation 模式下直接进入 `runExactProgram()`。旧 point-loop 不再作为真实设备失败后的替代路径。

`IMotionCommandSink::prepareExactProgram()` 默认 fail-closed：

```text
Exact-plan lowering is unavailable for this controller
```

因此 B2.S2 尚未实现时不会偷偷转回旧 `lineTo()` 路径。

### A2 — PreparedDeviceProgram value freeze

**PASS_WITH_F01**

PreparedDeviceProgram 按值保存：

```text
CamMotionPlanSnapshot
MachineAxisLayout
DeviceRunRecipe
runEpoch
realMachine flag
```

`DeviceRunRecipe` 包含：

```text
planHash
contextHash
recipe revision
sourceId
process / IO profile
Tool values by contour
feedOverride
```

没有保留 ToolFactory / mutable settings 指针。这个架构是正确的，但 Tool canonical identity 仍不完整，详见 F01。

### A3 — Stale / raw / default recipe rejection

**PASS**

准备阶段已拒绝：

```text
missing/stale FinalMotionPlan
Raw block
invalid physical layout
recipe planHash/contextHash mismatch
empty/stale recipe revision
invalid feed override
invalid/empty IO profile
missing Tool recipe
__fallback__ Tool
invalid real-run line velocity/acceleration/jerk/delay
```

### A4 — Collision policy

**PASS**

执行策略取 `plan.context.collisionMode`：

```text
Disabled  -> no proof required
Optional  -> no proof required
Required  -> exact planHash + complete collision result + edge certificates
```

真实机床 Required 还检查 machine package / job overlay readiness，符合既有 fail-closed 语义。

### A5 — Controller qualification honesty

**PASS**

真实机执行准备要求：

```text
controllerQualification == Qualified
controllerCapabilityHash matches qualification snapshot
controllerMode == requestedMode
```

当前生产资格仍为 `Unavailable / revision 0`，因此真实执行继续关闭。这是正确行为。

### A6 — Existing DeviceCommandQueue remains authority

**PASS**

new exact-plan path 仍复用已有 DeviceCommandQueue，没有增加第二套 SDK worker / queue。

---

## 4. B2S1-F01 — P1：Tool frozen recipe identity 不完整

### 4.1 当前数据路径

```text
ToolFactory::executionRecipes()
    ↓
QHash<ContourId, Tool>
    ↓
DeviceRunRecipe
    ↓
deviceRunRecipeHash()
    ↓
Tool::toTable()
```

Tool 本身是完整 value copy，这一点正确。但 recipe identity 与 finite validation 都依赖 `Tool::toTable()`。

### 4.2 `SetFromTable()` 与 `toTable()` 不对称

当前实现中：

```text
Tool::SetFromTable() 读取字段：98
Tool::toTable() 写出字段：77
```

至少以下字段被读入 Tool，却未进入 `toTable()`：

```text
bSetPosA
fSetPosA
bSetPosA1
fSetPosA1

bMovePosX
fMovePosX
bMovePosX1
fMovePosX1

bMovePosA
fMovePosA
bMovePosA1
fMovePosA1

bMovePosY
fMovePosY
bMovePosY1
fMovePosY1

bCuttingHead
bCrossBridge
fServoCuttingHeight

fExtendSctart
fExtendEnd
```

因此可能出现：

```text
Tool A != Tool B
but
deviceRunRecipeHash(A) == deviceRunRecipeHash(B)
```

同理，遗漏字段中的 NaN/Inf 也不会被当前 `finiteValues(tool.toTable())` 检查到。

### 4.3 不能简单把所有 legacy Tool 字段继续交给 Process

需要区分：

**真正 execution recipe 字段**，例如：

```text
cut feed
acceleration
jerk
laser timing
laser energy / frequency
gas / IO
qualified controller smoothing/profile
```

这些必须进入 frozen execution identity。

**trajectory-changing legacy fields**，例如：

```text
bMovePos*
fMovePos*
fExtend*
geometry offset fields
Z linkage
servo-height trajectory semantics
```

这些如果在 exact-plan 模式继续由 Process 消费，会违反“Process does no trajectory transformation”。因此不能简单把整个 legacy Tool 原样交给 S2 lowerer。

### 4.4 Required Fix

推荐建立显式：

```text
FrozenToolExecutionRecipe
```

要求：

```text
1. every executable field enters canonical hash
2. every numeric executable field participates in finite validation
3. legacy trajectory-changing Tool fields are rejected or required neutral
4. S2 lowering consumes FrozenToolExecutionRecipe, not raw legacy Tool semantics
```

如果暂时仍复用 `Tool`，至少建立：

```text
toolExecutionRecipeHash()
validateToolExecutionRecipe()
```

禁止继续直接把 `Tool::toTable()` 当作完整 runtime identity。

### 4.5 Required tests

```text
F01-01 changing an executable Tool field changes recipe hash
F01-02 NaN executable field rejects prepare
F01-03 missing explicit recipe rejects
F01-04 __fallback__ recipe rejects
F01-05 path-changing legacy field is rejected / neutral-only
F01-06 mutation after prepare cannot change PreparedDeviceProgram
```

---

## 5. B2S1-F02 — P1：exact-plan pause 语义未保持

### 5.1 旧运行合同

现有 Process pause 语义是：

```text
request Pause
    ↓
current contour completes
    ↓
workflow checkpoint blocks
    ↓
Resume
    ↓
continue next contour
```

`ProcessModule::runPause()` 仍明确表示“pause after completion of current contour”。

### 5.2 当前 exact-plan 路径

当前 exact path 只有：

```text
checkpoint("beforeExactProgram")
    ↓
prepare entire exact program
    ↓
sink.startProgram()
    ↓
waitForQueuedMotion()
```

`waitForQueuedMotion()` 当前只检查 `interrupt.isStopping()`，不检查 `interrupt.isPaused()`。

因此一旦 B2.S2 实现真正硬件 lowering：

```text
contour 1 + contour 2 + contour 3
→ one finite controller program
→ Start
```

若用户在 contour 1 期间请求 Pause，host 当前没有合同阻止 controller 继续 contour 2/3。

当前真实 exact sink 尚未实现、资格也不可用，所以尚未形成实机行为；但 S2 一旦实现 lowering，这个缺口会变成可执行语义。

### 5.3 不要求提前完成 B2.S3

本轮不要求实现完整：

```text
Group reuse
LookAhead
host refill
indeterminate Start
controller pause qualification
```

只需要在 S1 冻结一个 S2 必须遵守的 pause boundary contract。

### 5.4 Required Fix

推荐定义：

```text
PreparedExecutionSection / PauseBoundary
```

首版可直接按 contour boundary 作为 pause-safe boundary。

每个 section 至少要绑定：

```text
section ordinal
contourId
first / last block identity
planHash
runEpoch
```

冻结行为：

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

禁止：

```text
Pause = Stop + automatic replay
Pause ignored after Start
```

### 5.5 Required tests

最小 fake exact sink：

```text
2 cutting contours
start section 1
requestPause()
section 1 completes
assert section 2 not started
requestResume()
section 2 starts

assert:
same PreparedDeviceProgram
same runEpoch
same planHash
no duplicate section
no replay
```

另补：

```text
Pause + Stop => Stop wins
Pause before first Start => no device section starts
```

---

## 6. P2 — 实施文档状态过时

`B2_S1_IMPLEMENTATION.md` 当前仍写“未提交、未推送”，但实际已经提交为：

```text
0b6b3ae0177b550c657b69e0a02fe2e5c38007c1
```

这是事实性文档问题，不阻塞代码。

---

## 7. 测试证据评价

提交内记录：

```text
Ninja Debug full build      PASS
targeted tests              14/14 PASS
elapsed                     12.47 s

Ninja ASan full build       PASS
same targeted suite         14/14 PASS
elapsed                     20.74 s
sanitizer errors            none

architecture check          PASS
git diff --check            PASS
```

这些属于有效的 S1 supporting evidence，但当前新增测试没有覆盖：

```text
complete Tool execution identity
exact-plan pause boundary
```

当前 SHA 没有远端 GitHub Actions/status，因此测试数字属于仓库记录的本地 Windows Debug/ASan 证据。

---

## 8. S1 Release Matrix

| Contract | Result |
|---|---|
| exact FinalMotionPlan cutover | PASS |
| immutable PreparedDeviceProgram | PASS_WITH_F01 |
| no real legacy/raw fallback | PASS |
| stale/default recipe rejection | PASS |
| collision policy | PASS |
| controller qualification fail-closed | PASS |
| existing DeviceCommandQueue authority | PASS |
| Stop/fault ownership | PASS |
| pause contract | OPEN |
| B2.S1 soft-check evidence | PASS_WITH_GAPS |

最终：

```text
B2.S1 = PASS_WITH_PATCH
B2.S2_RELEASE = NO
```

---

## 9. 收口后放行条件

只需关闭：

```text
[ ] F01 complete Tool execution recipe identity
[ ] F02 exact-plan pause boundary contract
[ ] direct targeted regressions pass
[ ] implementation document updated
```

满足后：

```text
B2.S1 = PASS
B2.S2_RELEASE = YES
```

不增加新的正式 Stage Gate，不重跑 B1 Gate。
