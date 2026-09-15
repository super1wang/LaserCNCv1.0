# CAM 轨迹规划系统设计方案 v3.1 FINAL — Execution-Compact

> Design Authority / Final Freeze
>
> Repository: `super1wang/LaserCNCv1.0`
> Branch: `main`
> Planning baseline HEAD: `4254db59696193963392e2946132cf614c8def26`
> Audit date: 2026-09-14

## 1. Problem / Goal

当前仓库已经形成较清晰的 CAM → committed execution snapshot → Process → GTN 架构，但真实加工执行仍以离散点序列为主要执行几何，碰撞状态又被当前 Process preflight 硬编码成事实前置。因此若直接叠加“删点、降维、圆柱模式、RTCP/非 RTCP 双路径”，会出现三类高风险：

1. CAM 优化轨迹与 Process 实际执行轨迹不一致；
2. `CollisionVerificationMode=Disabled` 仍被旧 collision gate 阻断；
3. `Full5D` / ReducedDOF 与 RTCP / PhysicalAxes 两个概念混在一个枚举或 fallback 中，导致控制器模式改变时偷偷改变轨迹。

本批次最终目标：

- 在**不加载机台几何模型、不启用碰撞检测**的情况下完成完整 CAM → 五轴求解 → 优化 → Process → GTN 验证；
- 对完整五轴 `Full5D` 轨迹执行与降维轨迹同等级的优化；
- 建立唯一的 `FinalMotionPlan` 执行真相；
- 将“轨迹运动类别”“控制器坐标/RTCP 模式”“碰撞验证策略”冻结为三个正交维度；
- 先完整五轴求解和优化，再做物理自由度分析/降维；
- 在 FinalMotionPlan 已选定后，根据 `ControllerMotionMode` 做**纯编码 lowering**，不得重新 IK、重排、删点、换候选；
- 碰撞认证作为 FinalMotionPlan 的可选后置消费者；
- 保持现有 CAM 权威、Process 不重新求解几何、设备调用经现有 `DeviceCommandQueue` 的责任边界。

明确不在本批次解决：

- 自动碰撞绕障/重规划；
- 替代 GTN 的 servo-cycle 插补器；
- 通用 NURBS NC native emitter；
- 未经过程资格验证的任意 Z 保持；
- 未经 SDK/HIL 资格验证的 GTN 圆柱/极坐标/特殊 native 插补默认启用。

## 2. Frozen Pipeline

```text
CAD / OCC source geometry
        |
        v
Geometry normalization / primitive recovery
        |
        v
Complete kinematic solve / baseline Full5D
        |
        v
Full5D Pose Optimizer
 unwrap / IK continuity / orientation / resample / 5D merge
        |
        v
Candidate generation
 Full5D + Reduced4D + 3D + 2D + SingleAxis + optional Z-hold
        |
        v
Candidate admissibility + deterministic selection
 process / accuracy / dynamics / limits
 + selected ControllerMotionMode / controller capability
        |
        v
+------------------- FinalMotionPlan -------------------+
| semantic blocks / physical solved poses / tcpMcs      |
| active-axis intent / process events / fences / hashes |
+-------------------------------------------------------+
        |                                  |
        |                                  +--> optional Collision Certifier
        |                                       Disabled / Optional / Required
        v
Controller Lowering
  PhysicalAxes OR RTCP
        |
        v
PreparedDeviceProgram
 frozen tool / IO / profile / plan identity
        |
        v
existing DeviceCommandQueue
        |
        v
GTN
```

### Key rule

**RTCP on/off does not select a different CAM planner.** It participates in candidate admissibility before publication and determines the final command encoding after publication.

If `ControllerMotionMode` changes and the currently published plan is no longer compatible, CAM must reselect/recompile and publish a new plan identity. Process MUST NOT silently replace a ReducedDOF block with another Full5D path after publication.

## 3. Three Orthogonal Policy Dimensions

```cpp
enum class MotionClass : uint8_t {
    SingleAxis,
    Coordinated2D,
    Coordinated3D,
    Reduced4D,
    Full5D
};

enum class ControllerMotionMode : uint8_t {
    PhysicalAxes,
    RTCP
};

enum class CollisionVerificationMode : uint8_t {
    Disabled,
    Optional,
    Required
};
```

### 3.1 MotionClass

回答：**这条最终运动从物理运动学上需要多少自由度/哪些轴参与？**

### 3.2 ControllerMotionMode

回答：**同一 FinalMotionPlan 应使用什么控制器坐标语义表达？**

- `PhysicalAxes`：发送 CAM 已求得的 physical-layout axis target，经显式 axis mapping 编码为 GTN ACS/轴坐标语义；
- `RTCP`：发送 table-zero reference TCP (`tcpMcs`) + rotary orientation，以 GTN RTCP Group 语义执行。

### 3.3 CollisionVerificationMode

回答：**是否要求对已经选定的 FinalMotionPlan 做几何碰撞认证？**

该策略不参与几何轨迹生成，不允许成为 optimizer 的隐藏依赖。

## 4. Component Boundary

### 4.1 CAM

CAM owns：

- OCC/source geometry；
- contour order；
- complete IK / machine-coordinate solve；
- geometry-space / pose-space optimization；
- candidate generation/admissibility/selection；
- `FinalMotionPlan`；
- `MotionCompilationContext`；
- selected `ControllerMotionMode` compatibility；
- collision policy snapshot；
- optional proof attachment identity。

CAM MUST NOT include vendor SDK types in its public motion contract.

### 4.2 Process

Process owns：

- execution preflight；
- frozen `PreparedDeviceProgram`；
- process IO/tool execution context；
- controller lowering orchestration；
- device command scheduling through the existing queue；
- runtime Stop/fault/state handling。

Process MUST NOT：

- reorder contours；
- redo IK；
- generate a different reduced/full candidate；
- fit arcs/delete points as a hidden optimizer；
- infer `ControllerMotionMode` from mutable UI after run preparation；
- require collision proof when the committed policy is `Disabled` or `Optional`。

### 4.3 ContinuousMotionEvaluator

This is a **core motion-semantics component, not a collision component**.

Required conceptual API:

```text
evaluate(block, u, context)
  -> physical-layout axes + world TCP + beam/process frame

bound(block, [u0,u1], context)
  -> conservative position/orientation/axis intervals or error bounds
```

Consumers include：

- Full5D segment merge；
- adaptive resampling；
- Simulation/Preview；
- PhysicalAxes/RTCP semantic-equivalence qualification；
- later Collision Certifier。

Collision may consume the evaluator; the evaluator MUST NOT depend on collision geometry/backends.

### 4.4 Collision Certifier

- consumes FinalMotionPlan；
- attaches proof to the exact plan identity；
- never changes motion blocks；
- Disabled: not scheduled/constructed/querying backend；
- Optional: diagnostics only, missing/stale/Unknown does not block execution；
- Required: fail closed on missing/stale/Unknown/collision。

### 4.5 Controller Lowering

Lowering owns translation from a **selected, immutable semantic plan** into controller commands.

It MUST NOT:

- run IK；
- choose a different MotionClass；
- choose a different geometric primitive candidate；
- silently raw-fallback；
- silently switch RTCP/PhysicalAxes；
- mutate toolpath ordering or process fences。

## 5. MotionCompilationContext

At minimum freeze/hash:

```text
workspace/generation
source toolpath revision
contour order revision
setup/WPC revision
machine kinematics fingerprint
axis layout/soft limits
calibration revision
tool/nozzle/process recipe revision
optimization policy hash
selected ControllerMotionMode
controller capability/qualification hash
dynamics/profile semantic hash
CollisionVerificationMode
collision environment revision (only proof binding)
algorithm/model version
```

Any context item capable of changing selected motion semantics invalidates/recompiles the plan rather than being read live during Process execution.

## 6. Geometry Classification vs Physical DOF Classification

Geometry stage may classify Line/Arc/Circle/Cylinder/Surface features, but it MUST NOT directly assert C-only or U+C merely from CAD shape.

True reduction order is:

```text
geometry clue
 -> complete IK
 -> optimized Full5D physical reference
 -> active-axis/forward-kinematic analysis
 -> reduced candidate
 -> continuous equivalence/admissibility proof
```

Examples:

- a circular contour is not automatically C-only；
- a cylinder chart does not prove only one linear axis + C is sufficient；
- `tcpMcs` movement does not directly equal physical XYZ movement in RTCP；
- source geometry is used to create candidates, physical solved motion decides admissibility。

## 7. Full5D Is the Optimized Baseline

Every valid five-axis path first produces a Full5D reference and passes through:

1. periodic-angle unwrap with turn preservation；
2. existing IK branch-continuity reuse/audit；
3. bounded orientation smoothing over material arc length；
4. adaptive resampling using position + orientation + rotary/dynamics demand；
5. 5D segment merge using the shared evaluator；
6. feed/dynamics/lookahead-friendly segmentation。

Only then generate reduced candidates.

If no reduced candidate is admissible, the result is **Optimized Full5D**, never raw sampled points.

## 8. Candidate Admissibility and Selection

Hard constraints first:

```text
source/process fences preserved
finite values
position/orientation/process error budgets
axis layout + soft limits
IK branch continuity
process semantics
controller mode/capability compatibility
dynamics/feed semantics representable
```

Collision is NOT a hard constraint in optimizer selection. In `Required` mode it is a post-selection execution certificate gate over the selected plan.

Then deterministic cost, for example:

```text
required stops
estimated material-path time / mode switches
normalized physical axis travel
rotary travel + reversal penalties
command/block count
stable preference for baseline semantics on ties
```

“fewest axes” MUST NOT be the sole objective.

Every rejected candidate records a deterministic rejection reason.

## 9. FinalMotionPlan — Single Execution Truth

Final plan blocks must carry enough semantic data to execute either supported controller mode without Process inventing motion:

- blockId / source spans / contourId / phase；
- MotionClass / activeAxisMask；
- interpolation model/version；
- physical-layout solved endpoints/knots；
- table-zero reference TCP where applicable；
- world TCP/normal/beam semantics where needed for evaluation；
- unwrapped rotary values；
- process events/fences/feed；
- position/orientation/process error proof summary；
- compatible `ControllerMotionMode` and capability hash；
- planHash / blockHash / contextHash；
- collision policy and optional proof attachment key。

Legacy `pointsByContourId` / `nodes` may remain only as derived display/compatibility views tagged `derivedFromPlanHash`; schema2/new execution MUST NOT use them as an independent execution truth.

## 10. Controller Lowering Matrix

| MotionClass | PhysicalAxes | RTCP |
|---|---|---|
| Full5D | baseline supported when explicit physical-layout→GTN mapping is valid | baseline supported via `tcpMcs + rotary` Group semantics |
| Reduced4D/3D/2D/SingleAxis | only if hold/ownership/feed semantics are exactly representable | only if RTCP Group semantics reproduce the intended physical motion exactly |
| native Arc/Cylinder | capability + HIL qualification required | capability + HIL qualification required |

If a combination is unsupported, that candidate is inadmissible **before FinalMotionPlan publication**. Lowering never chooses another candidate.

`activeAxisMask` is not the same thing as Group member axes and not the same thing as safety/collision mask.

## 11. Collision Policy

### Disabled — explicit commissioning policy

- no machine STEP required；
- no `.lmsi`/Job Overlay/Coal/OCCT proof required；
- no edge certificates required；
- no collision backend construction/query；
- current Process hard collision gate must be split so plan readiness remains independently checkable；
- UI/log says `not collision-certified`, never `safe`。

### Optional

- attach proof if available and current；
- warning/diagnostic only；
- missing/stale/Unknown cannot block current commissioning execution；
- never silently promoted/demoted from another mode during a prepared run。

### Required — production fail-closed policy

- exact plan-bound proof required；
- Missing/Pending/Stale/Unknown/Collision blocks；
- cannot auto downgrade to Optional/Disabled。

Current repository production architecture is fail-closed for collision; v3.1 adds Disabled as an explicit commissioning policy rather than silently weakening that production contract. Repository `ARCHITECTURE.md`/config/UI must be updated in Batch B0 when behavior changes.

Always enforced regardless of collision mode:

- kinematic validity；
- finite numeric values；
- explicit axis mapping and soft limits；
- controller connected/fault/enabled state；
- RTCP agreement/capability；
- immutable run plan/hash；
- Stop/E-stop priority；
- process IO ordering。

## 12. PreparedDeviceProgram

Before device submission Process freezes:

```text
final planHash/contextHash
ControllerMotionMode
controller capability revision
effective motion profile semantics
tool/process parameters
laser/gas IO values and ordering
block -> native command mapping
runEpoch / source revision
```

No raw `Tool*`, mutable UI setting or live optimization policy may change the prepared run after this point.

Current behavior that substitutes sanitized defaults after tool lookup failure is NOT an acceptable final execution semantic; preparation must either freeze an explicit valid recipe or block.

## 13. Publication / Invalidation

Owner thread captures immutable context → worker computes → owner revalidates source/context/generation/cancel → **single atomic publication of plan + semantic identity**.

Changing any of these requires a new plan or at least a new compatibility publication according to contract:

- source geometry/order；
- machine kinematics/calibration/setup；
- path-affecting optimization tolerance；
- selected ControllerMotionMode；
- controller capability semantics；
- path/interpolation model。

Proof attachment may publish separately only when it matches exact immutable plan identity; it cannot change the plan.

## 14. GTN Group / Device Authority

- reuse existing `DeviceCommandQueue`; no second SDK worker/queue；
- Group ownership state may be consolidated into a `GroupSession`, but not become a second device authority；
- preserve profile-to-encoder synchronization and existing fault/release protections；
- initial direct axis approach must respect Group ownership/release；
- command filling may be sliced through the queue (e.g. max 64 commands or ~5 ms host preparation budget), but this is a host scheduling budget, NOT a vendor-call execution-duration guarantee；
- first safe version fully builds finite command list before Start；
- never arbitrarily split a laser-on contour into hard-stop lists；
- Start outcome indeterminate ⇒ outputs safe/off, stop/latch indeterminate, no automatic replay。

## 15. Error Budget Ledger

Keep separate ledgers for:

- source/CAD discretization；
- primitive fit；
- position/tangent error；
- orientation angular error；
- DOF-reduction equivalence error；
- controller interpolation error；
- calibration/clamping/runout；
- Z focus/standoff/process error；
- numerical safety padding。

Never add degrees directly to millimeters. Orientation may remain angular or be conservatively converted to beam/lateral error with a documented lever arm/model.

## 16. Key Invariants

- **INV-01** CAM is the sole authority for ordered execution geometry/motion selection.
- **INV-02** Process and lowering never redo IK or select a different candidate.
- **INV-03** MotionClass, ControllerMotionMode and CollisionVerificationMode are orthogonal.
- **INV-04** Full5D always receives the same optimization quality path before reduction attempts.
- **INV-05** Reduced candidate failure returns Optimized Full5D, never raw points.
- **INV-06** Controller-mode incompatibility makes a candidate inadmissible before publication; no post-publication fallback.
- **INV-07** RTCP toggle that changes compatibility invalidates/recompiles the prepared plan.
- **INV-08** `tcpMcs` is reference TCP, not physical XYZ and not generic controller-axis coordinates.
- **INV-09** periodic rotary turn count/direction is preserved; no accidental modulo/shortest-path erasure.
- **INV-10** display sampling never affects execution/safety semantics.
- **INV-11** `CollisionVerificationMode=Disabled` has zero collision-backend dependency.
- **INV-12** Required collision mode is fail-closed and bound to exact final primitive/interpolation identity.
- **INV-13** ContinuousMotionEvaluator semantics are shared by merge/sim/equivalence/collision; collision is only a consumer.
- **INV-14** activeAxisMask != Group membership != collision/safety mask.
- **INV-15** tool/process/IO/profile semantics are frozen in PreparedDeviceProgram before device submission.
- **INV-16** all vendor calls continue through existing DeviceCommandQueue/authority.
- **INV-17** Stop/fault priority and device-health gates are independent of optimization/collision mode.
- **INV-18** native GTN special modes require capability qualification; unknown is not supported.
- **INV-19** certificates bind complete program/block/interpolation/context identity; old proof cannot attach to changed motion.
- **INV-20** no point-safe result is promoted to continuous-volume/segment-safe without a continuous bound/proof.
- **INV-21** no infinite retry, online OCCT fallback or hidden unbounded resource path is introduced.
- **INV-22** source spans/process fences/laser IO remain traceable through every optimization.
- **INV-23** optimization reports distinguish modeled/predicted/software/HIL/physical-quality evidence.
- **INV-24** an execution-start outcome that is indeterminate is never auto-replayed.

## 17. Failure Semantics

- geometry optimizer cannot improve safely → retain source-equivalent geometry + reason；
- Full5D optimizer no material improvement → publish `Full5D + Optimized(no-change)` if invariants hold；
- optimizer cannot establish required semantic bound → reject that transformation, keep prior valid optimized stage；
- reduced candidate rejected → keep Optimized Full5D；
- selected ControllerMotionMode cannot represent candidate → candidate rejected before publication；
- lowering mismatch after publication → block/error, never choose another motion；
- collision Disabled → no proof is normal；
- collision Optional → proof failure/stale is diagnostic；
- collision Required → proof failure/missing/stale blocks；
- tool/process freeze fails → block run preparation；
- vendor Start result indeterminate → safe stop/latch/no replay。

## 18. Rejected Alternatives

Forbidden:

- `Full5D` / `Reduced4D` as canonical MotionClass names that encode controller mode into motion class；
- RTCP-on planner and RTCP-off planner producing independent execution geometry；
- Process-side candidate fallback after plan publication；
- collision backend driving or rewriting optimizer path；
- forcing machine STEP/safety package to run trajectory optimization；
- using old raw points as silent real-machine fallback；
- creating a second device thread/command queue；
- generic `%360`/shortest path on established unwrapped rotary motion；
- per-point threshold dropping of small axis increments；
- hardcoding guessed focus/Z tolerance；
- mixing linear and angular tolerances into one scalar without a model；
- treating controller endpoint agreement as proof of whole-segment equivalence。

## 19. Release Boundary

This design can be released to implementation when:

- current local working tree is confirmed compatible with baseline or differences are recorded;
- Batch B0 first removes/splits the current hard collision execution gate without weakening non-collision device safety, freezes FinalMotionPlan + three orthogonal policy axes, and establishes the shared evaluator/observability prerequisites before transformations are trusted;
- no work package invents a second state truth or controller fallback.

Production/hardware release remains a later gate; development release does not imply collision safety, HIL qualification or process-quality qualification.

## 20. Core Data Model Contract

The following objects are architecture-level concepts. Private member layout remains an implementation choice, but ownership/lifetime/immutability do not.

| Object | Creator / Owner | Mutability & lifetime | Cross-thread / cross-callback rule | Invalidated by |
|---|---|---|---|---|
| `MotionCompilationContext` | CAM owner thread captures | immutable value for one compile attempt | detached/copyable worker input; no live UI/service pointers | any captured revision/hash/policy/mode change |
| `SourceSpan` | CAM geometry/primitive stage | immutable trace record inside candidate/block | may cross worker boundary by value/owned container | source geometry/order revision change |
| `MotionBlock` | CAM optimizer/candidate builder | immutable once part of selected plan | worker may build private candidates; consumers read only after publication | rebuild/new plan only |
| `FinalMotionPlan` | CAM publication authority | immutable published execution truth | owner-thread atomic publication; Process/Simulation read snapshot only | new plan publication/context mismatch |
| `MotionOptimizationReport` | optimizer | immutable diagnostics bound to input/output plan identity | read-only diagnostics; never drives hidden runtime replanning | plan/context change |
| `PreparedDeviceProgram` | Process prepare stage | immutable for one run epoch | no mutable Tool/UI/config pointer after prepare; device queue consumes exact prepared commands | plan/mode/capability/recipe/profile/run-epoch mismatch |
| `MotionBlockCertificate` / proof attachment | CAM collision certifier | immutable proof bound to exact plan/block/model/context | may publish after plan only if exact identity matches; cannot mutate motion | any bound identity/environment change |
| controller capability/qualification snapshot | controller integration / qualification authority | immutable revisioned snapshot | candidate admission and lowering must use the same revision/hash | SDK/firmware/config/qualification revision change |

There is one published motion truth: `FinalMotionPlan`. Derived legacy point/node projections, preview samples, collision proof objects and lowered command buffers are consumers/projections and MUST NOT become peer motion truths.

## 21. State Machine and Linearization Points

Detailed transitions are defined in [motion-plan-state-machine.md](motion-plan-state-machine.md). The architecture-level state flow is:

```text
Source/Context Captured
  -> Compiling (private candidates)
  -> Selected (private exact candidate)
  -> Commit Validation
  -> Published FinalMotionPlan          [CAM motion linearization point]
  -> PreparedDeviceProgram              [Process execution-freeze point]
  -> Queued/Sealed
  -> Started                            [device execution admission point]
  -> Running
  -> Completed | Stopped | Faulted | ExecutionIndeterminate
```

Collision proof is an orthogonal attachment state:

```text
NotRequested   (Disabled)
Requested -> Pending -> CertifiedSafe | Collision | Unknown/Stale
```

Rules:

- CAM publication is the only point at which a newly selected motion becomes visible as execution truth.
- A late worker result failing generation/revision/context/cancel validation never publishes.
- A proof attachment may become visible after motion publication only when its exact plan/block/model/context identity matches; it never rewrites the plan.
- `PreparedDeviceProgram` freezes the run; later UI/tool/mode/capability mutations stale the preparation instead of mutating it in place.
- once vendor Start outcome becomes indeterminate, the run enters a terminal/recovery-required state and is never auto-replayed.

## 22. Concurrency Semantics

- The owner/UI/CAM authority thread captures immutable compile context and owns the publication swap.
- Workers receive detached data only; they do not mutate active documents, Process runtime, controller objects or published FinalMotionPlan.
- Commit arbitration revalidates generation, source/order/setup/tool/calibration/mode/capability/dynamics/policy revisions and cancellation immediately before the single publication point.
- A stale/cancelled worker may finish computation, but its result is discarded; no partial block/proof publication is allowed.
- Collision certification may run asynchronously after plan publication. Late proof attaches only through exact identity comparison; mismatched proof is discarded/staled.
- Process preparation consumes one committed immutable snapshot and freezes an immutable `PreparedDeviceProgram`; Process does not observe mutable CAM working state while executing.
- All supplier SDK calls remain serialized through the existing `DeviceCommandQueue` / device authority. Stop keeps its existing priority semantics; the design does not claim software can preempt a vendor call already executing inside the SDK.
- Do not hold a CAM/domain mutex across vendor SDK calls or callbacks. Vendor lifetime/fault/release behavior stays in the existing Process/device integration boundary.
- Group lifecycle consolidation may centralize state but must not create a second scheduler, execution registry or device ownership authority.


## 18. v3.1 Execution-Compact Policy

The architecture above is unchanged from v3 FINAL. v3.1 only compresses implementation/validation orchestration.

```text
B0 Foundation + Contracts + Evaluator
  -> Astra stage review
B1 CAM Motion Compiler (geometry -> Full5D -> reduction -> publication)
  -> Astra stage review
B2 Process + GTN Execution Stack
  -> Astra stage review
B3 No-Collision Commissioning + Motion Software Closure
  -> Astra stage review
B4 Collision Reintegration + Production Qualification
  -> Astra final review
```

Rules:

- Internal workstreams are implementation slices, **not acceptance gates**.
- The executor continues automatically after quick/local checks pass.
- Full regression/evidence synthesis is run once per batch, not once per subtask.
- Astra reviews only the completed batch delta plus impacted frozen contracts; it does not re-review all prior batches unless an interface or invariant changed.
- Any ESCALATE, contract ambiguity, safety-invariant break, or persistent test failure stops the affected branch immediately.
