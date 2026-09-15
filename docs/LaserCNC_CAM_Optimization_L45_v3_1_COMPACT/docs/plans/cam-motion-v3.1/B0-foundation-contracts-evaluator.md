# B0 — Foundation / Collision Policy / FinalMotionPlan / Evaluator

## Goal

In one batch, establish the local repository baseline and all semantic contracts that later path-changing code depends on. This batch replaces old WP00–WP02 and has **one formal acceptance at the end**.

## Entry

- complete local worktree available;
- record local HEAD/dirty/build/SDK facts before claiming build success;
- if CAM/Process/GTN contracts materially differ from planning baseline, document delta and ESCALATE only if incompatible.

## Workstream 1 — Collision policy split and local baseline

Implement/freeze `CollisionVerificationMode {Disabled, Optional, Required}` in immutable execution context. Split Process plan/runtime readiness from collision eligibility.

Required behavior:

- Disabled does not require machine STEP, `.lmsi`, Job Overlay, collision complete/Pending state, or certificates;
- Disabled constructs/queries no collision backend;
- Optional is diagnostic only;
- Required stays fail-closed;
- controller connectivity/status/fault/axis enable, kinematics/layout/soft limits, Stop/E-stop remain independent gates;
- approach/travel eligibility cannot hide a collision-only prerequisite;
- UI/log clearly says Disabled = not collision-certified;
- repository architecture/config/UI policy updated together with code.

Forbidden: fake `CertifiedSafe`, fake `complete=true`, second DeviceCommandQueue, disabling non-collision safety.

## Workstream 2 — Single FinalMotionPlan contract

Freeze orthogonal contracts:

```cpp
MotionClass { SingleAxis, Coordinated2D, Coordinated3D, Reduced4D, Full5D }
ControllerMotionMode { PhysicalAxes, RTCP }
CollisionVerificationMode { Disabled, Optional, Required }
```

Final blocks/context must carry enough information for later exact execution/validation: physical-layout solved motion, source spans/fences/feed, unwrapped rotary intent, necessary tcpMcs/world/process semantics, interpolation/model identity, selected controller-mode/capability compatibility and stable hashes.

Legacy nodes/points become derived/legacy views, not a second writable execution truth. SDK structs stay out of core CAM contracts.

## Workstream 3 — Shared ContinuousMotionEvaluator + observability

Provide collision-independent block semantics conceptually equivalent to:

```text
evaluate(block,u,context) -> physical axes + derived world TCP/process frame
bound(block,[u0,u1],context) -> conservative/refinable motion/error bounds
```

Rules:

- PhysicalAxes line derives TCP from declared physical-axis interpolation;
- RTCP line uses only declared/qualified controller interpolation semantics;
- never independently linearly interpolate solved axes and world TCP and call that equivalent;
- evaluator/model version participates in plan identity;
- optimizer report records before/after knots/blocks, axis spans, rotary travel/reversal, max deviations, selected class/mode, rejected candidate reasons and command estimate;
- evaluator works fully with collision Disabled and zero backend calls.

## Soft checkpoint B0.S1 — one local run, no review

Run the accumulated targeted suite for:

- enum/config/default/migration/serialization;
- Disabled/Optional/Required Process gate;
- backend call counter;
- FinalMotionPlan identity determinism/staleness;
- evaluator endpoint + midpoint/continuous counterexamples.

If it passes, continue automatically. No long report.

## Stage Gate B0

Run validation rows V-001..V-006 and directly impacted existing safety/contract regressions once.

Boolean exit:

- [ ] local baseline and SDK/build reality captured;
- [ ] Disabled no longer depends on collision resources/proof and backend call count is zero;
- [ ] Required remains fail-closed;
- [ ] non-collision runtime safety preserved;
- [ ] FinalMotionPlan is the only writable motion truth;
- [ ] three policy dimensions are orthogonal;
- [ ] evaluator is collision-independent and model/version is identity-bound;
- [ ] no unresolved ESCALATE.

Then request Astra R0. B1 starts only after `PASS` or closure of `PASS_WITH_PATCH`.
