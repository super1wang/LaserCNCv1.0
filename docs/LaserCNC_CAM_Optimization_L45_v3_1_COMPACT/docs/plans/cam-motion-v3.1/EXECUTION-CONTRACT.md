# Execution Contract — Compact Authority

This file is the short-form authority for high-capability execution models. If a detail conflicts with the full architecture design, the full design wins.

## Frozen pipeline

```text
OCC/source geometry
 -> normalize / primitive recovery
 -> complete IK / baseline Full5D
 -> Full5D optimizer
 -> adaptive resampling / 5D merge
 -> reduced candidates / optional restricted Z-hold
 -> controller-mode admissibility + deterministic selection
 -> atomic FinalMotionPlan
 -> Process PreparedDeviceProgram
 -> PhysicalAxes OR RTCP lowering
 -> existing DeviceCommandQueue
 -> GTN

FinalMotionPlan -> optional collision certifier (Disabled/Optional/Required)
```

## Three orthogonal dimensions

```cpp
MotionClass            // physical degrees of freedom
ControllerMotionMode   // PhysicalAxes | RTCP
CollisionVerificationMode // Disabled | Optional | Required
```

Never encode RTCP into MotionClass. Never let collision policy choose the trajectory.

## Execution truth

- CAM owns contour order, IK, optimization, candidate generation/selection and FinalMotionPlan.
- Process does not fit, simplify, re-IK, reduce DOF, or choose a fallback trajectory.
- legacy points/nodes are derived display/compatibility views once cutover completes.
- Controller lowering is encoding only.

## Full5D baseline

Every simultaneous 5-axis path gets Full5D optimization before reduction:

- preserve unwrapped turns/direction;
- reuse/audit existing IK branch continuity;
- bounded orientation denoise/smoothing only under explicit tolerance;
- adaptive position+orientation+rotary/dynamics resampling;
- evaluator-backed 5D merge;
- lookahead-friendly segmentation.

Reduction failure returns **Optimized Full5D**, never raw sampled points.

## Reduction

Geometry is only a clue. A circle is not automatically C-only. Generate reduced candidates from Optimized Full5D physical motion, then prove continuous equivalence and controller-mode representability. Preserve cumulative small motion and periodic turns.

## RTCP vs PhysicalAxes

- `PhysicalAxes`: encode CAM solved physical-layout axes through explicit mapping.
- `RTCP`: encode table-zero reference TCP (`tcpMcs`) + rotary orientation using qualified GTN semantics.
- RTCP on/off does not select a different CAM planner.
- unsupported MotionClass×mode cell is rejected before publication; lowering must not switch trajectory.

## Collision policy

- `Disabled`: no machine geometry/package/overlay/certificate prerequisite; zero collision backend calls; visible “not collision-certified” status.
- `Optional`: diagnostic proof may attach but does not gate execution.
- `Required`: fail closed on missing/stale/Unknown/collision and certify the exact final interpolation semantics.

Collision mode never disables controller connectivity/fault/axis-enable/limits/Stop safety.

## Evaluator

One collision-independent ContinuousMotionEvaluator provides block `evaluate(u)` and conservative/refinable `bound([u0,u1])`. Optimizer, simulation and later collision proof reuse it. Endpoint agreement alone is not continuous equivalence.

## Publication / lifetime

Capture immutable compilation context; compute detached; owner thread revalidates generation/revisions/cancel/mode/capability; publish plan+context+hash atomically. Late proof may attach only to exact matching identity.

## Prepared run

Process freezes planHash/contextHash/controller mode/capability/tool/process/IO/profile/runEpoch before device submission. Missing/stale real-run recipe blocks preparation; no sanitized defaults.

## GTN/device

Reuse existing DeviceCommandQueue and device authority. Preserve profile→encoder sync, Group release/recovery, Stop priority, safe shutdown and no-replay on indeterminate Start. Unknown native/special interpolation remains disabled.

## Mandatory ESCALATE conditions

- required semantics cannot fit the single FinalMotionPlan truth;
- evaluator/controller interpolation cannot be bounded where optimization relies on it;
- Disabled collision mode would require weakening device safety;
- Process needs to invent motion;
- a second SDK/device queue appears necessary;
- real controller behavior contradicts the model.
