# Configuration Migration — v3.1 Execution-Compact

## 1. Logical Settings

The final design requires logical ownership for:

```text
trajectory.optimizationMode
trajectory.position/orientation/process tolerance sources
trajectory.maxRotaryStepDeg
trajectory.enableDofReduction
trajectory.enableLaserZHold
controller.motionMode = PhysicalAxes | RTCP
collision.verificationMode = Disabled | Optional | Required
controller capability/qualification revision
```

Exact TOML keys may reuse existing project naming; do not introduce duplicate settings when an authoritative GTN/RTCP setting already exists.

## 2. Existing Collision Boolean

If legacy configuration exposes a boolean equivalent to collision enabled:

```text
false -> migration policy must be explicit
true  -> Required
```

Because the current repository architecture treats real-machine production collision as fail-closed, do **not** silently migrate a production installation from `true` to Optional/Disabled.

For legacy `false`, the product migration must distinguish whether it historically meant PureSimulation/unsupported/commissioning. If that distinction is not available, migration requires explicit confirmation/profile selection rather than silently granting real-machine Disabled execution.

New runtime authority is the committed enum/policy snapshot, not a mirrored boolean.

## 3. Controller Motion Mode

Reuse the existing RTCP enable/config authority where possible and project it into canonical:

```text
RTCP enabled  -> ControllerMotionMode::RTCP
RTCP disabled -> ControllerMotionMode::PhysicalAxes
```

But changing this value after FinalMotionPlan/PreparedDeviceProgram creation follows the v3 invalidation/reselection rules; it is not merely a live controller toggle.

## 4. Safe Feature Defaults

- numerical normalization may be enabled when strict-equivalent;
- relaxed orientation smoothing requires explicit process tolerance;
- Full5D merge requires evaluator/model qualification and bounded tolerance;
- DOF reduction may be feature-gated during staged rollout;
- process-relaxed Z-hold default off until qualified;
- native arc/cylinder/persistent Group default off until required qualification;
- production collision policy remains Required unless explicitly changed by product policy;
- commissioning Disabled must be visibly selected/labeled.

## 5. Policy Snapshot

At compilation/preparation, freeze effective values/revisions into `MotionCompilationContext` and `PreparedDeviceProgram`. Later UI changes affect a subsequent compile/run and cannot mutate an already prepared execution.
