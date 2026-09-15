# Motion Parameters / Policy Ownership — v3 FINAL

## 1. Parameter Domains

### CAM optimization policy

- `trajectory.optimizationMode = Off | Conservative | Full`
- position/chord tolerance source (reuse existing authoritative CAM/process tolerance rather than inventing a new default)
- `trajectory.orientationToleranceDeg`
- `trajectory.maxRotaryStepDeg`
- `trajectory.enableDofReduction`
- `trajectory.enableLaserZHold`
- Z-hold focus/standoff/contour envelope references
- primitive/resampling/merge bounded budgets
- rotary reversal/soft-limit cost weights

### Execution policy

- `ControllerMotionMode = PhysicalAxes | RTCP`
- `CollisionVerificationMode = Disabled | Optional | Required`

### Controller expert parameters

Reuse existing GTN ownership/settings for:

- Group motion/orientation constraints;
- per-axis vel/acc/dec/jerk/dvMax;
- Group smooth time/k;
- lookAheadNum/time/radiusRatio;
- command velocity reference axes/rotary ratios;
- RTCP validation tolerance/stride;
- Group/list/axis mapping configuration.

Do not introduce duplicate parameter owners.

### Commissioning locked

- axis scale/resolution/direction;
- machine kinematic topology/centers/vectors;
- setup/calibration;
- controller/firmware/SDK identity;
- hard/soft limits.

## 2. Current Priority Commissioning Profile

The profile intentionally disables geometry collision verification while retaining all non-collision safety:

```text
trajectory.optimizationMode = Full
trajectory.enableDofReduction = true
trajectory.enableLaserZHold = false until dedicated qualification
collision.verificationMode = Disabled
controller.motionMode = RTCP or PhysicalAxes only when that cell is qualified
```

No new numeric machining tolerance is hardcoded by this planning package. Position/orientation/process error limits must come from the repository's authoritative CAM/process settings or explicit commissioning recipe and be captured in `MotionCompilationContext`.

Relaxed orientation smoothing is off unless an explicit nonzero process tolerance is provided.

## 3. Hash / Invalidation Rules

### Must affect FinalMotionPlan/context identity

- source/order/setup/kinematics/calibration changes;
- path-affecting optimizer tolerances/policies;
- tool/process values that affect motion/feed selection;
- selected ControllerMotionMode when command semantics or candidate compatibility differ;
- controller capability/qualification revision used by admission;
- interpolation model/version;
- dynamics semantics when candidate admissibility/trajectory shape depends on them.

### PreparedDeviceProgram identity at minimum

Even when they do not require CAM geometric recompilation, effective controller smooth/lookahead/profile settings used for a run must be frozen/logged/hashable in PreparedDeviceProgram.

If a “runtime tuning” value changes actual interpolation semantics assumed by the evaluator/certifier, it is not merely diagnostic and must invalidate the relevant plan/model/proof identity.

## 4. UI Rules

Collision Disabled:

- clearly display “碰撞验证关闭 / 未碰撞认证”;
- do not display “Certified Safe”;
- do not show missing machine collision model/package as an execution error;
- do not hide device/axis/kinematic/limit faults.

Full5D optimizer no material change:

- report `Optimized / no material change`, not failure.

Controller mode change:

- if current plan/prepared program compatibility changes, mark stale and request recompile/reselection;
- never silently re-encode a different candidate.

## 5. Parameter Evidence

Every optimizer/controller run report should record requested/effective/source/unit/revision for values that materially affect results, so A/B tests are reproducible and UI value does not masquerade as device-effective value.
