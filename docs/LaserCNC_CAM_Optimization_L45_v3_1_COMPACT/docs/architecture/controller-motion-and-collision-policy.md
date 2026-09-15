# Controller Motion Mode 与 Collision Policy 正交关系

## 1. Frozen Rule

以下三个问题必须分别回答：

1. **MotionClass** — 轨迹在物理运动学上需要哪些自由度；
2. **ControllerMotionMode** — 控制器以 PhysicalAxes 还是 RTCP 坐标语义接受该运动；
3. **CollisionVerificationMode** — 是否要求几何碰撞认证。

任何一个都不能编码进另一个枚举名中。

## 2. Selection Sequence

```text
complete IK
 -> optimized Full5D
 -> reduced candidate generation
 -> evaluate candidate process/accuracy/dynamics
 -> evaluate selected ControllerMotionMode compatibility/capability
 -> deterministic selection
 -> FinalMotionPlan publish
 -> optional collision proof
 -> pure controller lowering
```

Collision never chooses the motion candidate. Lowering never chooses the motion candidate.

## 3. Mode Changes

- RTCP on/off changed before compile: participates in candidate admissibility and plan hash.
- RTCP on/off changed after publication: current prepared program becomes stale; CAM recompile/reselection required when semantics/compatibility differ.
- collision Disabled/Optional/Required changed: requires a new committed policy snapshot; Required additionally requires current exact proof before execution.
- UI live state cannot silently override the immutable run policy.

## 4. No-machine-model Commissioning

Allowed architecture state:

```text
valid machine kinematics + calibration
controller mode selected
collision = Disabled
machine geometry package = absent
job overlay = absent
FinalMotionPlan = valid
Process preflight = passes non-collision gates
```

This state is **not collision-safe certification** and must be visibly reported as such.
