# FinalMotionPlan / Proof / Prepared Program State Machine

## 1. CAM Plan States

```text
NoToolpath
Dirty
GeometryReady
Solving
Solved
Optimizing
Selecting
MotionPlanReady
Stale
Failed
```

Legal primary path:

```text
Dirty -> Solving -> Solved -> Optimizing -> Selecting -> MotionPlanReady
```

`Selecting` includes ControllerMotionMode/capability admissibility. The candidate is fixed before `MotionPlanReady` becomes visible.

## 2. Collision Proof Attachment State

Orthogonal to the motion plan:

```text
NotRequested
Pending
CurrentSafe
CurrentCollision
CurrentUnknown
Stale
Failed
```

Policy interpretation:

```text
Disabled: NotRequested is normal and executable if non-collision gates pass
Optional: any proof state is diagnostic; missing/stale/unknown does not block commissioning
Required: only exact-current eligible proof allows execution
```

Proof attachment is a separate atomic publication referencing exact immutable plan identity; it never replaces blocks.

## 3. Prepared Execution State

```text
Unprepared
Preparing
Prepared
Submitting
Started
Running
Stopping
Completed
Faulted
Indeterminate
```

`Prepared` freezes planHash/contextHash/controller mode/capability/tool/process/IO/profile/runEpoch.

Changing source, controller mode compatibility, capability semantics, tool/process values or relevant profile semantics invalidates `Prepared` before Start.

## 4. Publication Linearization

Owner thread:

```text
capture immutable context
 -> worker solve/optimize/select
 -> owner revalidate generation/revisions/context/cancel
 -> atomic FinalMotionPlan publication
 -> notify consumers
```

No notification before the new complete semantic snapshot is visible.

## 5. Invalidation

Whole plan/reselection invalidation includes:

- geometry/toolpath/order;
- setup/WPC;
- kinematics/layout/calibration/limits;
- solver/interpolation model;
- path-affecting optimization policy;
- ControllerMotionMode when compatibility/semantics differ;
- controller capability revision used in candidate admission.

Proof-only invalidation includes collision environment/package/overlay changes **only when they do not alter motion selection**.

Prepared-program invalidation includes tool/process/IO/profile/controller state changes whose values are frozen for execution.

## 6. Concurrency / Failure

- workers operate on detached immutable inputs;
- late stale optimizer results do not commit;
- late collision proof attaches only on exact plan identity;
- Process captures committed plan by value/immutable snapshot;
- execution plan is not swapped after Start;
- indeterminate Start outcome latches Indeterminate, drives safe stop/output state, and is never automatically replayed.
