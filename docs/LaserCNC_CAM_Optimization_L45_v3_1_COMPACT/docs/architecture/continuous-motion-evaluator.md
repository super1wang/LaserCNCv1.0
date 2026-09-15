# ContinuousMotionEvaluator 设计合同

## 1. Position in Architecture

`ContinuousMotionEvaluator` belongs to CAM/core motion semantics. It is required even when collision verification is disabled.

It provides a single mathematical interpretation of the final block so that optimizer, simulation, controller-equivalence qualification and later collision certification do not each invent a different interpolation model.

## 2. Conceptual API

```text
evaluate(block, u, context) -> EvaluatedMotionState
bound(block, u0, u1, context) -> MotionIntervalBound
```

`EvaluatedMotionState` minimally contains:

- physical-layout axes;
- world TCP where defined;
- table-zero/reference TCP where defined;
- tool/beam direction/process frame;
- source parameter mapping if available.

`MotionIntervalBound` supplies conservative bounds needed by segment merge and later collision certification. Exact type layout is implementation-local unless it crosses existing public module boundaries.

## 3. Models

### AxisLine / PhysicalAxes linear

Interpolate declared physical axes according to the block interpolation model, then use the machine kinematic model for derived TCP/world state.

### RTCP Line

Interpolate according to the **declared controller-equivalent RTCP command model**, not by independently linearly blending a solved physical pose and a world TCP. Controller equivalence must be qualified before the block model can be used for real-machine merge decisions.

### Native Arc/Cylinder

Only define a real-execution evaluator after controller interpolation semantics are qualified. Before that, native forms may exist as CAM candidates/preview primitives but must lower to a qualified equivalent or remain inadmissible for real execution.

## 4. Consumers

- adaptive resampling;
- Full5D merge;
- DOF-reduction residual/equivalence tests;
- Simulation/Preview;
- PhysicalAxes vs RTCP semantic qualification;
- collision certificate builder in Batch B4.

## 5. Failure Rule

If the evaluator cannot conservatively model a proposed real-execution interpolation:

- do not certify/merge based on that interpolation;
- keep the previous valid optimized representation;
- mark the candidate inadmissible for real execution if exact/qualified lowering is unavailable;
- do not guess using endpoint interpolation.

## 6. Invariants

- one block/model version has one evaluator semantic;
- simulation and merge use the same evaluator;
- collision later uses the same evaluator;
- controller-mode-specific assumptions are covered by capability/model hash;
- no point-only sample is treated as proof of an entire continuous interval without a conservative bound/refinement rule.
