# Current Repository → v3.1 Execution-Compact Migration Strategy

## 1. Staged, Not Big-Bang

```text
Phase A  add contracts/evaluator/reports while legacy execution still owns real motion
Phase B  generate FinalMotionPlan + compare against legacy, no hidden execution switch
Phase C  Simulation/diagnostics consume exact FinalMotionPlan
Phase D  Process builds PreparedDeviceProgram from FinalMotionPlan
Phase E  GTN lowering executes PhysicalAxes/RTCP encoded blocks
Phase F  legacy point execution loses real-machine eligibility
Phase G  collision certifier rebinds to exact final block semantics
```

## 2. Key Surfaces

### ToolpathExportSnapshot

Keep `pointsByContourId` as catalog/geometry/debug projection where needed. It is not the new execution truth.

### CamMotionPlanSnapshot

Prefer extending the existing type/family into typed blocks/context/identity. Legacy nodes are derived compatibility views only.

### Collision policy

Current repository production architecture is fail-closed. Batch B0 introduces an explicit commissioning Disabled policy and updates code + architecture/config/UI together. Do not hide this architecture change behind an empty proof.

### NormalCuttingManager

Transition from copied executable sampled points to immutable plan block references/value slices and PreparedDeviceProgram. Freeze tool/process/IO/profile/mode; missing tool/recipe blocks preparation rather than substituting a silent real-run default.

### Controller mode

The same CAM optimization pipeline produces candidates. Current `ControllerMotionMode`/capability participates in candidate admissibility **before publication**. Process/GTN lowering only encodes the selected plan:

```text
PhysicalAxes -> solved physical-layout axes -> explicit GTN mapping
RTCP         -> tcpMcs + rotary -> GTN RTCP Group semantics
```

No post-publication trajectory fallback.

### IMotionCommandSink / lowering facade

Add the smallest typed/block or lowered-batch interface needed. Keep GTN SDK/controller-specific semantics out of CAM core and all vendor calls under the existing DeviceCommandQueue.

### Collision

B0–B3: Disabled commissioning path is independent.  
B4: Required proof uses the shared final interpolation evaluator and complete identity.

## 3. A/B / Rollback

Before Batch B2 cutover, an explicit legacy baseline run may exist for A/B comparison. After Batch B2 Exit, legacy raw execution must not be a silent fallback. Any commissioning rollback switch must be explicit, prominently logged, and excluded from claims that the new plan path executed.
