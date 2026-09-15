# Final Remote Repository Audit — 2026-09-14

## 1. Audit Baseline

```text
Repository: super1wang/LaserCNCv1.0
Branch: main
HEAD: 4254db59696193963392e2946132cf614c8def26
Commit time: 2026-09-14T04:08:15Z
Tree: 781c39c091622c2e97d30ef83b0a792f37c7f9fb
```

The remote HEAD did not drift during the final planning review.

## 2. Scope Reviewed

Planning was cross-checked against the repository areas governing:

- repository architecture / CAM authority boundaries;
- CAM export/motion/collision/travel contracts;
- Process cutting preflight and normal cutting flow;
- existing five-axis/tube solver facts from the same HEAD;
- RTCP reference-TCP semantics;
- current GTN Group/RTCP/non-RTCP behavior;
- existing device command queue and stop/fault authority;
- optimizer/certificate interpolation mismatch risk;
- package/build visibility in the public remote snapshot.

This audit is a source/architecture review, not a successful local C++ build/HIL result.

## 3. Confirmed Repository Facts Relevant to the Final Plan

### FACT-R01 — CAM remains the correct motion authority

Repository architecture makes CAM responsible for contour order/motion solution and Process a consumer. The final plan preserves this: all geometry classification, complete IK, Full5D optimization and reduced candidate selection remain in CAM.

**Planning consequence:** no Process-side planner, no controller-side IK.

### FACT-R02 — Current execution geometry is still heavily point-oriented

`ToolpathExportPoint` already carries both machine solution fields and reference `tcpMcs`, but `NormalCuttingManager` currently builds run data from `pointsByContourId` and submits point motions. A typed FinalMotionPlan cutover is therefore a real migration, not a documentation-only rename.

**Planning consequence:** B2 is mandatory before claiming single execution truth.

### FACT-R03 — `tcpMcs` semantics must not be confused with physical XYZ

The existing export contract explicitly documents table-zero reference TCP semantics with carrier motion removed. It is neither posed world TCP nor controller physical-axis coordinates.

**Planning consequence:** physical DOF analysis uses solved physical axes/FK; RTCP lowering uses `tcpMcs` only under its exact contract.

### FACT-R04 — Current Process has a hard collision gate

At this HEAD, `process_cutting_safety.cpp::camExecutionBlockReason()` checks:

- machine safety package and job overlay when configured/required;
- `motionPlan.collision.complete/Pending` unconditionally before the later certificate section;
- collision blocking state;
- real-machine continuous edge certificates when safety is enabled.

This means the desired no-model `CollisionVerificationMode::Disabled` cannot be achieved by config/UI alone.

**Planning consequence:** B0 is the first mandatory implementation package; split runtime/plan readiness from collision eligibility.

### FACT-R05 — Non-collision safety must survive the split

The same Process safety area checks controller connection, readable status, fault code and axis enable state.

**Planning consequence:** Disabled means “collision verification disabled”, not “motion safety disabled”.

### FACT-R06 — Existing five-axis capabilities should be reused

The same repository baseline already includes planar/tube/five-axis solver paths and continuity logic. The final plan does not create a second tube 4-axis solver; it layers candidate analysis and optimization over the existing solve authority.

### FACT-R07 — Existing DeviceCommandQueue/device authority should be reused

The project already has a device command serialization/priority mechanism and stop/fault paths. A second SDK execution thread/queue would split authority.

**Planning consequence:** GroupSession may consolidate lifecycle state only; it is not a scheduler replacement.

### FACT-R08 — RTCP and non-RTCP are controller encoding semantics, not MotionClass

The existing GTN setup already configures different coordinate/profile semantics for RTCP and non-RTCP. This supports the final architecture decision to separate `ControllerMotionMode` from `MotionClass`.

### FACT-R09 — RTCP endpoint agreement is not a whole-segment proof

Existing endpoint transformation checks are useful but do not by themselves establish how the controller interpolates the full path while rotary angles change.

**Planning consequence:** B0 shared ContinuousMotionEvaluator + qualification boundary is mandatory before trusting aggressive 5D merge/native interpolation.

### FACT-R10 — Current collision certificate identity/interpolation is insufficient for final optimized primitives

Current node/edge certificate structures and interpolation assumptions predate the proposed typed FinalMotionPlan. They must not be reused by revision number alone after interpolation semantics change.

**Planning consequence:** B4 binds proof to exact plan/block/model/context identity and uses the same evaluator.

### FACT-R11 — Tool/process values need freezing at run preparation

Current normal cutting logic can obtain a Tool object at execution preparation and has a default/sanitized fallback path when lookup fails.

**Planning consequence:** the final design freezes an explicit valid recipe in PreparedDeviceProgram and blocks missing/stale execution recipe rather than silently changing cutting semantics.

### FACT-R12 — Public remote snapshot is not sufficient to assert build/test pass

The public repository root visible during this audit contains source/docs but does not expose the complete root build/test/dependency layout described by project documentation.

**Planning consequence:** B0 local baseline/build verification is a hard entry criterion. Planning review does not claim C++ build/CTest/HIL success.


### FACT-R13 — Current repository architecture defines real-machine collision as fail-closed production policy

Current `ARCHITECTURE.md` explicitly states that production Rapid/LeadIn/Cutting/Traverse edges require current collision certificates and that Pending/Indeterminate/BoundaryUnknown/stale/missing/incomplete proof blocks real machining. Process safety boundaries repeat that collision Pending/Collision/Indeterminate must block real machining.

**Planning decision:** v3 does not silently reinterpret this current fact. It introduces an explicit **commissioning execution policy** (`CollisionVerificationMode::Disabled`) for the user's staged validation workflow, while keeping production safety policy as `Required` unless/until product policy is deliberately changed. B0 must update the repository architecture/config/UI contract together with code so the new commissioning exception is explicit and auditable.

## 4. Planning Defects Found During Final Review and Their Resolution

| ID | Found issue | Severity if unfixed | Resolution in v3.1 Compact |
|---|---|---:|---|
| P-01 | MotionClass names mixed RTCP with DOF (`Full...RTCP...`) | Critical | MotionClass / ControllerMotionMode split |
| P-02 | v2 RTCP lowering was more explicit than PhysicalAxes lowering | Critical | full dual-mode matrix + explicit physical-layout mapping |
| P-03 | reduced candidate fallback could be misread as a lowering decision | Critical | candidate admission/selection frozen before publication; lowering cannot fallback |
| P-04 | Continuous evaluator was not sufficiently promoted as collision-independent prerequisite | Critical | B0 now shared core motion-semantics package |
| P-05 | Process hard collision gate needed exact current-code treatment | Critical | B0 first package, explicit `camExecutionBlockReason` split |
| P-06 | mutable/default tool semantics could diverge from prepared plan | High | B2 PreparedDeviceProgram + tool/process freeze |
| P-07 | controller mode changes after publication lacked explicit invalidation rule | High | mode/capability part of context/compatibility/identity |
| P-08 | B3 was only simulation/diagnostics, weaker than user's desired early machining validation | High | B3 upgraded to no-model/collision-disabled commissioning gate |
| P-09 | collision proof key not strong enough for optimized primitives | Critical for Required | B4 complete identity + exact evaluator |
| P-10 | public build gap could be mistaken for implementation pass | Medium | explicit baseline entry condition/evidence |
| P-11 | current ARCHITECTURE.md says collision fail-closed for real machining; Disabled commissioning is a deliberate policy extension | Critical if undocumented | commissioning vs production policy split; B0 must update architecture/config/UI with code |
| P-12 | several v3 draft Work Units were shorthand and did not yet contain every L4.5 fixed field | High for Terra readiness | all 114 Work Units expanded to Goal / Inputs / Repository Facts / Affected Symbols / Required Behavior / Implementation Sequence / Forbidden Solutions / Tests / Done Condition |
| P-13 | Master Plan DAG lacked per-node Entry / Implementation-Ready / Exit / Downstream criteria required by the planning standard | High | added explicit B0–B4 node readiness table |
| P-14 | Design Specification concepts existed across sections/files but Core Data Model / State Machine / Concurrency Semantics were not explicit enough in the authority document | High | added explicit architecture-level contracts and linearization/concurrency rules with linked detailed state-machine document |

All fourteen issues are closed at the **planning/design level** in v3.1 Compact. Their production code work remains assigned to the corresponding WPs.

## 5. Remaining Open Items — Deliberately Implementation/Qualification Time

These are not unresolved architecture choices and therefore do not block development start:

1. exact private C++ names/layout for block/evaluator helpers;
2. greedy vs bounded DP/local algorithm where both satisfy deterministic error invariants;
3. which ReducedDOF×ControllerMotionMode cells the actual GTN SDK/firmware can qualify;
4. exact measured process tolerance for Z-hold;
5. native cylinder/arc/persistent-Group qualification outcome;
6. local complete build/preset/SDK matrix availability;
7. HIL/physical-quality thresholds derived from the actual machine.

Each has a MUST/ESCALATE/qualification gate; Terra is not asked to invent architecture when they arise.

## 5.1 Astra → Terra Planning-Standard Compliance

The final package was re-audited against the user-supplied L4.5 planning standard, not only against repository code. The final state includes:

- explicit Design Specification component boundaries, core data model, state/linearization rules, invariants, failure semantics, concurrency semantics and rejected alternatives;
- Batch Master Plan baseline/scope/DAG plus Entry, Implementation-Ready, Exit and Downstream criteria for every WP;
- 18 Work Package Execution Specifications with all required top-level sections;
- 114 Work Units, each carrying all nine mandatory fields;
- checkpoint/test/evidence/boolean exit gates;
- explicit ESCALATE conditions for SDK/HIL/repository-contract gaps rather than asking Terra to redesign.

This closes planning-format gaps that would otherwise invalidate a `TERRA_READY = YES` conclusion.

## 6. Remaining Optimization Space After v3

Useful but not required to start the batch:

- global candidate sequence optimization across compatible adjacent blocks after local deterministic baseline proves stable;
- serialized/reusable controller capability qualification records;
- richer material-path time estimator using actual GTN feed/lookahead behavior;
- SIMD/SoA batch evaluator if profiling shows optimizer cost significant;
- automatic golden-case extraction from execution logs;
- later planner-aware safe approach optimization, kept separate from collision certification;
- persistent Group / streaming only after finite-list baseline and Stop/fault semantics are qualified.

These should not be pulled into early WPs unless profiling/qualification proves meaningful benefit.

## 7. Audit Verdict

```text
ARCHITECTURE_CONSISTENCY = PASS
PLANNING_COMPLETENESS = PASS
TERRA_READY = YES
DEVELOPMENT_RELEASE = CONDITIONAL_GO
PRODUCTION_RELEASE = NO
```

Conditions for DEVELOPMENT_RELEASE:

1. B0 begins first and verifies the actual local worktree/build baseline;
2. no optimizer real-execution cutover before B0 contracts pass;
3. no no-collision hardware commissioning before B2–B3 gates pass;
4. no production collision-safe claim before B4 as required by the target production policy.
