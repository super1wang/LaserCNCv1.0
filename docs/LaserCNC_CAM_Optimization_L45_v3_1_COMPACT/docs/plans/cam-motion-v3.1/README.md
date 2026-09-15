# CAM Motion Optimization v3.1 — Execution-Compact Master Plan

## 1. Purpose

This package keeps the **same frozen architecture, contracts and safety invariants as v3 FINAL**, but compresses execution from 18 formal Work Packages / 114 micro work units into **5 development batches and 5 Astra stage reviews**.

The user will use a higher-capability execution model. Therefore the plan optimizes for:

- more coding per execution context;
- fewer context reloads;
- fewer full regression cycles;
- fewer completion reports;
- independent Astra review only at meaningful architecture/execution boundaries.

## 2. Repository Baseline

```text
Repository: super1wang/LaserCNCv1.0
Branch: main
Planning HEAD: 4254db59696193963392e2946132cf614c8def26
Remote tree: 781c39c091622c2e97d30ef83b0a792f37c7f9fb
Audit date: 2026-09-14
Architecture authority: repository ARCHITECTURE.md + docs/architecture/cam-trajectory-system-design-v3.1.md
```

The public snapshot still does not prove the complete local build/test/SDK surface. B0 establishes the local build baseline before implementation claims build/test success.

## 3. Five-Batch DAG

```text
B0 Foundation / Policy / Contracts / Evaluator
  collision Disabled commissioning semantics
  FinalMotionPlan + identity + orthogonal modes
  ContinuousMotionEvaluator + observability
                |
                v
          Astra Review R0
                |
                v
B1 CAM Motion Compiler
  geometry normalize + OCC primitive recovery
  Full5D optimization + resampling + 5D merge
  DOF reduction + restricted Z-hold
  candidate selection + atomic publication
                |
                v
          Astra Review R1
                |
                v
B2 Process / GTN Execution Stack
  Process cutover + PreparedDeviceProgram
  PhysicalAxes + RTCP lowering
  Group / LookAhead / dynamics lifecycle
                |
                v
          Astra Review R2
                |
                v
B3 No-Collision Commissioning / Motion Closure
  exact-plan simulation and metrics
  no-model Disabled end-to-end
  conservative GTN dry-run / selected motion classes
  non-collision software/performance closure
                |
                v
          Astra Review R3
                |
                v
B4 Collision Reintegration / Production Qualification
  exact FinalMotionPlan proof binding
  Disabled/Optional/Required policy matrix
  HIL/process qualification
  final production release matrix
                |
                v
          Astra Review R4
```

## 4. Formal Acceptance Count

Only B0..B4 exits are formal acceptance events. There is **no per-workstream acceptance**.

Inside a batch:

1. implement continuously;
2. run Quick checks after trivial edits only when needed;
3. run Local targeted suites at listed soft checkpoints;
4. continue automatically if they pass;
5. run one Stage Gate validation at the end;
6. hand the batch delta to Astra for review.

Target validation/review cadence:

```text
old v3: 114 work-unit checks + 18 WP exits + final gates
new v3.1: ~10-15 soft checkpoints + 5 stage gates + 5 Astra reviews
```

## 5. Cross-Batch Invariants

- CAM `FinalMotionPlan` is the only real execution motion truth.
- `MotionClass`, `ControllerMotionMode`, `CollisionVerificationMode` are orthogonal.
- Full5D gets primary optimization before DOF reduction.
- geometry classification is only a candidate clue; physical reduction happens after complete IK/Optimized Full5D.
- `ControllerMotionMode` affects admission/encoding, not selection of an independent CAM planner.
- Process/lowering never re-IK, replan, simplify, or select an alternate candidate.
- `ContinuousMotionEvaluator` is collision-independent and shared by optimizer/simulation/later proof.
- B0–B3 must work with `CollisionVerificationMode::Disabled` without machine geometry or collision backend.
- Disabled never weakens controller/fault/axis/limit/Stop safety.
- Required collision is fail-closed and binds the exact final motion identity.
- periodic turn count/direction is preserved.
- tool/process/IO/profile/controller mode are frozen before submission.
- all vendor SDK calls stay under the existing DeviceCommandQueue/device authority.
- unsupported/Unknown native controller semantics are never emitted as real execution.

## 6. Batch Entry / Exit Summary

| Batch | Entry | Exit summary | Astra review focus |
|---|---|---|---|
| B0 | local worktree available | collision policy split, FinalMotionPlan/identity/evaluator frozen, Disabled zero backend, Required preserved | architecture/contracts/safety |
| B1 | B0 PASS | one deterministic optimized FinalMotionPlan compiler from source geometry through Full5D/reduction/publication | math/trajectory semantics/atomic publish |
| B2 | B1 PASS | Process consumes exact plan; prepared run frozen; PhysicalAxes/RTCP lower correctly; Group lifecycle safe | execution equivalence/device safety |
| B3 | B2 PASS | no-model collision-disabled E2E + exact-plan dry-run and motion metrics pass | observed motion quality/regression |
| B4 | B3 PASS | exact collision proof + HIL/process matrix + release decision | production safety/qualification |

## 7. Soft Checkpoints Are Not Gates

Soft checkpoints exist only to avoid accumulating broken code. They do not require Astra, long reports, status approval, or full regression.

- B0.S1 contracts compile / serialization / policy matrix targeted tests
- B1.S1 geometry source fidelity
- B1.S2 Full5D continuity + resampling/merge numeric suite
- B1.S3 reduction/selection/publication targeted suite
- B2.S1 Process exact-plan cutover
- B2.S2 dual lowering traces
- B2.S3 Group/Stop/fault targeted suite
- B3.S1 simulator/trace exact-plan check
- B3.S2 no-model dry-run
- B4.S1 proof identity/evaluator tests
- B4.S2 HIL/process qualification tranche

## 8. Validation Levels

See `VALIDATION-POLICY.md`. Core rule:

```text
Quick  = compile/static/one focused test; no report
Local  = accumulated targeted suite at soft checkpoint; concise report only on failure
Stage  = batch-owned matrix once; one evidence summary
HIL    = only B3/B4 where hardware/process evidence is actually required
```

## 9. Reporting / Token Budget Rule

The execution model MUST NOT restate the plan after every task. During a batch it reports only:

- meaningful implementation decision or repository mismatch;
- test failure/blocker;
- soft-checkpoint status in a few lines.

At batch exit it produces one concise stage summary:

```text
commit range / changed areas
what was implemented
stage tests + raw evidence locations
known limitations / capability states
Astra review request
```

Detailed reasoning is reserved for ESCALATE or failed invariants.

## 10. Context Loading Rule

Do not load the entire package into every execution context. Use `CONTEXT-MAP.md` and the active batch document. Research/audit files are fallback evidence, not mandatory context unless a repository fact is disputed.

## 11. Development Release

- Coding may start with B0.
- No path-changing execution claim before R0.
- No Process/GTN cutover claim before R1.
- No no-collision hardware commissioning before B3 gate/R3.
- No production collision-safe claim before B4/R4.
