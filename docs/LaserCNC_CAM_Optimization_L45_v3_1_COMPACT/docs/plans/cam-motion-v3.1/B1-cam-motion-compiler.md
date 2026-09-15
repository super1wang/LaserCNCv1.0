# B1 — CAM Motion Compiler: Geometry → Optimized Full5D → Reduction → Atomic Publication

## Goal

Implement the complete CAM-side trajectory compiler in one extended development batch. This replaces old WP03–WP10. There is **one formal Stage Gate and one Astra review** at the end.

B0 contracts are frozen inputs. Do not reopen them unless an ESCALATE proves they cannot express required semantics.

## Workstream 1 — Geometry normalization and source fidelity

- remove strict numeric duplicate points while preserving topology/parameter order;
- preserve closed-contour winding, corners, seams and process fences;
- repair finite normal/tangent numeric noise only;
- retain mapping to source edge/parameter;
- use original OCC Wire/Edge/parameter interval as first authority;
- retain/recover line/circle/arc/trimmed primitive metadata;
- only fit from points when source geometry is unavailable, with explicit residual/error provenance;
- geometry resampling uses geometric chord/tangent/normal criteria and never collision.

Do not force BSpline→arc or discard CAM primitive semantics merely because GTN lacks a native emitter.

### Soft checkpoint B1.S1

Run one accumulated source/geometry suite: duplicate/noise cases, closed contour, reversed/trimmed line/arc/circle, false arc fit, source mapping. Continue automatically on pass.

## Workstream 2 — Optimized Full5D baseline

Every simultaneous five-axis solved path enters the same Full5D optimizer before any reduction.

Required behavior:

- preserve unwrapped physical rotary continuity, turn count and direction;
- reuse/audit existing IK branch continuity rather than creating a second global IK solver;
- evaluate physical solved pose semantics, not tcpMcs numeric constancy;
- default orientation tolerance zero permits only numerical-equivalent denoise;
- nonzero orientation smoothing requires explicit process/angular budget, material-arc-length parameterization and hard endpoint/fence preservation;
- output an explicit `Optimized Full5D` reference.

## Workstream 3 — Adaptive pose resampling

Use the shared evaluator. Error criteria include at least position, orientation and rotary step/dynamic demand. Preserve monotonic source parameter and hard fences. Enforce `maxRotaryStepDeg` as a hard bound. Removed/inserted spans carry concise max-error evidence. Never enlarge tolerance to meet a knot-count target.

## Workstream 4 — 5D continuous merge / typed blocks

Merge only when the same shared evaluator proves the candidate over internal/source/adaptive parameters. Store tolerance proof summary on the resulting block. If uncertain or unsupported, keep shorter optimized blocks; never return to raw sampled path. Endpoint-only equality is insufficient, especially for RTCP.

### Soft checkpoint B1.S2

Run one accumulated Full5D numeric suite: 359→361, >360 multi-turn, near-limit unwrap, orientation noise, explicit smoothing budget, adaptive refine/decimate, midpoint merge counterexample, determinism. Continue automatically on pass.

## Workstream 5 — Physical DOF candidate generation

Generate SingleAxis / 2D / 3D / Reduced4D candidates only from Optimized Full5D physical solved motion + source/process semantics.

Rules:

- geometry type is only a clue;
- block-level continuous constancy/equivalence, not per-step tiny-delta clipping;
- preserve cumulative small motion and periodic turns;
- record MotionClass, activeAxisMask, error, dynamics/process admissibility and deterministic rejection reason;
- check exact representability for the selected ControllerMotionMode/capability before publication;
- unsupported mode/capability rejects the candidate;
- Optimized Full5D remains the conservative fallback candidate if admissible;
- `fewest axes` is not the sole cost.

## Workstream 6 — Restricted laser Z-hold

Default relaxed/process Z-hold off. Numerical-only Z noise may be canonicalized only under strict equivalence. Process hold requires a real process envelope: focus/standoff/contour/calibration/runout/control bounds, valid beam/material intersection in supported geometry, continuous feasible physical-Z interval over the whole block and no mid-cut jump. Missing/ambiguous process ownership rejects the candidate.

## Workstream 7 — Deterministic selection and atomic CAM publication

Capture immutable compilation context before worker calculation. Complete all candidate generation/admission/selection before publication. Owner thread revalidates generation/source/context/controller mode/capability/cancel and publishes plan+context+hash atomically.

A controller-mode/capability change that affects semantics stales the plan and triggers recompilation/reselection. Late collision proof can attach only to exact matching identity and cannot mutate blocks.

### Soft checkpoint B1.S3

Run one accumulated reduction/publication suite: true C-only/U+C cases, false circular reduction, cumulative tiny-motion case, controller-mode unsupported candidate, Z-hold qualified/rejected, deterministic cost/tie, stale/cancel publish, identity change. Continue automatically on pass.

## Stage Gate B1

Run validation rows V-007..V-016 and directly impacted CAM/kinematics regressions once. Include deterministic before/after optimizer metrics for representative geometry. Collision backend call count remains zero.

Boolean exit:

- [ ] source fidelity/topology/fences are preserved;
- [ ] every simultaneous five-axis path has an Optimized Full5D baseline;
- [ ] resampling/merge use the shared continuous evaluator;
- [ ] rotary turns/direction survive;
- [ ] reduced candidates are physical-solve based and continuously admissible;
- [ ] restricted Z-hold has explicit process bounds or remains off;
- [ ] controller-mode capability admission happens before publication;
- [ ] candidate selection is deterministic;
- [ ] FinalMotionPlan/context/hash publication is atomic and stale-safe;
- [ ] no collision dependency or backend call;
- [ ] no unresolved ESCALATE.

Then request Astra R1. B2 starts after review closure.
