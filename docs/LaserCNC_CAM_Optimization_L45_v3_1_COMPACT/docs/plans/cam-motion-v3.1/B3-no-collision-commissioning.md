# B3 — No-Model / Collision-Disabled Commissioning and Motion Software Closure

## Goal

Validate the new motion stack end-to-end without requiring machine geometry/collision proof, using exact FinalMotionPlan semantics, conservative controller dry-runs and representative Full5D/reduced paths. Fold the non-collision software/performance closure into the same batch so there is only one commissioning acceptance.

## Workstream 1 — Exact-plan simulation / diagnostics

Simulation/preview/diagnostics evaluate/render the exact FinalMotionPlan, not reconstructed legacy points. Report before/after knots/blocks, MotionClass, ControllerMotionMode, active axes, rotary travel/reversals, declared errors, command estimate/actual and elapsed time.

Use controlled baselines for representative paths. Do not label display sampling as execution proof.

### Soft checkpoint B3.S1

Run exact-plan simulation/trace equivalence for representative Full5D and reduced plans, including closed/multi-turn cases. Continue automatically on pass.

## Workstream 2 — Collision Disabled end-to-end

Run with:

```text
CollisionVerificationMode = Disabled
machine geometry = absent/optional
.lmsi / package / overlay / certificate = absent allowed
```

Verify zero collision backend construction/query and visible `not collision-certified` status. Non-collision preflight and device safety remain active.

## Workstream 3 — Conservative controller commissioning

- first hardware stage laser off and conservative speed;
- qualify RTCP and PhysicalAxes only where each software cell is supported;
- capture command targets plus available APOS/encoder/controller status;
- compare expected evaluator/physical semantics with observed traces;
- qualify each reduced motion class separately; failure leaves Optimized Full5D available;
- Z-hold/native special interpolation stays off unless independently qualified.

Do not increase speed/power to mask trajectory defects.

### Soft checkpoint B3.S2

Run the planned dry-run set once. Any unexplained RTCP/PhysicalAxes/evaluator disagreement, Stop/fault regression or controller interpolation contradiction is an ESCALATE.

## Workstream 4 — Non-collision software/performance closure

Run the build/regression/numeric/fault/performance checks directly relevant to B0–B3. Produce raw benchmark samples for optimizer and execution preparation. Check core/Process dependency boundaries and absence of OCC in Process.

Do not rerun B0/B1/B2 full matrices unless a public contract changed; run the integrated regressions needed to prove the end-to-end stack.

## Stage Gate B3

Run validation rows V-024..V-027 plus the directly impacted integration/fault regressions once.

Boolean exit:

- [ ] exact FinalMotionPlan drives simulation and device command trace;
- [ ] no-model Disabled E2E has zero collision backend calls;
- [ ] Full5D optimized path shows explained, bounded motion semantics;
- [ ] each enabled reduced class has separate commissioning evidence;
- [ ] RTCP/PhysicalAxes observed behavior matches supported software model;
- [ ] no Stop/fault/ownership regression;
- [ ] deterministic/performance evidence collected;
- [ ] user-facing state clearly says not collision-certified;
- [ ] no unresolved ESCALATE.

Then request Astra R3. Passing R3 authorizes the documented Collision=Disabled commissioning profile, not production collision safety.
