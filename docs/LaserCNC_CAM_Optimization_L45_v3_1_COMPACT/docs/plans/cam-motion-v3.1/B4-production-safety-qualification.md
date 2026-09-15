# B4 — Exact Collision Reintegration / HIL / Process / Production Qualification

## Goal

After motion semantics are proven in B3, reconnect collision certification to the **exact selected FinalMotionPlan**, complete remaining full regression/HIL/process qualification, and make the production release decision. This combines old WP15 + remaining WP16 + WP17 into one final production batch.

## Workstream 1 — Exact FinalMotionPlan collision proof

- collision certificate builder reuses the same shared ContinuousMotionEvaluator/model used by optimizer/simulation;
- identity binds planHash/blockHash/interpolation model and relevant machine/environment/setup/calibration/tool/capability/dynamics/policy revisions;
- Required certifies actual physical motion implied by selected ControllerMotionMode, not an independent endpoint interpolation;
- point-safe does not imply segment-safe; use conservative interval/refinement evidence;
- Unknown/stale/budget exhausted/missing proof blocks Required;
- Disabled still makes zero backend calls; Optional remains diagnostic;
- late proof attachment checks exact identity and never mutates motion.

No Required→Disabled downgrade, no path rewrite, no unbounded online exact OCCT fallback.

### Soft checkpoint B4.S1

Run proof identity/staleness, endpoint-safe/mid-collision, model mismatch, Disabled zero-call and Optional/Required policy targeted suite. Continue automatically on pass.

## Workstream 2 — HIL capability matrix

Maintain states `Unknown / Documented / SDKVerified / HILQualified / Rejected` per semantic capability. Qualify Full5D×RTCP and Full5D×PhysicalAxes separately if both ship; qualify each reduced MotionClass×mode cell separately. Native arc/cylinder/persistent Group special semantics need dedicated evidence before default enable.

Indeterminate Start/no-replay and Stop/fault behavior are explicitly included.

## Workstream 3 — Process qualification

Hardware stage progression is conservative: laser-off/low-speed first, then low-risk material/coupon only after motion acceptance. Capture command/APOS/encoder where available, faults, timing, optimizer report, effective controller parameters and process measurements.

Z-hold requires its own process coupon/tolerance window. A simulator or one successful contour cannot qualify every mode/class.

### Soft checkpoint B4.S2

Close the required HIL/process cells for intended production defaults. Rejected/Unknown cells stay default-off.

## Workstream 4 — Final release regression

Run the complete release matrix once: builds, old+new regressions, numeric/fault/lifetime checks, policy matrix, performance samples, Required collision proof and intended production HIL/process cells.

## Stage Gate B4

Run validation rows V-028..V-034 and all required production regressions once.

Boolean exit:

- [ ] Required collision certifies the exact final interpolation/evaluator identity;
- [ ] Disabled/Optional/Required behavior remains correct;
- [ ] no stale/endpoint-only/point-safe promotion;
- [ ] intended production MotionClass×ControllerMode cells are HILQualified or explicitly off;
- [ ] Z-hold/native special behaviors are qualified or off;
- [ ] Start/Stop/fault/no-replay behavior is qualified;
- [ ] production collision policy is evidence-backed;
- [ ] release build/regression/performance matrix is complete;
- [ ] no default-on Unknown capability;
- [ ] no unresolved correctness/safety ESCALATE.

Then request Astra R4. Only R4 PASS authorizes production release.
