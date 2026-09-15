# B2 — Process / GTN Execution Stack

## Goal

Cut real execution over to the exact published FinalMotionPlan, freeze the run context, implement PhysicalAxes and RTCP lowering, and preserve/qualify GTN Group/LookAhead lifecycle. This replaces old WP11–WP13 and has one formal acceptance.

## Workstream 1 — Process exact-plan cutover

- consume FinalMotionPlan blocks in published order;
- preserve CAM-authoritative contour order, rapid/lead-in/cutting/process fences;
- build one immutable `PreparedDeviceProgram` freezing planHash/contextHash/controller mode/capability/tool/process/IO/profile/runEpoch;
- missing/stale real-run recipe blocks preparation; no sanitized default substitution;
- use committed collision policy: Disabled/Optional do not require proof;
- preserve Stop/fault/pause/ownership;
- after cutover, legacy point loop is not a silent real-machine fallback.

Process does no OCC planning, point simplification, arc fitting, IK, DOF reduction/restoration or post-publication candidate switch.

### Soft checkpoint B2.S1

Run exact-plan consumer, recipe freeze, mutable-UI-after-prepare, no-legacy-fallback and collision-policy targeted integration tests. Continue automatically on pass.

## Workstream 2 — GTN PhysicalAxes / RTCP lowering

- RTCP encodes exact selected block table-zero reference TCP (`tcpMcs`) + rotary data under the qualified Group semantics;
- PhysicalAxes encodes exact CAM solved physical-layout targets through explicit axis mapping; no IK in Process;
- preserve unwrapped rotary turn intent;
- command trace/userTag maps back to block/knot/source identity;
- unsupported MotionClass×ControllerMotionMode cell blocks as capability mismatch; no alternate trajectory is selected;
- feed/dynamics metric is explicit and qualified; never blindly reuse mixed mm/s↔deg/s;
- special/native circle/cylinder modes default off until separately qualified;
- keep existing RTCP target validation and whole-segment qualification where optimization relies on controller interpolation.

### Soft checkpoint B2.S2

Run RTCP and PhysicalAxes trace tests, axis mapping negative case, turn-preservation case, unsupported-cell/no-fallback case, feed-unit cases. Continue automatically on pass.

## Workstream 3 — Group / LookAhead / dynamics lifecycle

- log requested/effective/source/unit/revision for material run parameters;
- preserve profile→encoder synchronization and current fault/release recovery;
- consolidate Group state only if it removes duplicate authority;
- reuse only when prior list drained/stopped and mode/profile/capability/IO state compatible;
- direct-axis phases that require point mode own/release Group correctly;
- production-safe first implementation seals finite list before Start;
- host filling may be sliced through the existing DeviceCommandQueue so Stop can run between slices;
- indeterminate Start => safe outputs, stop/release/latch as possible, **no automatic replay**;
- lookahead/smoothing tuning happens only against known-good FinalMotionPlan, not to hide bad geometry.

Never create a second SDK worker/queue.

### Soft checkpoint B2.S3

Run Group ownership, queue priority, Stop/fault/recovery, command-list start, indeterminate-start and parameter-effective logging targeted suite. Continue automatically on pass.

## Stage Gate B2

Run validation rows V-017..V-023 and directly impacted Process/GTN regressions once. Produce command traces for at least one Full5D RTCP and one PhysicalAxes path where supported.

Boolean exit:

- [ ] Process consumes exact FinalMotionPlan as the only real motion source;
- [ ] PreparedDeviceProgram freezes all run-semantic inputs;
- [ ] no legacy/raw fallback or Process-side trajectory transformation;
- [ ] both lowering modes are explicit and only enabled in supported cells;
- [ ] rotary intent/feed units/identity are preserved;
- [ ] existing DeviceCommandQueue remains the single device authority;
- [ ] Group/Stop/fault/recovery/no-replay semantics are intact;
- [ ] no unresolved ESCALATE.

Then request Astra R2. B3 starts after review closure.
