# Validation Policy — Minimal Acceptance / High-Quality Development

## 1. Goal

Spend engineering time and model tokens on implementation, math and debugging rather than repeated full validation. Correctness coverage is retained, but acceptance is **batched**.

## 2. Levels

### Q0 — Quick

Use for DTO/enums/logging/config plumbing, simple mappings, refactors with no semantic change, fixtures, documentation.

Typical action:

```text
build affected target OR static compile check
+ 0..2 directly relevant tests
```

No formal evidence document. If clean, continue immediately.

### L1 — Local

Use for pure algorithms and a cluster of tightly related implementation slices.

Typical action:

```text
run targeted unit/numeric suite for the accumulated slice
run one negative/boundary case where risk exists
```

No Astra review. No full project regression. If clean, continue.

### S2 — Stage Gate

Exactly once at the end of each batch.

Run:

- all validation-matrix rows assigned to the batch;
- directly impacted existing regressions;
- the relevant build/integration profile;
- raw benchmark/fault traces only where the batch owns them.

Produce one evidence summary. Then request Astra review.

### Q3 — Qualification / HIL

Only when real hardware/process behavior is the subject of the gate. Primarily B3/B4.

## 3. Escalation overrides compression

Compression never means ignoring a suspicious result. Any of these immediately stop the affected branch:

- semantic ambiguity in a frozen contract;
- unexplained evaluator/controller mismatch;
- trajectory error beyond budget;
- device safety regression;
- stale async publish/lifetime race;
- persistent deterministic test failure.

## 4. Rerun policy

Do not rerun a completed earlier Stage Gate unless:

- its public contract changed;
- its execution semantics/hash inputs changed;
- a later failure directly implicates it.

Fixing a local implementation detail in B2 does not automatically rerun all B0/B1 matrices.

## 5. Test authoring policy

Tests may be written during implementation, but they are **accumulated**. Avoid creating a full positive/negative/boundary matrix for every helper. Test at the lowest level that catches the failure mode:

- helper math: focused numeric cases;
- block transform: block-level equivalence;
- public contract: contract/consumer test;
- execution boundary: integration/fault tests;
- real controller assumption: HIL qualification.

## 6. Evidence policy

Only Stage/HIL gates require durable evidence. Quick/Local results can remain in console/CI history unless a failure or surprising metric needs preservation.
