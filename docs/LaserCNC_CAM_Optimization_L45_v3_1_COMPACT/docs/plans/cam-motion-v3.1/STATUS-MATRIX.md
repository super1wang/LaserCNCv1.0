# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Stage Gate Passed | `6b11aeb..HEAD` | B0.S1 Passed | Supplemental G1..G8 Passed | Supplemental R0 pending | F01..F05 accepted; R06/R07 patched; no unresolved ESCALATE |
| B1 | Blocked | — | — | — | R1 pending | Awaiting supplemental Astra R0 PASS; immutable production context capture remains Workstream 7 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
