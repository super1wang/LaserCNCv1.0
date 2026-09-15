# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..HEAD` | B0.S1 Passed | B0-CLOSE 12/12 Passed | R0 PASS | F01..F05 closed; local baseline `815ceb8`; no unresolved ESCALATE |
| B1 | Not Started | — | — | — | R1 pending | Released by B0 R0 PASS; immutable production context capture remains Workstream 7 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
