# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Stage Gate Passed | `6b11aeb` | B0.S1 Passed | V-001..V-006 Passed | R0 pending | Local baseline `815ceb8`; target v1.1 remote currently empty; no unresolved ESCALATE |
| B1 | Blocked | — | — | — | R1 pending | Awaiting Astra R0 PASS or PASS_WITH_PATCH closure |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
