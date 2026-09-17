# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | In Progress | `1aa3aee..HEAD` + S2 工作区 | B1.S1 PASS；controller-independent S2 已实现，Debug/ASan 定向各 5/5 | B1 Stage Gate 尚未执行；全量并行性能门禁有波动、隔离复跑通过，见执行步骤 S2 实施记录 | S2 待用户人工 review | Full5D reference、严格等价 merge/refine、冻结 FK adapter 与硬预算已落地；生产 policy 仍 Off，controller 保持 Unavailable/revision 0；S3 未实施 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
