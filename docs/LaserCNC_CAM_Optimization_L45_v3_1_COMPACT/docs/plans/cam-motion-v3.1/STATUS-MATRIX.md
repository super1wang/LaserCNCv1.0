# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | Astra PASS | `1aa3aee..d1e82b9` | S1/S2/S3 累计 suite 与直接回归已合并验证 | V-007～V-016：Debug 12/12 + 补强单项 1/1；ASan 12/12；见 B1_R1_CLOSEOUT_REPORT.md | R1 PASS（用户确认） | B2_RELEASE=YES；controller=Unavailable/revision 0；不代表实机放行 |
| B2 | In Progress | S1 `0b6b3ae..398b4e0`；S2 `ba4f8d7` | S1 用户审阅通过；S2 软件编码检查点通过，Debug/ASan 定向均 5/5；见 B2_S2_IMPLEMENTATION.md | 未执行 | R2 pending；S2 待人工 review | PhysicalAxes/RTCP 主机编码已实现；S3 Group/IO 提交与启动待实施；production controller=Unavailable/revision 0，exact Start 保持拒绝 |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
