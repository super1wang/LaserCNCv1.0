# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | Stage Gate Passed | `1aa3aee..aa8fd34` + R1 收口工作区 | S1/S2/S3 累计 suite 与直接回归已合并验证 | V-007～V-016：Debug 12/12 + 补强单项 1/1；ASan 12/12；见 B1_R1_CLOSEOUT_REPORT.md | R1 Review Pending，待用户人工复核 | F01/F02 已修复，F03 Gate 完成；B2_RELEASE=NO（等待 R1 复核）。生产默认 Off，controller=Unavailable/revision 0；RTCP/降维准入关闭，process Z whitelist 未启用；不代表实机放行 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
