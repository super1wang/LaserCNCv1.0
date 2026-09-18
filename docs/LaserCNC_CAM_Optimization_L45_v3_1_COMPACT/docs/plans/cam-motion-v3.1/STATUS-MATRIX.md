# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | Astra PASS | `1aa3aee..d1e82b9` | S1/S2/S3 累计 suite 与直接回归已合并验证 | V-007～V-016：Debug 12/12 + 补强单项 1/1；ASan 12/12；见 B1_R1_CLOSEOUT_REPORT.md | R1 PASS（用户确认） | B2_RELEASE=YES；controller=Unavailable/revision 0；不代表实机放行 |
| B2 | In Progress | 初版 `0b6b3ae`；首次收口 `27a8674`；F03 本次交付见实施记录 | B2.S1 PASS；F01/F02/F03 CLOSED；首次 Debug/ASan 11/11，F03 定向均 2/2；见 B2_S1_IMPLEMENTATION.md | 未执行 | R2 pending | B2.S2_RELEASE=YES（开发准入）；S2/S3 尚未实施；controller=Unavailable/revision 0，硬件 exact-section sink 默认拒绝 |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
