# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | In Progress | `1aa3aee..HEAD` | B1.S1 PASS（复审修补 Soft Check）；controller-independent S2 可进入、未实施 | 最终 CTest 52/52 + ASan 定向 5/5；四组复审反例及真实 owner/worker/export 回归通过 | Review fixes verified | 计算前冻结、顺序身份、保守区间证明、源覆盖/屏障、硬预算和旧解拒绝已修复；ControllerMotionMode 保持 Unavailable/revision 0；见 B1_S1_REVIEW_FIX_REPORT.md |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
