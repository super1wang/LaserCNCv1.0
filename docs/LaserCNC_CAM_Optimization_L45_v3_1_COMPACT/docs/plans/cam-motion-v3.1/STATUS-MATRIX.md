# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | In Progress | `1aa3aee..e8428b6` + S2 收口工作区 | B1.S1 PASS；B1.S2 PASS，Debug 累计 6 项通过 / ASan 6/6；见 B1_S2_CLOSEOUT_REPORT.md | B1 Stage Gate 尚未执行；本次仅 S2 Soft Check，不新增门禁 | S2 R0 PASS_WITH_PATCH 的 C2.1/C2.2/C2.3 已修复验证 | S3_RELEASE=YES，S3 未实施；Full 缺失生产 bound 时保留 Conservative 子集并报告拒绝；位置/姿态 bound 为 infrastructure-only、动态指标 audit-only；生产 policy 仍 Off，controller 为 Unavailable/revision 0，RTCP/admission 关闭 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
