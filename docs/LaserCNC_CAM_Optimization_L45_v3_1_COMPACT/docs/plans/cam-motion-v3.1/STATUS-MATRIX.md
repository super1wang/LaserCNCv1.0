# Batch Status Matrix

| Batch | State | Commit range | Soft checkpoints | Stage Gate | Astra review | Decision / blockers |
|---|---|---|---|---|---|---|
| B0 | Astra PASS | `6b11aeb..54d1eaf` | B0.S1 Passed | Supplemental G1..G8 Passed | R0 PASS（用户确认） | F01..F05 accepted; R06/R07 patched |
| B1 | In Progress | `1aa3aee..9ca7222` + S3 实施工作区 | B1.S1 PASS；B1.S2 用户审阅通过；B1.S3 已实施，验证记录见执行步骤的 S3 实施记录 | B1 Stage Gate 尚未执行；本次仅 S3 Soft Check，不新增门禁 | S3 待用户人工 review | S3_RELEASE=YES；泛化降维、数值 Z、资格 admission、确定性成本及派生数据重建已接入。工艺 Z-hold 按步骤 19 仅保留接口/默认关闭/拒绝原因，未提供白名单实现；生产默认 Off，controller 为 Unavailable/revision 0，RTCP/降维 admission 保持关闭；不代表硬件放行 |
| B2 | Blocked by B1 | — | — | — | R2 pending | — |
| B3 | Blocked by B2 | — | — | — | R3 pending | — |
| B4 | Deferred / Blocked by B3 | — | — | — | R4 pending | — |

Allowed states: `Not Started`, `In Progress`, `Soft Check Passed`, `Stage Gate Passed`, `Astra PASS`, `PASS_WITH_PATCH`, `Blocked`, `Deferred`.
