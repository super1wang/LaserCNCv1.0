# Validation Matrix — v3.1 Compact

> This matrix preserves v3 FINAL correctness coverage while changing **when** validation runs. Individual workstreams do not own a full acceptance cycle. The complete row set for a batch is executed once at its Stage Gate.

| ID | Validation | Positive evidence | Negative / boundary evidence | Stage |
|---|---|---|---|---|
| V-001 | Disabled no-model planning | valid kinematics, no machine geometry | Required with missing proof/resources blocks | B0 |
| V-002 | non-collision safety preserved | Disabled + healthy device passes | controller fault/axis disabled still blocks | B0 |
| V-003 | orthogonal policy model | Full5D+RTCP and Full5D+PhysicalAxes representable | MotionClass cannot encode RTCP | B0 |
| V-004 | single execution truth + identity | FinalMotionPlan blocks/context/hash stable | legacy points cannot become second writable truth | B0 |
| V-005 | shared evaluator | same block semantics used by optimizer/sim | endpoint-only equivalence rejected | B0 |
| V-006 | collision-independent evaluator | evaluator works with no collision backend | any backend call in Disabled fails | B0 |
| V-007 | geometry normalization | duplicates/noise removed | hard corners/fences/winding preserved | B1 |
| V-008 | OCC-first primitive retention | source line/arc/circle retained | false arc fit rejected | B1 |
| V-009 | rotary continuity | 359→361 and multi-turn preserved | modulo/shortest path erases turn -> fail | B1 |
| V-010 | Full5D optimization | bounded denoise/smoothing/resampling | no tolerance => no real orientation change | B1 |
| V-011 | adaptive resampling | smooth 5D path reduces knots | high orientation/dynamics regions retain/refine | B1 |
| V-012 | 5D merge | evaluable near-linear chain merges | midpoint violation or unknown model rejects | B1 |
| V-013 | DOF classification | true C-only/U+C/reduced candidate | geometry-only or accumulated tiny motion false reduction rejected | B1 |
| V-014 | Z-hold | numerical/qualified process hold only | missing process envelope rejects | B1 |
| V-015 | deterministic selection | same input -> same class/plan hash | unsupported mode/capability candidate rejected pre-publish | B1 |
| V-016 | atomic publication | plan/context/hash visible together | stale/cancel worker cannot publish | B1 |
| V-017 | Process cutover | exact FinalMotionPlan consumed | no legacy/raw point fallback | B2 |
| V-018 | PreparedDeviceProgram | tool/IO/profile/mode frozen | missing/stale tool blocks; mutable UI cannot alter prepared run | B2 |
| V-019 | RTCP lowering | exact tcpMcs+rotary trace | invalid reference/mode identity blocks | B2 |
| V-020 | PhysicalAxes lowering | explicit physical-layout mapping | semantic-axis/index mismatch rejects | B2 |
| V-021 | no lowering fallback | supported block encodes | unsupported primitive/mode blocks, no alternate path | B2 |
| V-022 | Group/LookAhead | existing queue/Stop priority and recovery preserved | second queue/ownership leak fails | B2 |
| V-023 | Start indeterminate | safe stop/latch/no replay | automatic replay fails | B2/B4 |
| V-024 | no-model E2E | plan→Process→GTN with zero collision calls | hidden package/cert demand fails | B3 |
| V-025 | Full5D motion quality | before/after metrics + dry-run trace | unexplained physical mismatch escalates | B3 |
| V-026 | reduced class commissioning | each enabled class has evidence | one class success cannot qualify all | B3 |
| V-027 | deterministic/performance closure | repeat input stable; raw benchmark samples | unbounded retry/recursion/exact fallback fails | B3 |
| V-028 | exact collision semantics | certifier uses same evaluator/model | independent node interpolation fails | B4 |
| V-029 | proof identity | exact plan/block/context attaches | stale plan/mode/capability proof rejected | B4 |
| V-030 | continuous proof | interval/refinement safe evidence | endpoint-safe/mid-collision rejects | B4 |
| V-031 | policy matrix | Disabled zero calls; Optional diagnostic; Required gate | Required Missing/Pending/Stale/Unknown blocks | B4 |
| V-032 | capability qualification | HILQualified cells may enable | Unknown native feature default-on fails | B4 |
| V-033 | process qualification | low-risk staged evidence | simulator evidence cannot become HILQualified | B4 |
| V-034 | production release | enabled matrix evidence-backed | unresolved safety/correctness blocker -> no release | B4 |

## Execution rule

B1 v3.2.1 的 V-007～V-016 收口证据与 case 映射见
[R1 收口记录](../plans/cam-motion-v3.1/B1_R1_CLOSEOUT_REPORT.md)。
V-010～V-012 仅接受已批准 strict subset：非零平滑和生产通用区间 bound 延后，dynamics 为 audit-only；
V-014 的工艺 Z whitelist 未启用；测试资格不得解释为 ACS/GTN 生产资格。

- During implementation, run only the smallest tests necessary to keep the branch healthy.
- At a soft checkpoint, run the accumulated targeted suite for that workstream cluster.
- At `B0..B4 Stage Gate`, run all matrix rows owned by that batch exactly once, plus directly impacted regressions.
- Do not rerun earlier complete batch matrices unless a frozen contract used by that earlier batch changed.
