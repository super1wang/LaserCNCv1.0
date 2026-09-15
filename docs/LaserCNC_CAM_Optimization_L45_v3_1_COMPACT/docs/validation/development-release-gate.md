# Development Release Gate — v3.1 Compact

```text
PLANNING_READY              = YES
START_CODING                = YES, subject to B0 local baseline
NO_COLLISION_COMMISSIONING  = after B3 PASS
PRODUCTION_COLLISION_READY  = after B4 PASS
```

## Gate policy

There are only **five formal acceptance events**. Everything inside a batch is implementation work plus quick/local checks.

| Gate | Batch | What it authorizes | Astra review required |
|---|---|---|---|
| G0 | B0 Foundation | path-changing optimizer development against frozen contracts | YES |
| G1 | B1 CAM Motion Compiler | Process/GTN cutover development | YES |
| G2 | B2 Execution Stack | no-collision machine commissioning preparation | YES |
| G3 | B3 Commissioning | documented Collision=Disabled commissioning use | YES |
| G4 | B4 Production | production policy/capability release decision | YES |

## Non-gates

The following are not formal acceptance points and must not trigger full regression/review by default:

- enum/DTO/logging/config plumbing;
- helper functions and pure numeric utilities;
- individual primitive recovery cases;
- individual optimizer heuristics;
- isolated unit-test fixture changes;
- documentation synchronization.

They receive Quick/Local checks and remain inside the active batch.

## Mandatory stop conditions

Stop the affected branch immediately if any of the following occurs:

- repository facts materially contradict the batch contract;
- a frozen public motion/execution contract cannot express required semantics;
- collision Disabled would require weakening non-collision device safety;
- evaluator/controller semantics cannot be bounded where a transformation depends on them;
- Process/GTN would need a second trajectory truth or second SDK authority;
- persistent test failure shows a correctness/safety invariant break.
