# Readiness Review — v3.1 Execution-Compact

## Decision

The architecture remains development-ready. v3.1 reduces orchestration overhead without removing any correctness/safety invariant from v3 FINAL.

- Start coding: **YES**, beginning with B0 local baseline.
- Path-changing CAM optimization: after B0 stage review.
- Process/GTN cutover: after B1 stage review.
- Collision-disabled hardware commissioning: after B2 implementation and B3 gate.
- Production collision-safe claim: only after B4.

## Why the plan is compressed

The old 18-WP/114-work-unit structure repeated validation matrices at too many subtask boundaries. v3.1 keeps the same design and tests but moves acceptance to five stage gates. This is appropriate because the user will use a higher-capability execution model and Astra for independent stage review.

## Risk control retained

- B0 remains a hard contract gate because mistakes there contaminate all later motion semantics.
- B1 is large, but has internal soft checkpoints for geometry, Full5D, and selection/publication; these are not external review gates.
- B2 remains an execution-boundary gate because Process/GTN mistakes can alter real motion.
- B3 is the only no-collision commissioning gate.
- B4 is the only production safety/HIL gate.
