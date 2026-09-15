# LaserCNC CAM Motion Optimization — v3.1 Execution-Compact

This is the **complete compact execution package** derived from v3 FINAL. The architecture is unchanged; implementation and validation are reorganized to spend more time/tokens on development.

## Key change

```text
18 formal WPs / 114 micro work units
        ↓
5 development batches
~10–15 soft checkpoints (no external acceptance)
5 Stage Gates
5 Astra stage reviews
```

## Start here

1. `docs/plans/cam-motion-v3.1/README.md` — compact master plan
2. `docs/plans/cam-motion-v3.1/EXECUTION-CONTRACT.md` — short-form frozen architecture
3. `docs/plans/cam-motion-v3.1/CONTEXT-MAP.md` — minimal context per batch
4. `docs/plans/cam-motion-v3.1/EXECUTION-PROMPTS.md` — executor prompt
5. `docs/plans/cam-motion-v3.1/ASTRA-REVIEW-PROMPTS.md` — stage review prompt

## Batch sequence

- B0 Foundation / contracts / evaluator
- B1 CAM motion compiler
- B2 Process / GTN execution
- B3 no-collision commissioning / motion closure
- B4 collision reintegration / production qualification

## Frozen baseline

- repository: `super1wang/LaserCNCv1.0`
- branch: `main`
- planning HEAD: `4254db59696193963392e2946132cf614c8def26`
- remote tree: `781c39c091622c2e97d30ef83b0a792f37c7f9fb`

The public snapshot does not prove the complete local build/test/SDK surface; B0 confirms the actual local baseline.

## Important

The compression removes **repeated acceptance ceremonies**, not safety or correctness requirements. Any semantic ambiguity, safety regression or model/controller mismatch still triggers ESCALATE immediately.
