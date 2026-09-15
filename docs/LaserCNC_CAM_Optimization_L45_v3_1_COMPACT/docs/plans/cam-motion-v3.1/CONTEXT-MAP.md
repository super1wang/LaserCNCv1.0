# Context Map — Token-Efficient Reading

Use the smallest authoritative context needed for the active batch.

## Always read once per fresh execution context

1. `README.md` in this plan directory
2. `EXECUTION-CONTRACT.md`
3. active `B*.md`

## B0 additional

- `../../../architecture/collision-policy-decoupling.md`
- `../../../architecture/final-motion-plan-contract.md`
- `../../../architecture/continuous-motion-evaluator.md`
- `../../../research/repository-facts.md` only for disputed facts

## B1 additional

- `../../../architecture/geometry-space-optimizer.md`
- `../../../architecture/pose-space-optimizer.md`
- `../../../architecture/dof-reduction-and-z-hold.md`
- `../../../architecture/motion-parameters-v3.md`

## B2 additional

- `../../../architecture/gtn-controller-lowering.md`
- `../../../architecture/motion-plan-state-machine.md`
- `../../../architecture/controller-motion-and-collision-policy.md`

## B3 additional

- `../../../validation/full5d-validation.md`
- `../../../validation/reduced-dof-validation.md`
- `../../../validation/gtn-qualification.md`
- `../../../validation/collision-disabled-validation.md`

## B4 additional

- `../../../architecture/continuous-motion-evaluator.md`
- `../../../architecture/collision-policy-decoupling.md`
- `../../../validation/validation-matrix.md`
- `../../../validation/gtn-qualification.md`

## Only when needed

- `../../../research/final-remote-audit-2026-09-14.md`
- industry research
- previous-plan delta
- migration details

Do not spend context on historical/audit material unless the implementation encounters the exact issue it documents.
