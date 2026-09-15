# Full5D 优化验收

## Data Sets

至少覆盖：

- 平滑自由曲面 5D；
- 法线离散噪声；
- A/C 双轴同时变化；
- C 跨 360 与多圈；
- 近 rotary soft limit；
- 小 XYZ、大姿态变化；
- 大 XYZ、小姿态变化。

## Required Metrics

```text
input/output knots
max TCP deviation
max orientation deviation
rotary travel A/C before/after
reversal count before/after
max rotary delta per segment
GTN command count
optimizer wall time
```

## Acceptance

- position/orientation 不超过 configured tolerance；
- 无 relaxed tolerance 时几何意义不变；
- multi-turn 不被 modulo；
- reversal 不应无原因增加；
- no-change case 仍标记为 valid optimized result；
- collision backend call count = 0（B4 前 profile）。
