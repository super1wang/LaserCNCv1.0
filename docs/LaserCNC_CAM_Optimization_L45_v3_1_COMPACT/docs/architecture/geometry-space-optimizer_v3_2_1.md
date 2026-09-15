# Geometry-Space Optimizer 设计 v3.2.1

## 1. 定位

Geometry-Space Optimizer 工作在 authoritative IK 之前。

它负责：

```text
source fidelity
normalization
primitive recovery
geometry-only adaptive sampling
source provenance
```

它不负责：

```text
physical DOF classification
controller capability
collision
rotary dynamics
FinalMotionPlan candidate selection
```

---

## 2. 输入与输出

输入：

- `LaserContour` OCC Wire/Edge；
- sampled `ToolpathPoint`；
- sourceEdgeIndex / curve parameter；
- normal/tangent；
- process feature/fence metadata；
- frozen geometry tolerance policy。

输出：

```text
GeometryCompilationArtifact
  ordered geometry samples
  primitive metadata
  source edge/parameter mapping
  hard barrier markers
  fit/resampling evidence
```

输出仍然是 geometry-space 数据，不包含最终 controller/collision 状态。

---

## 3. Authority 优先级

```text
OCC source curve semantics
  > validated analytic primitive
  > bounded-error sampled polyline
```

有 OCC 原始 curve 时，禁止仅因为 sampled points“看起来更简单”就覆盖 source fact。

---

## 4. Hard Barriers

以下边界首版禁止 optimizer 跨越：

```text
contour boundary
lead-in attachment
LeadIn -> Cutting
Cutting -> Rapid
semantic seam
corner/discontinuity
process/laser fence
required stop
```

这些 barrier 必须被带入后续 Pose-Space Optimizer 和 FinalMotionPlan block 构建。

---

## 5. Normalization

允许无工艺容差的 numerical-equivalent 操作：

- 去除 exact/strict numerical duplicate；
- 规范 tangent/normal；
- 修复 finite numeric noise；
- 规范 closed contour 首尾重复表达；
- 保留 curve parameter 单调性；
- 保留 winding/order；
- 不跨 topology edge / semantic seam 合并。

任何 normalization 不得改变真实加工几何意义。

---

## 6. Primitive Recovery

优先读取 OCC curve type：

- Line；
- Circle / Arc；
- Ellipse（识别即可）；
- BSpline；
- Bezier；
- Trimmed curve。

只有 source geometry 缺失时允许 point fitting。

point fitting 必须记录：

```text
fit source
residual
tolerance used
source interval
execution primitive eligibility
```

point-fit 结果不得冒充 OCC source primitive。

---

## 7. Geometry Adaptive Sampling

IK 前 sampling 只根据：

```text
chord error
tangent angle
surface-normal change
process feature boundary
source parameter interval
```

不使用：

```text
machine-axis motion
rotary speed
controller native mode
collision
```

建议实现 budget：

```text
max geometric subdivision depth
min parameter span
max sample multiplier
```

预算耗尽不得增大几何 tolerance；保持较密 source representation。

---

## 8. Geometry → IK Boundary

这是 v3.2.1 新增的冻结规则。

只要最终 geometry sample sequence 相比已有 solved sequence 发生：

```text
insert
delete
move
resample
reordered source mapping
```

旧：

```text
machine axes
solved pose
tcpMcs derived from old solve
```

全部 stale。

后续必须调用 authoritative existing complete IK solve。

Geometry Optimizer 自身禁止复制/猜测新的 machine coordinates。

---

## 9. Source Provenance

每个最终 sample 至少可以追溯：

```text
contour
source edge
source parameter
primitive provenance
normal/tangent provenance
hard barrier
```

拟合/插值产生的新点必须记录它所属 source interval，而不是变成无来源的匿名采样点。

---

## 10. Metrics / Report

每 contour/edge 建议记录：

```text
input sample count
output sample count
strict duplicate removals
primitive type
fit count / max fit residual
max chord error
max tangent error
max normal deviation
subdivision depth
processing time
```

---

## 11. Required Invariants

- GEO-INV-01：最终 geometry path 在允许几何容差内覆盖 source path。
- GEO-INV-02：seam/corner/process fence 不得被错误平滑。
- GEO-INV-03：closed contour winding/order 不变。
- GEO-INV-04：lead-in attachment parameter 不漂移。
- GEO-INV-05：不依赖 machine STEP / collision backend。
- GEO-INV-06：有 OCC source 时 point fit 不覆盖原始几何事实。
- GEO-INV-07：sample sequence 变化后旧 machine-coordinate solve 必须失效。
- GEO-INV-08：所有新增 geometry sample 必须保留 source provenance。

---

## 12. First Release Scope

首版完成：

1. source mapping；
2. normalization；
3. line/circle/arc/trimmed semantics retention；
4. bounded geometry resampling；
5. hard barrier propagation；
6. solve invalidation；
7. metrics/report。

不要求通用 NURBS compressor。
