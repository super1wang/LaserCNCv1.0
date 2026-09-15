# Geometry-Space Optimizer 设计

## 1. 输入与输出

输入：`LaserContour` 的 OCC wire/edge、现有 sampled `ToolpathPoint`、sourceEdgeIndex、curve parameter、法线/切线。

输出：仍是几何空间的 ordered path samples + primitive metadata，不包含控制器或碰撞状态。

## 2. 优先级

```text
OCC source curve semantics
  > validated analytic primitive
  > bounded-error sampled polyline
```

不得在有 OCC 原始曲线时仅用离散点拟合结果覆盖原始几何事实。

## 3. Normalization

允许无工艺容差的“数值等价优化”：

- 去除完全重复或低于严格 numerical epsilon 的点；
- 规范切线/法线长度；
- 修正闭合轮廓首尾重复表达；
- 保留 source edge/curve parameter 单调映射；
- 不跨拓扑 edge 合并语义不连续处。

## 4. Primitive Recovery

优先直接读取 OCC curve type：

- line；
- circle/arc；
- ellipse（仅识别，不要求首阶段 native lowering）；
- BSpline/Bezier 保留 sampled/curve-backed 表达。

只有缺失源几何时才允许 point fitting，并必须记录：

- fit residual；
- source = sampled-fit；
- tolerance used；
- 是否可作为 execution primitive。

## 5. Adaptive Sampling

IK 前采样只负责几何：

- chord error；
- tangent angle；
- surface normal change；
- process feature boundary。

IK 后的 rotary/axis dynamic 采样由 Pose-Space Optimizer 负责。

## 6. Required Invariants

- GEO-INV-01：优化后的 path 在允许几何容差内覆盖原始路径。
- GEO-INV-02：拓扑 seam/corner 不得因删点被错误平滑。
- GEO-INV-03：closed contour 的 winding/order 不变。
- GEO-INV-04：lead-in attachment param 不因优化漂移。
- GEO-INV-05：不依赖 machine STEP 或 collision backend。

## 7. First Release Scope

首版只要求：

1. normalization；
2. line/circle semantic retention；
3. bounded resample；
4. metrics/report。

不要求一次性实现通用 NURBS compressor。
