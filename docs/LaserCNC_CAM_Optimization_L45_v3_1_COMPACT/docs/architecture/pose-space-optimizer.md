# Pose-Space Optimizer / Full 5D 优化设计

## 1. 核心原则

Pose-Space Optimizer 在完整五轴求解之后工作，因此它知道：

- 每个 sample 的最终物理轴；
- TCP/world/reference 信息；
- rotary continuity；
- 轴限位；
- 几何/工艺容差。

它不依赖 collision geometry。

## 2. Full5D 必做优化

### 2.1 Rotary Unwrap

- 对周期旋转轴寻找与前一物理姿态连续的等价角；
- 保留多圈累计，例如 `351 -> 711`；
- 不做 `%360` 回绕；
- 超出软限位时不能通过 unwrap 隐藏。

### 2.2 IK Branch Continuity

如果存在多个等价 IK 解，候选成本至少考虑：

```text
rotary travel
+ reversal penalty
+ proximity to soft limit
+ singularity penalty
+ discontinuity penalty
```

首版可复用当前 solver continuity，并在 optimizer 侧做 block-level audit；不得重新实现第二套完整 IK。

### 2.3 Orientation Numerical Denoise

无显式 relaxed tolerance 时：

- 只消除由浮点/离散化产生的 numerical noise；
- 不主动改变真实 surface normal 目标。

启用 `orientationToleranceDeg > 0` 后，才允许 bounded smoothing，并必须回采样证明。

### 2.4 Adaptive Pose Resampling

误差指标至少包括：

```text
TCP positional deviation
orientation deviation
rotary step
rotary velocity demand proxy
axis acceleration/reversal proxy
```

不得只按 XYZ chord error。

### 2.5 5D Segment Merge

尝试把连续多个 knots 合并成更长 block。

对任意候选 `[i..j]`：

1. 构建候选 interpolation；
2. 在 source samples + 自适应中间参数上求值；
3. 验证 TCP deviation；
4. 验证 orientation deviation；
5. 验证轴软限位与连续性；
6. 验证 active-axis 语义；
7. 通过才提交合并。

碰撞不属于这一步的必要条件。

## 3. Full5D Cost Function

推荐 block/candidate 排序：

```text
score = wPoint * outputKnotCount
      + wRotaryTravel * totalRotaryTravel
      + wReverse * rotaryReversalCount
      + wAccel * dynamicVariationProxy
      + wLimit * softLimitProximity
      + wError * normalizedPathError
```

权重需有默认值且进入 optimization policy hash。

## 4. Fallback

任何 ReducedDOF 失败：

```text
Optimized Full5D
```

而不是 raw sampled 5D。

如果 Full5D optimizer 自身找不到更优候选：

- 保留 normalized solved path；
- optimization report 标记 `changed=false`；
- 不把“没改变”视为失败。

## 5. Metrics

每 contour/block 输出：

- input knot count；
- output knot count；
- full5d merge ratio；
- max position/orientation deviation；
- rotary travel before/after；
- reversal count before/after；
- max rotary delta per segment；
- fallback reason；
- processing time。

## 6. Invariants

- POSE-INV-01：Full5D 不得绕过 optimizer。
- POSE-INV-02：任何合并都必须有 continuous/bounded error evidence。
- POSE-INV-03：periodic axis 保持物理连续角。
- POSE-INV-04：soft limit 不得通过等价角归一化规避。
- POSE-INV-05：optimizer 不调用 collision backend。
- POSE-INV-06：没有显式 relaxed tolerance 时不得改变加工几何意义。
