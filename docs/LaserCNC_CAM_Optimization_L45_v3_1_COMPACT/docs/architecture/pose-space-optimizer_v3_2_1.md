# Pose-Space Optimizer / Full5D 优化设计 v3.2.1

## S2 首版能力冻结（2026-09-17 收口）

本节限定下文目标设计在 S2 的实际交付范围：Off 仅验证/必要归一化，硬约束超限失败；
Conservative 支持旋转连续性、硬旋转步长细分、evaluator 派生状态重建、严格等价 midpoint merge、软限位和指标。
Full 缺少区间 bound 时保留 Conservative 等价子集并记录确定性拒绝原因，不声明弦误差已证明。
生产 `boundPhysicalAxes` 未资格化且尚未提供；位置/姿态容差驱动细分仅属基础设施，测试 callback 不代表生产能力。
非零姿态平滑仍拒绝并保留 strict reference。速度、加速度 proxy 和 reversal count 仅审计，不是细分阈值。
生产 policy 仍 Off；控制器资格保持 Unavailable/revision 0。下文 relaxed 优化与 dynamics refinement 是后续目标，不是 S2 已交付能力。

闭合 source 的到达参数使用 `sourceEdgeIndex/param`，同点离开源不同则使用可选
`departureSourceEdgeIndex/departureSourceParameter`。反向时交换两侧，旋转起点时随点移动；
源接缝离开处拆 block，incoming span 使用离开源参数，不能把周期端点重写成起点。
该别名经导出、计划 identity 和包 metadata 持久化；不新增接缝执行节点。

## 1. 定位

Pose-Space Optimizer 工作在：

```text
final geometry sequence
  -> authoritative complete IK solve
  -> Raw solved Full5D
```

之后。

它知道：

- 每个 sample 最终 physical axes；
- world/reference TCP semantics；
- rotary continuity；
- soft limits；
- source mapping / hard barriers；
- geometry/process tolerance；
- frozen dynamics semantics；
- interpolation model/version。

它不依赖 collision geometry/backend。

---

## 2. 输入前置合同

进入 Pose Optimizer 前必须保证：

- geometry optimizer 已结束；
- geometry changes 已使旧 solved coordinates stale；
- 当前 Raw Full5D 来自 authoritative existing solver；
- B0 `entryBoundary`/process fence semantics 可表达当前 blocks；
- frozen compilation context 已捕获。

否则不得开始优化。

---

## 3. Full5D 必做优化

### 3.1 Rotary Unwrap

- 保留 physical periodic continuity；
- 保留多圈，例如 `351 -> 711`；
- 不 `%360`；
- 保留 turn direction；
- soft limit 是 hard admissibility condition。

### 3.2 IK Branch Continuity Audit

继续复用已有 solver continuity。

optimizer 只允许 block-level audit：

```text
rotary travel
reversal
soft-limit proximity
available singularity/discontinuity metric
```

不得实现第二套 global IK。

### 3.3 Orientation Numerical Denoise

`orientationToleranceDeg == 0/未配置`：

```text
numerical-equivalent cleanup only
```

不改变真实 normal target。

只有显式非零 process tolerance 才能 bounded smoothing，并且：

- material arc-length parameterization；
- endpoint preserved；
- hard barrier preserved；
- resample proof required。

---

## 4. Adaptive Pose Resampling

目标误差指标如下；S2 生产仅 rotary step 用作硬细分条件，位置/姿态 bound 为基础设施，动态 proxy 仅审计：

```text
TCP positional deviation
orientation deviation
rotary step
rotary velocity demand proxy
axis acceleration/reversal proxy
```

不得只按 XYZ chord error。

### Derived-state rule

新增 knot 的：

```text
physical axes
world TCP
process direction
reference TCP
```

必须从 authoritative evaluator/kinematics 得到。

禁止把这些字段彼此独立线性插值。

### Budget

必须有限制：

```text
max refinement depth
minimum parameter interval
maximum output knot multiplier
cancellation checkpoint
```

预算耗尽：

```text
keep original/shorter optimized interval
```

不扩大 tolerance。

---

## 5. 5D Segment Merge

候选 `[i..j]`：

1. 构造候选 interpolation；
2. 在 source knots / internal samples / adaptive midpoints 上求值；
3. 使用 conservative/refinable bound；
4. 验证 TCP deviation；
5. 验证 orientation deviation；
6. 验证 soft limits；
7. 验证 rotary continuity；
8. 验证 active-axis semantics；
9. 验证 hard barriers；
10. 通过才提交。

Unknown/unsupported：

```text
keep shorter block
```

禁止 raw sampled fallback。

---

## 6. B0 Boundary Preservation

首版 Pose Optimizer 不得跨越：

```text
contour
phase
Rapid subphase
entryBoundary
laser/process fence
required stop
semantic seam/corner
```

如果 block 带 `entryBoundary`：

- incoming canonical edge 必须继续由该 block 唯一拥有；
- 不得把 entryBoundary 复制成第二个 execution node；
- resampling/merge 不能把该 edge 吃掉；
- source span / fence ownership 必须同步。

---

## 7. optimizationMode

### Off

只允许 contract-validity / mandatory numeric normalization。

不做真正 path-changing resampling/merge/reduction。

### Conservative

允许：

```text
rotary unwrap
numerical orientation denoise
strict-equivalent resampling/merge
```

不消耗 relaxed process tolerance。

### Full

目标能力是在 explicit tolerance 与连续证明 authority 下做 bounded smoothing/resampling/merge；S2 首版仅按上方能力冻结执行。

DOF reduction 是否启用仍由 `enableDofReduction` 决定。

---

## 8. Full5D Cost / Ordering

推荐先 hard constraints，再 deterministic lexicographic tuple：

```text
requiredStops
estimatedTimeProxy
softLimitRisk
rotaryReversalCount
normalizedAxisTravel
outputKnotCount
normalizedError
stableCandidateOrdinal
```

如果实现仍采用 weighted score，则必须冻结：

```text
unit
normalization
default weight
tie epsilon
stable tie-break
```

并全部进入 optimization policy hash。

---

## 9. Fallback

任何 ReducedDOF 失败：

```text
Optimized Full5D
```

而不是 raw sampled 5D。

Full5D optimizer 无可优化内容：

```text
changed=false
```

仍然是成功输出。

---

## 10. Metrics

每 block/contour 输出：

- input/output knot count；
- merge ratio；
- max position/orientation deviation；
- rotary travel before/after；
- reversal count before/after；
- max rotary delta；
- refinement depth；
- compute-budget fallback count；
- changed flag；
- processing time。

---

## 11. Invariants

- POSE-INV-01：每条 simultaneous 5D path 不得绕过 Optimized Full5D。
- POSE-INV-02：任何 merge/resample 都有 continuous/bounded evidence。
- POSE-INV-03：periodic axis 保持 physical unwrapped continuity。
- POSE-INV-04：soft limit 不得被等价角归一化规避。
- POSE-INV-05：不调用 collision backend。
- POSE-INV-06：无 relaxed tolerance 时不得改变加工几何意义。
- POSE-INV-07：entryBoundary/process fences 是 hard barriers。
- POSE-INV-08：path-changing optimization 后 derived semantics 必须重建。
- POSE-INV-09：budget exhaustion 不能扩大 tolerance。
- POSE-INV-10：同 input/context/policy 的结果必须 deterministic。
