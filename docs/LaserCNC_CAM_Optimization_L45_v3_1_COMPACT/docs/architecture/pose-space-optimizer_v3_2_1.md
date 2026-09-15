# Pose-Space Optimizer / Full5D 优化设计 v3.2.1

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

误差指标至少包括：

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

在 explicit tolerance 下可做 bounded smoothing/resampling/merge。

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
