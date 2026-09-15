# 自由度降维与激光 Z 保持设计 v3.2.1

## 1. 定位

降维是 Pose-Space Optimizer 之后的 candidate generation stage。

它不是 mandatory output。

```text
不能连续证明降维
  -> Optimized Full5D
```

降维只消费：

```text
Optimized Full5D physical motion
source/process semantics
frozen controller mode/capability
frozen tolerance policy
```

不直接消费 raw sampled path。

---

## 2. Active Axis Analysis

对连续 block 每个 physical axis 分析：

```text
total span
monotonicity
max local delta
dynamic contribution
freeze residual at canonical value
```

禁止只看：

```text
RTCP input XYZ
单步 tiny delta
几何“看起来像圆”
```

### Cumulative-motion rule

类似：

```text
1001 个每步很小但累计显著的运动
```

必须保留真实累计运动，不得误判 constant axis。

---

## 3. Candidate Types

可尝试：

```text
SingleAxis
Coordinated2D
Coordinated3D
Reduced4D
Optimized Full5D
```

候选生成顺序可以低 DOF 优先，但最终选择不能只看轴数。

Optimized Full5D 始终保留为 conservative candidate。

---

## 4. Continuous Candidate Validation

候选必须在：

```text
source samples
internal knots
adaptive interpolation parameters
```

对比：

```text
candidate physical axes
 -> forward kinematics
 -> TCP / orientation / process semantics

vs

Optimized Full5D reference
```

验证：

- contour/path residual；
- orientation residual；
- process-direction residual；
- linear/rotary limits；
- periodic turn intent；
- source parameter monotonicity；
- entryBoundary/process fences；
- dynamics/feed representability。

---

## 5. C-only / Rotary-only

只有以下全部成立才接受：

- 其它 physical axes held 时，FK 路径满足误差；
- rotary angle continuous；
- turn direction preserved；
- soft limits valid；
- beam/process orientation valid；
- source contour parameter 单调覆盖整个 block；
- frozen ControllerMotionMode/capability 可以 exact/qualified 表达。

“原始 RTCP XYZ 数值几乎不变”不是判据。

---

## 6. U+C / Linear + Rotary

候选：

```text
u = physical axial coordinate
theta = unwrapped rotary coordinate
```

验证：

- forward physical path；
- contour residual；
- orientation/process residual；
- linear/rotary limits；
- seam continuity；
- cumulative motion；
- controller representability。

MotionClass 与 controller native special mode 是不同概念。

是否最终使用 native cylinder instruction 属于 lowering/qualification，不属于 DOF candidate definition。

---

## 7. Z Hold

### 7.1 Numerical Z Canonicalization

B1 首版必须实现。

只有 physical Z 的 variation 为 numerical-equivalent noise 才允许 canonicalize。

不借用 relaxed process tolerance。

### 7.2 Process-Tolerance Z Hold

默认：

```text
enableLaserZHold = false
```

首版只实现 whitelist case。

必须有：

```text
explicit focus/standoff envelope
contour tolerance
orientation tolerance
calibration/runout/control bound
supported beam/material intersection
continuous feasible physical-Z interval
no external height-follow conflict
```

### 7.3 No mid-cut jump

如果 block 中没有单一 feasible physical Z：

```text
reject candidate
```

不得在 laser-on block 内中途跳 Z。

重新定位只能发生在允许的 process fence / laser-off boundary。

### 7.4 Derived semantics

Z 被改变后，必须重新计算/revalidate：

```text
world TCP
reference TCP/tcpMcs
beam/process direction
process residual
```

不得只改 `axes[Z]`。

---

## 8. optimizationMode / Feature Flags

### Off

不产生 reduced candidate。

### Conservative

只允许 strict-equivalent reduction / numerical Z canonicalization。

### Full

`enableDofReduction=true` 时允许 bounded reduced candidate。

`enableLaserZHold=true` 且 whitelist process envelope 完整时才允许 relaxed Z-hold candidate。

---

## 9. Controller Admissibility

candidate 在进入 final selection 前必须检查：

```text
ControllerMotionMode
controller capability/qualification revision
feed/dynamics representability
```

不兼容：

```text
reject candidate
```

禁止：

```text
publish candidate
-> lowering 再偷偷 fallback Full5D
```

---

## 10. Deterministic Rejection / Selection Evidence

每个 rejected candidate 记录稳定 reason，例如：

```text
non-constant held axis
continuous error exceeded
soft limit
turn intent mismatch
process orientation mismatch
controller-mode unsupported
capability not qualified
Z process envelope missing
compute budget exhausted
```

候选顺序和 tie-break 不得依赖 worker/container/pointer order。

---

## 11. First Release Scope

B1 首版要求：

1. physical active-axis analysis；
2. C-only / linear+rotary / generic reduced candidates；
3. continuous FK/evaluator validation；
4. cumulative-small-motion negative case；
5. numerical Z canonicalization；
6. process Z-hold infrastructure + whitelist case；
7. controller admissibility；
8. deterministic rejection reasons；
9. Optimized Full5D fallback。

不要求通用 constrained process IK。

---

## 12. Invariants

- RED-INV-01：reference 永远是 Optimized Full5D。
- RED-INV-02：inactive axis 是 continuous physical hold commitment。
- RED-INV-03：failure 只能回退，不能扩大 tolerance。
- RED-INV-04：process tolerance 为 0/缺失时只允许 numerical-equivalent reduction。
- RED-INV-05：不依赖 collision backend。
- RED-INV-06：controller-mode incompatibility 在 publication 前淘汰。
- RED-INV-07：cumulative tiny motion 不得被 local threshold freeze。
- RED-INV-08：entryBoundary/process fences 不得被降维破坏。
- RED-INV-09：Z 修改后必须 rebuild derived semantics。
- RED-INV-10：relaxed process Z-hold 默认关闭且 whitelist-only。
