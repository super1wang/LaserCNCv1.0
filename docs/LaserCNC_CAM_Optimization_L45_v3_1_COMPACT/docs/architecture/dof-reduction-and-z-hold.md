# 自由度降维与激光 Z 保持设计

## 1. 定位

降维是 Pose-Space Optimizer 的候选生成器之一。它不是必经输出；不能安全降维时保留 Optimized Full5D。

## 2. Active Axis Analysis

对一个连续 block 计算每个物理轴的：

- total span；
- monotonicity；
- max local delta；
- dynamic contribution；
- freeze residual if held at canonical value。

必须分析**物理轴**，不能只看 RTCP input TCP 字段。

## 3. 候选类型

按低自由度优先尝试但不强制：

```text
SingleAxis
Coordinated2D
Coordinated3D
Reduced4D
Full5D
```

## 4. C-only / Rotary-only

只有以下全部成立才可接受：

- 其它物理轴保持不动时，forward kinematics 生成的 TCP/beam 路径满足工艺误差；
- C 角连续且方向合理；
- 轴限位有效；
- 激光入射方向满足要求；
- source contour 参数单调覆盖完整切割路径。

“原始 RTCP 输入 XYZ 看起来是圆”或“XYZ 数值变化很小”都不是充分条件。

## 5. U+C / 线性+旋转

用于管材典型展开轨迹。候选构造：

```text
u = axial coordinate
θ = continuous rotary angle
```

验证：

- forward physical path；
- contour residual；
- orientation residual；
- linear/rotary limits；
- seam continuity。

MotionClass 本身不由控制器 native mode 决定；但当前 `ControllerMotionMode` / capability 是否能等价表达该 candidate 必须在 FinalMotionPlan 发布前完成 admissibility 检查。native cylinder 是否真正编码为特殊 GTN 指令仍由 lowering 执行，但 lowering 不再选择另一条 motion candidate。

## 6. Z Hold

### 6.1 Numerical Z Canonicalization

如果 block 内物理 Z 仅有 numerical noise，允许固定为 canonical Z。

### 6.2 Process-Tolerance Z Hold

仅在配置显式允许时：

```text
maxFocusOrStandoffError <= zHoldToleranceMm
maxContourPositionError <= contourToleranceMm
maxOrientationError <= orientationToleranceDeg
```

### 6.3 首版白名单

为了避免 Terra 自行设计任意约束 IK，首版 Z-hold 只允许：

- rotary orientation 固定或已证明 Z freeze 不破坏 TCP；
- 激光头/beam 与 Z carrier 关系明确；
- 无独立高度跟随回路与该固定策略冲突；
- 有明确 process Z tolerance。

其它情况：`ESCALATE` 或保留 Full5D。

## 7. Candidate Validation / Controller Admissibility

降维验证至少要在 source sample 参数和自适应插值参数上比较：

```text
candidate physical axes -> forward kinematics -> TCP/orientation
vs
optimized full5d reference
```

随后检查 selected `ControllerMotionMode` + capability 是否能精确表达 candidate。只有通过后才参与最终选择/发布。

碰撞关闭时到此结束；碰撞 Required 时 Final Motion Plan 发布后再由 Collision Verifier 认证。

## 8. Invariants

- RED-INV-01：降维以 Optimized Full5D 为参考，不直接以 raw sampled path 为参考。
- RED-INV-02：非活动轴是物理保持承诺。
- RED-INV-03：失败只能回退，不得扩大容差。
- RED-INV-04：process tolerance 为 0/未配置时只允许数值等价降维。
- RED-INV-05：降维不依赖 collision backend。
- RED-INV-06：controller-mode incompatibility 在发布前淘汰 candidate；Process/lowering 不做 motion fallback。
