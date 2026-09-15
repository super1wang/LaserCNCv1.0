# 上一版规划 → v3.0 FINAL 差异

## 1. Collision 从前置变为可选后置认证

旧规划倾向：Final Motion Plan 之后必须连续碰撞证明才能进入执行。

v3：

```text
Final Motion Plan
  -> Execute                 [Disabled]
  -> Optional proof          [Optional]
  -> Required proof -> Execute [Required]
```

直接结果：B0–B3 不得依赖 machine STEP / `.lmsi` / Overlay / Coal。

## 2. Full5D 从 fallback 变成一等优化输出

旧规划容易形成：

```text
可降维 -> optimized
不可降维 -> full5d fallback
```

v3 冻结：

```text
Solved Full5D
   -> Full5D optimization
   -> optional ReducedDOF candidate
   -> choose valid lowest-cost representation
```

无论能否降维，都先得到 Optimized Full5D。

## 3. Continuous Proof 顺序后移

为了当前运动质量验证，proof consistency 的实现工作后移到 B4；但 Final Motion Plan 的 interpolation semantics/evaluator 必须在前面先设计正确，否则后续无法安全接回碰撞。

## 4. Z Hold 更保守

首版默认关闭 relaxed Z hold；先实现 numerical canonicalization 和白名单。等实际焦距/离焦容差明确后再启用 process-tolerance hold。

## 5. Controller Native Cylinder 进一步降级为资格化优化

CAM 可以识别 `RotaryLinear/CylinderEligible`，但第一阶段 GTN 仍可用已验证 Group linear RTCP lowering 执行等价优化点。原生 cylinder 仅在 SDK + 软件 + 实机资格满足后开启。
