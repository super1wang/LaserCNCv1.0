# Reduced-DOF 验收

## Principle

Reduced candidate 与 **Optimized Full5D reference** 比较，不与 raw points 比较。

## Test Classes

### C-only positive

圆管横切、固定其它物理轴时 forward path 与 reference 在容差内。

### C-only negative

构造输入 TCP 数值看似圆，但 RTCP compensation 要求 physical XYZ 变化；必须拒绝。

### Linear+C positive

管材轴向/斜线/螺旋类路径。

### Fixed-orientation 3D

姿态恒定、XYZ 变化。

### Limit / continuity

- multi-turn C；
- soft limit；
- start/end seam；
- reversal candidate。

## Acceptance

- activeAxisMask 与实际 physical hold 一致；
- forward residual 有界；
- candidate fail 时 Full5D optimized plan 完整可用；
- 不调用 collision backend。
