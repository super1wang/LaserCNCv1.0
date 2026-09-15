# 设计选择与主流 CAM/CNC 模式对照

本文件只记录用于设计取向的模式，不作为具体厂商 API 规范。

## 1. 分层优化而不是“离散点直接下控制器”

成熟 CAM/CNC 普遍把：

```text
geometry intent
-> toolpath/pose generation
-> machine kinematic optimization
-> controller-aware post/lowering
-> CNC lookahead/dynamics
```

分层处理。v3 采用 Geometry-Space + Pose-Space + Controller Lowering 三段式。

## 2. 4轴/3+2/5轴是策略输出，不是固定输入

同一加工几何会根据机床/工艺选择较低自由度模式或完整五轴；因此 v3 把 MotionClass 作为 block 属性，而不是给整个项目写死“5轴就必须五轴全联动”。

## 3. Point Distribution / Orientation Smoothing

五轴轨迹质量问题经常来自点密度、法线离散和旋转轴翻转。v3 在 Full5D 路径上同样做 resampling、orientation continuity 和 reversal penalty，而不是只优化降维场景。

## 4. Controller LookAhead 是最后一级，不代替 CAM 轨迹质量

前瞻/jerk/smoothing 能改善速度规划，但无法修复错误的 IK 分支、角度回绕、过度离散或错误自由度。因此 GTN 调参位于轨迹优化之后。

## 5. Process-specific Tolerance

工业控制常把轮廓误差/动态等级与加工模式绑定。v3 同样区分 numerical-equivalent optimization 与显式 process-tolerance optimization，防止“平滑”偷偷改变加工几何。
