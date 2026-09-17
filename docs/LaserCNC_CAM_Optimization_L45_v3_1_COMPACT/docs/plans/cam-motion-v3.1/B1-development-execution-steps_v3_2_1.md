# B1 开发执行步骤 v3.2.1（紧凑详细版）

> 目的：提供比主规划更具体、但不拆成大量微型 Work Unit 的连续开发步骤。  
> 验收节奏保持原 B1：**B1.S1、B1.S2、B1.S3 三次 Soft Checkpoint；最终一次 Stage Gate；最终一次 Astra R1。**  
> Soft Checkpoint 通过后执行模型自动继续，不需要人工逐项确认。

---

## 0. 执行前准备

先读取：

```text
1. B1 主规划
2. 当前阶段对应功能设计
3. B0 FinalMotionPlan / ContinuousMotionEvaluator / entryBoundary 实现
4. 当前仓库 authoritative IK solve / CAM publication 路径
```

确认当前开发 HEAD 是 B0/R0 PASS 之后的分支。

本阶段禁止：

```text
重新设计 B0
引入第二套 IK
引入 collision dependency
提前实现 B2 lowering
为未知工艺自行创造 tolerance
```

---

# 第一阶段：冻结编译输入 + Geometry Optimizer

## 开发步骤 1：梳理真实 authority

将 B0 当前 MotionCompilationContext 中每个字段映射到仓库真实 authority：

```text
source/toolpath/order
machine kinematics/layout/limits
setup/calibration
tool/process
optimizer policy
controller mode
controller capability revision
dynamics semantics
interpolation model
collision mode
```

目标不是马上改所有模块，而是先消除“某字段到底谁负责”的歧义。

凡是影响 path semantics、candidate admission、planHash 的值，只允许一个 authority。

---

## 开发步骤 2：建立 immutable compilation input

在 CAM owner thread 捕获完整 detached input。

后台 geometry / IK / pose optimizer 都只读取该 input。

必须同时保留：

```text
revision/hash
requested/effective parameter
parameter source/unit
```

这样后面 Stage Gate 可直接 A/B 复现。

---

## 开发步骤 3：整理 source geometry provenance

为 Geometry Optimizer 建立稳定 source mapping：

```text
contour
OCC edge/wire
parameter interval
sample source parameter
normal/tangent
hard barrier
```

原 OCC source 存在时，不用 sampled fit 覆盖它。

---

## 开发步骤 4：实现 strict normalization

先实现低风险动作：

```text
strict duplicate removal
finite normal/tangent normalization
closed-contour duplicate representation cleanup
```

然后补：

```text
seam/corner
winding/order
lead-in attachment
source parameter monotonicity
```

这些都不能改变真实加工几何意义。

---

## 开发步骤 5：实现 primitive retention/recovery

优先处理：

```text
line
circle/arc
trimmed primitives
```

ellipse/BSpline/Bezier 先保留语义，不要求 native controller support。

point fitting 只用于 source 缺失，并记录 residual/provenance。

---

## 开发步骤 6：实现 geometry-only adaptive sampling

只根据：

```text
chord
tangent
normal
process feature
```

细分。

加上 subdivision/sample budget，但 budget 不允许改变 tolerance。

只要 sample sequence 发生变化，就显式 invalidate 原 machine coordinates / solved poses。

---

## B1.S1 — Geometry Soft Checkpoint

一次运行 geometry accumulated tests：

```text
duplicate/noise
closed/winding
seam/corner
reversed/trimmed primitive
false fit
source mapping
lead-in attachment
bounded resample
old solved-coordinate invalidation
```

通过后自动继续。

---

# 第二阶段：Authoritative IK + Optimized Full5D

## 开发步骤 7：把最终 geometry sequence 接到现有 authoritative solver

不要在 optimizer 内另建 solver。

重点检查：

```text
final geometry -> current solver input mapping
ordered contour continuity seed
workpiece/setup/tool context
physical layout mapping
```

geometry sequence 改变后必须完整 re-solve。

---

## 开发步骤 8：建立 Raw solved Full5D block

把 solver 输出转换成 B0 FinalMotionPlan block semantics：

```text
physicalKnots
phase
rapidPhase
entryBoundary
sourceSpans
fences
feed semantics
```

先保证 canonical edge ownership 与 B0 test 一致，再做 optimizer。

---

## 开发步骤 9：实现 rotary unwrap

先做：

```text
359 -> 361
351 -> 711
```

再处理：

```text
near soft limit
reversal
turn-direction preservation
```

禁止 modulo 360。

---

## 开发步骤 10：IK continuity audit

复用现有 solver continuity。

只增加 optimizer-side metrics / rejection information，不重求全局 IK。

---

## 开发步骤 11：orientation numerical denoise

默认 tolerance=0：

```text
only numerical cleanup
```

如果仓库存在明确非零 process orientation tolerance，再启用 bounded smoothing。

否则保持关闭。

---

# 第三阶段：Adaptive Pose Resampling + 5D Merge

## 开发步骤 12：绑定 shared evaluator

将当前 candidate block/context 绑定到 B0 `ContinuousMotionEvaluator`。

如果要优化 B0 中每次求值重复 block hash 的 hot path，可以在这里结合 immutable ownership 优化；但不能通过移除身份保护换性能。

---

## 开发步骤 13：实现 Full5D adaptive resampling

依次加入指标：

```text
position
orientation
rotary step
rotary velocity proxy
accel/reversal proxy
```

先保证 correct，再优化性能。

所有新 knot 的 TCP/reference/process direction 必须重新通过 evaluator/kinematics 派生。

---

## 开发步骤 14：实现 5D merge

从短 span 开始。

每个 merge candidate：

```text
build candidate interpolation
evaluate source/internal/adaptive points
check bounds
check soft limits
check rotary continuity
check hard barriers
```

Unknown 就不 merge。

不需要为了更少 knot 放松 tolerance。

---

## B1.S2 — Full5D Soft Checkpoint

一次运行：

```text
359→361
351→711
near-limit
orientation zero/nonzero tolerance
adaptive refine/decimate
max rotary step
midpoint merge counterexample
entryBoundary/fence
budget exhaustion
repeat determinism
```

### S2 实施记录（2026-09-17）

本节原实施记录的能力解释以 S2 收口为准：Off 验证/归一化、硬超限失败；Conservative
提供旋转连续性、硬步长细分、严格等价 merge、evaluator 重建和软限位。
Full 无区间 bound 时执行 Conservative 等价子集并报告拒绝原因；非零 smoothing 未启用。
生产 `boundPhysicalAxes` 尚无资格来源，位置/姿态容差驱动细分是 infrastructure-only；
速度/加速度 proxy、反转计数是 audit-only。生产 policy 仍 Off，不开放 controller admission/RTCP。
收口修复和最终定向验证见 `B1_S2_CLOSEOUT_REPORT.md`。O2.1 热路径复用、D1 直接分段数、
D2 最大等价区间合并延后，不作为新增 S3 门禁。

- 新增 `full5d_optimizer`：既有完整 IK 输出作为 Raw reference；生产导出统一经过显式 Optimized reference。保留原 physical layout，不新增 IK 或 DOF reduction。
- 保留已解算多圈角度；wrapped observation 必须显式指定，半圈方向歧义与软限位越界拒绝。Conservative 使用 evaluator 重建新增/保留 knot 的 TCP、reference TCP、process direction。
- 细分采用 rotary hard bound；显式 position/orientation chord 限额要求共享 evaluator 的保守区间界，Unknown/预算耗尽/取消整次拒绝，不扩大 tolerance。严格 affine 中点等价证明允许合并；不跨 source span 端点、fence、semantic barrier 或 entry ownership。
- 生产 FK adapter 只读取 frozen machine/layout/setup/head/mounts/locked targets；测试覆盖 XYZ、AC/BC 转台、AB 摆头倾斜解及 AC 摆头零姿态。既有 AC_HEAD 倾斜 fixture 被 authoritative IK 判为奇异，保持拒绝，不由 optimizer 补解。
- 当前没有独立用户优化策略与非零工艺姿态 tolerance authority：生产默认仍为 Off，所有计算参数进入 capture identity；Off 超过 hard rotary step 直接拒绝，不偷偷细分。非零 smoothing 请求记录 rejection，保留 strict reference。模式配置/admission/reduction 属于 S3。
- 数值 fixture：`351→711` 在 5° hard bound、multiplier=128 下为 2→129 knots；affine `359,360,361` 为 3→2；非线性 position/orientation 区间界为 2→17。无 collision backend 调用。
- 验证记录：Ninja Debug / ASan 完整构建成功，S2 定向各 5/5。全量首跑 51/53（BVH 计时、资产 timeout），失败项隔离复跑 2/2；最终排除两项已验证重型测试的回归为 50/51，BVH 并行计时 5032 ms 超过 5000 ms，随后与 Full5D 隔离复跑 2/2。没有修改计时阈值，不宣称单次全量全绿。未进行实机验证，不代表 controller qualification 或 B1 Stage Gate；代码留在工作区，未提交/推送。

通过后自动继续。

---

# 第四阶段：DOF Reduction + Z Candidate

## 开发步骤 15：完成 physical active-axis analysis

每轴计算：

```text
span
monotonicity
local delta
dynamic contribution
freeze residual
```

不要先设计具体 C-only shortcut；先把通用分析做对。

---

## 开发步骤 16：实现 reduced candidate generation

基于 active-axis analysis 生成：

```text
SingleAxis
2D
3D
Reduced4D
Full5D fallback
```

然后再对典型：

```text
C-only
U+C
```

补专门候选构造。

每个 candidate 都必须做 FK/evaluator continuous comparison。

---

## 开发步骤 17：补 cumulative tiny-motion negative case

这是必须的专门反例：

```text
很多步每步变化极小
但累计变化明显
```

不得被 constant-axis 判定冻结。

---

## 开发步骤 18：实现 numerical Z canonicalization

只解决 numerical noise。

此时不引入 process relaxed tolerance。

---

## 开发步骤 19：接入 process Z-hold whitelist

如果仓库已经有完整 focus/standoff/process envelope：

```text
实现一个受支持 whitelist case
```

如果没有：

```text
完成接口、rejection reason、默认关闭
```

不要因此阻塞整个 B1。

---

# 第五阶段：Controller Admission + Deterministic Selection

## 开发步骤 20：冻结 optimizationMode 行为

验证：

```text
Off
Conservative
Full
```

实际启用 pass 与主规划一致。

同时确认：

```text
enableDofReduction
enableLaserZHold
```

确实是 feature gate，不是 UI 装饰字段。

---

## 开发步骤 21：controller-mode/capability admission

对所有候选统一做：

```text
PhysicalAxes/RTCP compatibility
capability revision
feed/dynamics representability
```

不支持的 candidate 在此淘汰。

不要留到 Process/lowering fallback。

---

## 开发步骤 22：实现 deterministic selection

首版优先使用 deterministic lexicographic cost。

稳定 tie-break 至少包括：

```text
block/source order
candidate type
activeAxisMask
stable ordinal
```

重复运行同一输入必须选同一个 candidate。

---

# 第六阶段：Derived Rebuild + Atomic Publication

## 开发步骤 23：重建 selected motion derived fields

选中 candidate 后统一重新生成：

```text
world TCP
reference TCP
process direction
source mapping
source spans
fences
feed/duration evidence
tolerance proof
metrics
```

不要把 candidate generation 阶段的临时缓存直接当最终执行字段。

---

## 开发步骤 24：生成 optimizer report

每 block/contour 至少记录：

```text
knots before/after
blocks before/after
max deviations
rotary travel/reversal
selected class
rejected candidate reasons
estimated commands
compile time
parameter provenance
```

`changed=false` 是合法成功状态。

---

## 开发步骤 25：owner-thread final revalidation + atomic commit

提交前重新比较：

```text
generation
source/order
context
controller mode
capability revision
policy
cancel
```

任意 stale：

```text
discard whole result
```

没有 partial publication。

---

## B1.S3 — Reduction / Publication Soft Checkpoint

### S3 实施记录（2026-09-17，基线 `9ca7222`）

`dof_reduction` 已接入 Optimized Full5D 后的生产导出链。通用 physical-axis analysis
统计整个 block（含 entry）的 span/travel/local delta/反转/dynamics proxy/freeze residual；
枚举全部合法物理轴子集，覆盖 C-only、U+C、3D、4D。精确常量轴通过分段 affine
端点恒等证明整个区间 hold；不对局部小增量设冻结阈值。节点保留完整 physical layout，
block active mask 表达 inactive-axis hold，FinalMotionPlan finalizer 检查 hold 与资格快照。

数值 Z 使用 compiler-v1 的 `64 * machine epsilon * max(1, abs(anchor))`，只消耗浮点误差预算。
进入/终止边界保持不变，末点不等于 canonical anchor 时拒绝该数值候选，防止跨 block 偷改路径。
planar/table 的独立 linear Z 提供解析平移上界，缩放安装、Z 驱动工件/旋转轴和 head 模式不推断该证明。
所有 changed candidate 经共享 evaluator 重建 TCP/reference/direction；证明不可用回到 Optimized reference。
重建使用 evaluator 的冻结 physical-knots 批量入口：一次 block identity 校验后求值精确节点，
保留逐点取消、有限值校验及原子输出；避免 1001 点回归中逐节点重新哈希整个 block 的二次开销。

Controller admission 检查请求模式、qualification hash/revision、active-mask capability 和 frozen feed/dynamics hash。
生产没有 qualification service，默认 authority 不可用；测试资格仅存在于测试 fixture。
process Z envelope 类型已定义，但缺少正式工艺来源/whitelist，启用时记录稳定拒绝原因；实际工艺 Z-hold 不开放。

策略来自现有 `CamConfig` 的 `[trajectory]`：`optimizationMode = "Off" | "Conservative" | "Full"`、
`enableDofReduction`、`enableLaserZHold`，默认 Off/false/false。未知模式/错误类型拒绝编译。
开关、候选预算、数值误差策略和 cost revision 纳入 parameter provenance/context hash。
按 stops、duration、soft-limit proximity、rotary reversals、normalized travel、knots、active-axis count、mask、ordinal
进行确定性字典序比较；active-axis count 只作后置 tie-break，数值 Z 的最后 tie 优先 canonical。
有限软限位区间用于 travel 归一化，否则使用 `max(1, abs(entry axis))`，规则随 cost revision 冻结。
首版候选不改变 stop/feed/time/source/fence，消耗的 process error budget 为零。预算耗尽保留原 block，与 collision mode 无关。
最终记录每 block 选择、拒绝原因、轴 span、数值 Z、候选数和非 identity 的耗时，发布前再次比较 owner authority。

验证：Ninja Debug (`acs-gtn-debug`) 与 ASan (`asan`) 完整构建通过；两套定向 CTest 均 7/7 通过，
覆盖 `dof_reduction`、`full5d_optimizer`、`cam_algorithm_pipeline`、`cam_motion_foundation`、
`cam_motion_plan_contract`、`cam_lead_in`、`project_package`。最终耗时 Debug 17.02 秒、ASan 42.04 秒。
首次 ASan 的 1001 点降维测试因逐点全块哈希超时；改为冻结块批量求值后，该项 Debug 0.48 秒、ASan 1.31 秒通过，
保留 60 秒超时不变，并新增批量入口 stale identity/取消原子性回归。架构检查与 `git diff --check` 通过。
本次为 S3 Soft Check，待用户人工 review；未执行 B1 Stage Gate、GUI 或物理机验证，未提交/推送。

一次运行：

```text
true/false C-only
U+C
1001 tiny increments
numerical Z
process Z-hold enabled/disabled/rejected
unsupported controller mode
capability stale
deterministic tie
same input -> same planHash
cancel/stale
entryBoundary
derived field consistency
Off/Conservative/Full
```

通过后进入正式 Stage Gate。

---

# 最终 B1 Stage Gate

只执行一次完整 B1 验收。

运行：

```text
V-007..V-016
B1.S1 accumulated suite
B1.S2 accumulated suite
B1.S3 accumulated suite
direct CAM/kinematics regressions
```

特别检查：

```text
collision backend calls == 0
no raw sampled fallback
no tolerance inflation
no stale partial publish
same input/context/policy => same planHash
```

输出一份简洁 Stage Summary：

```text
commit range
major components completed
before/after metrics
validation results
known deferred items
R1 review request
```

然后交给 Astra R1。

---

# Astra R1 后续

`PASS`：

```text
B1 complete
B2 released
```

`PASS_WITH_PATCH`：

只修真正影响 B2 contract 的问题。

禁止因为：

```text
GTN 实机
HIL
production collision proof
generic Z-hold
```

尚未完成而阻断 B1；这些属于后续批次。
