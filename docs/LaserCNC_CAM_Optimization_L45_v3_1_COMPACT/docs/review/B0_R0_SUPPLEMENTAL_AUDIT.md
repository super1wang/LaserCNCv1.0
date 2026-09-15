# LaserCNC CAM Motion v3.1 — B0 / Astra R0 补充审阅报告

> 审阅对象：`super1wang/LaserCNCv1.0`
> 分支：`codex/cam-motion-v3.1-b0`
> 审阅 HEAD：`e2cbf7902e801e295fdd44305047a390e281a00c`
> 提交：`fix(cam): 关闭运动基础合同R0审计项`
> 上一轮审计基线：`fcc3d173dc82faa8ae7d60d7ea46593cc1615081`
> 本轮结论：**PASS_WITH_PATCH**
> B1：**暂不放行；关闭 R06/R07 后可放行**

---

## 1. 补充审阅目的

上一轮 B0 审计提出 F01–F05：

```text
F01 canonical edge / block boundary 完整性
F02 evaluator model identity + bound 合法性
F03 Optional initial approach 非门禁化
F04 Required 仅接受 CertifiedSafe
F05 非法 collision mode 禁止 silent downgrade
```

当前提交已针对这些问题完成收口。

本次补充审阅不重新评估整个 B0，而是检查：

1. F01–F05 是否真正进入生产代码；
2. 是否存在从相邻调用入口绕过修复语义的问题；
3. 是否已达到放行 B1 的最低正确性条件。

---

# 2. F01–F05 收口结果

## F01 — Canonical Edge Ownership / Block Boundary

### 结果

**PASS**

当前实现增加：

```text
CamMotionBlock::entryBoundary
CamMotionBlock::hasEntryBoundary
```

后续 block 的 canonical incoming edge 由 `entryBoundary -> physicalKnots.front()` 唯一拥有。

`finalizeMotionPlan()` 已要求：

```text
first block: 不允许 entryBoundary
later block: 必须有 entryBoundary
entryBoundary == predecessor.physicalKnots.last()
source span 必须覆盖 entry boundary
process fence 必须覆盖 entry boundary
```

`entryBoundary` 被 block hash / plan hash 覆盖，同时不会作为重复 legacy node 投影。

新增 contract test 已检查：

```text
LeadIn
Cutting
Retract
Traverse
Approach
Cutting
```

跨阶段所有 canonical edge：

```text
无遗漏
无重复
projection node 顺序不改变
entry boundary 修改会使身份失效
```

### 判定

B1 可以依赖该 Block 边界语义。

---

## F02 — ContinuousMotionEvaluator Contract

### 结果

**PASS**

新增 `BoundMotionEvaluationContext`，只允许通过 finalized plan 一次绑定。

绑定检查：

```text
plan identity current
contextHash / planHash current
interpolationModelVersion exact match
block membership / block hash
```

`bound()` 已增加：

```text
physical axis bounds finite
world TCP bounds finite
min <= max
position error >= 0
orientation error >= 0
```

同时保留：

```text
PhysicalAxes:
  physical interpolation -> kinematics-derived TCP

RTCP:
  only qualified callback

missing conservative bound:
  reject
```

### 判定

正确性语义满足 B1 进入条件。

### 非阻断性能备注

当前 `supportsBlock()` 每次 evaluate/bound 仍会重新计算 `motionBlockHash(block)`。

由于 `motionBlockHash()` 遍历 knots / source spans / fences，B1 在 adaptive sampling / merge / candidate evaluation 中可能形成明显重复成本。

该问题不是 B0 correctness blocker。

建议在 B1 的 immutable compiler context 中优化为：

```text
bind once
  -> immutable block handle / frozen block reference
  -> O(1) or near-O(1) membership check
  -> hot evaluate/bound path
```

禁止简单移除保护而继续允许 block 可变。

---

## F03 — Optional Initial Approach

### 结果

**PASS（仅 initial approach 链）**

当前首刀逻辑：

```text
Disabled
  -> 不调用 certificate path
  -> 直接使用运动学合法候选

Optional
  -> 可以执行诊断
  -> 无论诊断 Collision / Unknown，都保留首个运动学合法候选

Required
  -> only CertifiedSafe
```

同时 Disabled 不再无条件依赖 workpiece shape。

### 判定

F03 原始问题已关闭。

但本轮完整调用链审阅发现，**Optional 在轮廓间 travel path 中仍存在另一处隐藏门禁**，见 R06。

---

## F04 — Required Certificate Eligibility

### 结果

**PASS**

当前：

```text
Process Required gate
InitialApproach Required gate
```

均显式要求：

```text
certificate.state == CertifiedSafe
```

因此：

```text
Disabled
Invalid
BoundaryUnknown
Blocked
```

均不能取得 Required 执行资格。

### 判定

原问题关闭。

---

## F05 — Invalid Explicit Collision Mode

### 结果

**PASS**

配置解析已区分：

```text
field missing
field valid
field explicitly invalid
```

只有字段缺失时才走 legacy migration。

显式非法或空字符串：

```text
collisionVerificationModeValid = false
effective policy -> Required direction
collision configuration -> invalid
activation -> fail-closed
```

非法原值保留并 round-trip，避免“自动修复后隐藏配置错误”。

### 判定

原问题关闭。

---

# 3. 新发现的补充收口项

## R06 — P1：Optional 仍可通过 TravelPlan failureReason 间接阻断执行

### 位置

```text
CamModule::attachTravelPlan()
TravelPlanSnapshot::isPathReady()
camExecutionBlockReason()
```

### 当前行为

Optional 仍满足：

```text
collisionConfigSnapshot.enabled == true
```

当碰撞环境不完整时：

```text
plan.collision.failureReason = collision diagnostic reason
plan.failureReason = plan.collision.failureReason
```

随后：

```text
TravelPlanSnapshot::isPathReady()
  = !stale && failureReason.isEmpty()
```

Process 对多轮廓执行无条件检查：

```text
travelPlan.isPathReady()
```

因此可能形成：

```text
motion path valid
+ Optional collision policy
+ collision resource missing / diagnostic incomplete
        ↓
travelPlan.failureReason set
        ↓
isPathReady() == false
        ↓
Process blocked
```

### 为什么这是 B0 blocker

Optional 的冻结语义是：

```text
diagnostic only
不得成为 path selection 或 execution eligibility gate
```

当前虽然 Process 的 collision gate 已放开 Optional，但 collision diagnostic 仍污染 path-level failure state，形成隐藏门禁。

### Required Fix

必须分离：

```text
Path Failure
vs
Collision Diagnostic Failure
```

正式规则：

```text
Path failure:
IK / endpoints / geometric route / axis limits / route semantics
-> travelPlan.failureReason
-> all modes block

Collision diagnostic failure:
missing package / overlay / unknown / collision diagnostic
-> travelPlan.collision.*
-> only Required can block
-> Optional must not set path failure
```

### Forbidden Fix

禁止：

```text
Optional -> 无条件 clear travelPlan.failureReason
```

因为这会把真正的路径失败一并隐藏。

必须在写入点区分 failure ownership。

### Required Test

至少：

```text
2 contours
motion path valid

Optional + collision resources missing
  -> travel path ready
  -> Process not blocked by collision diagnostic

Required + same missing resources
  -> blocked

Disabled/Optional + real IK/path failure
  -> still blocked
```

---

## R07 — P1/P2：Disabled 仍可能自动构建碰撞几何 / Coal Pair

### 位置

```text
CamModule::rebuildTravelPlanForCurrentOrder()
CamModule::scheduleCollisionSafetyDomainPreparation()
```

### 当前行为

travel rebuild：

```text
if fullEnvironmentVerificationPending
    scheduleFullEnvironmentVerification()
else
    scheduleCollisionSafetyDomainPreparation()
```

Disabled 通常不会进入 full verification，因此会进入 `scheduleCollisionSafetyDomainPreparation()`。

该函数当前根据：

```text
collision.valid
machine safety package ready
machine index available
overlay cache state
```

决定是否构建，但没有显式：

```text
if mode == Disabled return;
```

后续会执行：

```text
buildCollisionGeometry(...)
buildCoalCollisionPairs(...)
```

### 为什么这是 B0 blocker

B0 contract 明确：

```text
Disabled:
no machine geometry/package/overlay/certificate prerequisite
no collision backend construction/query
```

当前 no-model 测试可以 PASS，但在：

```text
machine model/package already loaded
+ mode Disabled
+ overlay stale/missing
```

情况下仍可能后台构建碰撞数据。

这违反的是**Disabled policy 本身**，不是单纯性能建议。

### Required Fix

所有自动碰撞 preparation/scheduling 入口必须以 policy 为第一判断：

```text
Disabled:
  no collision preparation task scheduled
  no overlay build
  no collision geometry build
  no Coal pair build
```

切换到 Disabled 后：

```text
abort/ignore stale collision prep task
generation guard prevents late publish
```

允许手工显式“验证碰撞”命令根据产品定义单独处理，但不能由普通 CAM motion generation 在 Disabled 下自动准备碰撞后端。

### Required Test

构造：

```text
machine geometry/package exists
collision mode = Disabled
overlay absent/stale
toolpath/travel rebuilt
```

断言：

```text
collision preparation scheduled = 0
buildCollisionGeometry calls = 0
Coal pair build calls = 0
```

同时：

```text
Optional / Required
```

可按各自策略继续诊断/认证准备。

---

# 4. 其他审阅结论

## 4.1 FinalMotionPlan 单一真相

未发现本次补丁新增第二执行轨迹真相。

`entryBoundary` 属于 block semantic boundary，并不投影成重复 legacy node，这个方向正确。

## 4.2 Device / Process safety

未发现本次补丁新增：

```text
second DeviceCommandQueue
SDK direct bypass
Stop/fault bypass
axis-enable bypass
```

R06/R07 的修补不得改变这些安全边界。

## 4.3 B1 handoff placeholders

B0 文档已正确声明：

```text
workspaceGeneration
setupCalibrationHash
controller capability hash
tool/process hash
dynamics semantic hash
```

部分仍是 transition/baseline identity。

这不是 B0 blocker。

B1 Workstream 7 必须接入真实 immutable compilation context。

---

# 5. 补充 Stage Gate

R06/R07 修复后，不再重跑完整 B0-CLOSE 12/12。

只需运行一次定向矩阵：

```text
T1 Optional multi-contour + missing collision environment
T2 Required multi-contour + same environment
T3 Disabled + machine/package present + stale/missing overlay
T4 Disabled/Optional + true travel/IK failure
T5 existing no-model CAM flow
T6 existing process cutting safety
T7 directly impacted travel path test
T8 existing collision foundation/config test
```

若 T1–T8 全部通过，且没有新的 architecture mismatch：

```text
R0 = PASS
B1_RELEASE = YES
```

---

# 6. 最终补充结论

```text
HEAD:
e2cbf7902e801e295fdd44305047a390e281a00c

F01 PASS
F02 PASS
F03 PASS for initial-approach scope
F04 PASS
F05 PASS

New adjacent-entry findings:
R06 OPEN
R07 OPEN

R0 = PASS_WITH_PATCH
B1_RELEASE = NO
```

只需一个小型 collision-policy-entry 收口提交。

R06/R07 关闭后，不建议继续扩展 B0，应立即放行 B1。
