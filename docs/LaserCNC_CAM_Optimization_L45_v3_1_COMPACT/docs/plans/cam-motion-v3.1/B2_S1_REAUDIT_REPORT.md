# LaserCNC CAM Motion v3.2.1 — B2.S1 复审审计报告

> 仓库：`super1wang/LaserCNCv1.0`  
> 分支：`codex/cam-motion-v3.2.1-b1`  
> 复审 HEAD：`27a867413544a4323e366440a16a83af8fb3c3a8`  
> 提交：`fix(process): close B2 S1 execution contracts`  
> 父提交：`0b6b3ae0177b550c657b69e0a02fe2e5c38007c1`  
> 日期：2026-09-18  
> **结论：B2.S1 = PASS_WITH_PATCH**  
> **B2.S2_RELEASE = NO**

---

## 1. 本轮复审目标

本轮仅复核上一轮 B2.S1 两个收口项是否完成，并判断是否具备进入 B2.S2 的条件：

```text
F01 complete frozen Tool execution recipe identity
F02 exact-plan pause boundary contract
```

同时检查修复是否引入新的生产路径异常。

不扩展到：

```text
PhysicalAxes lowering
RTCP lowering
Group / LookAhead
B2 Stage Gate
HIL
real-machine qualification
production collision qualification
```

---

## 2. 总体判断

上一轮两个原始缺口均已按正确方向完成：

```text
F01 execution recipe architecture = CLOSED
F02 exact-plan pause contract      = CLOSED
```

当前新增：

```text
FrozenToolExecutionRecipe
PreparedExecutionSection
IExactSectionSink
executePreparedSections()
```

使 B2.S1 的交付边界明显更清晰：

```text
FinalMotionPlan
    ↓
PreparedDeviceProgram
    ↓
FrozenToolExecutionRecipe
    ↓
PreparedExecutionSection[]
    ↓
B2.S2 controller lowering
```

但复审发现一个新的生产入口兼容缺陷：

```text
F03 production Tool load / arc-profile mirror = OPEN / P1
```

所以当前不能直接放行 B2.S2。

---

# 3. 已关闭项

## F01 — Frozen Tool execution recipe identity

### 结论

**CLOSED**

当前实现不再直接以 `Tool::toTable()` 作为执行 identity。

新增：

```text
FrozenToolExecutionRecipe
frozen_tool_execution_fields.inc
frozenToolExecutionRecipeHash()
validateToolExecutionRecipe()
freezeToolExecutionRecipe()
```

允许进入执行层的字段由一份有序 schema 同时参与：

```text
declaration
owner-thread capture
canonical hash
finite-value validation
```

identity 还包括：

```text
toolName
sourceId
profileSchema
```

这解决了上一版 Tool 内容变化但 recipe hash 不变化的问题。

### 轨迹修改语义

当前明确拒绝以下 legacy path-changing semantics：

```text
axis preset / move
path extension
geometry compensation
Z linkage
servo-height path alteration
flight cutting
trough
punch
independent native arc / PD special modes
```

因此 S2 只能消费显式 frozen execution DTO，不能重新解释 legacy Tool 来改变 CAM FinalMotionPlan。

### Bypass 防护

Process owner capture 会移除 committed settings 中的旧 `Setting.Tool` 子表，并且 `PreparedDeviceProgram::prepare()` 也拒绝该旁路重新出现。

这一方向符合：

```text
Process does no post-publication trajectory transformation
```

---

## F02 — Exact-plan pause contract

### 结论

**CLOSED**

当前已建立：

```text
PreparedExecutionSection
IExactSectionSink
executePreparedSections()
```

首版 pause-safe section 按连续 CAM contour 分组。

每个 section 冻结：

```text
ordinal
contourId
first/last block index
first/last block id
first/last block hash
planHash
runEpoch
```

### 运行语义

```text
prepareExactSection(section N)
    ↓
pause / stop admission check
    ↓
startExactSection(section N)
    ↓
wait until section N drains
    ↓
checkpoint
    ↓
if paused -> do not start N+1
```

同一次 live run 始终使用同一个：

```text
PreparedDeviceProgram
planHash
contextHash
recipeRevision
runEpoch
```

Start failure 不重试；old checkpoint 的重新进入不作为自动 replay 授权。

### 测试覆盖

现有 closeout fixture 已覆盖：

```text
pause during section
pause then Stop
paused before first Start
pause during prepare
pause during queued health check
Start failure no replay
same program/runEpoch identity
no duplicate predecessor motion
```

因此上一轮 F02 可以正式关闭。

---

# 4. 新发现 F03 — P1：真实 Tool 加载与 native-arc neutral 判定冲突

## 4.1 当前 Frozen capture

`freezeToolExecutionRecipe()` 要求：

```text
m_dArcVelocity == 0
m_dArcAcc      == 0
m_dArcJerk     == 0
```

目的是避免 B2 重新启用 legacy/native arc profile。

这个安全目标本身正确。

---

## 4.2 真实 Tool::SetFromTable 行为

当前生产 Tool 加载中：

```cpp
if (tryGetDouble(t, "fCutAcc",  m_dLineAcc))  ++n;
if (tryGetDouble(t, "fCutJerk", m_dLineJerk)) ++n;

m_dArcAcc  = m_dLineAcc;
m_dArcJerk = m_dLineJerk;
```

也就是说，即使配置中没有启用任何独立 arc/native interpolation：

```text
fCutAcc  = 1000
fCutJerk = 10000
```

正常加载后也会形成：

```text
m_dLineAcc  = 1000
m_dLineJerk = 10000
m_dArcAcc   = 1000
m_dArcJerk  = 10000
```

这是现有兼容行为，而不是用户选择了独立 arc profile。

---

## 4.3 生产结果

生产数据链：

```text
ProcessSettings
    ↓
Tool::SetFromTable()
    ↓
ToolFactory::SetTool()
    ↓
ToolFactory::executionRecipes()
    ↓
freezeToolExecutionRecipe()
```

当前会因为：

```text
REQUIRE_ZERO(m_dArcAcc)
REQUIRE_ZERO(m_dArcJerk)
```

拒绝一个正常 real-run Tool。

而真实执行又要求：

```text
lineVelocity > 0
lineAcceleration > 0
lineJerk > 0
```

所以当前形成矛盾：

```text
正常 real-run dynamics
        ↓
SetFromTable mirrors line acc/jerk into arc acc/jerk
        ↓
Frozen capture treats mirror as unsupported native arc semantics
        ↓
PreparedDeviceProgram cannot be built
```

一旦 B2.S2 开始实现真实 lowering，这会成为生产阻断。

---

## 4.4 为什么现有测试没有发现

现有 prepared-program fixture 手工创建：

```cpp
Tool source;
source.m_dLineVelocity = 100;
source.m_dLineAcc = 1000;
source.m_dLineJerk = 10000;
```

没有经过：

```text
Tool::SetFromTable()
```

所以：

```text
m_dArcAcc == 0
m_dArcJerk == 0
```

该 fixture 无法复现真实生产 Tool load。

因此当前 11/11 证明了 frozen DTO 合同，但还缺：

```text
real Settings / TOML
→ Tool::SetFromTable
→ ToolFactory
→ freezeToolExecutionRecipe
```

端到端入口回归。

---

# 5. 推荐修复

不应删除 native-arc 防护，也不需要重新设计 FrozenToolExecutionRecipe。

只需区分：

```text
compatibility mirror
vs
independent native arc profile
```

推荐规则：

```text
m_dArcVelocity == 0
```

继续要求 neutral。

对于：

```text
m_dArcAcc
m_dArcJerk
```

允许现有兼容镜像：

```text
m_dArcAcc  == m_dLineAcc
m_dArcJerk == m_dLineJerk
```

但如果：

```text
m_dArcAcc != m_dLineAcc
or
m_dArcJerk != m_dLineJerk
```

继续 fail-closed：

```text
unsupported independent native arc profile
```

这样不会让 S2 获得 arc trajectory 权限。

---

# 6. 必需回归

新增生产式测试：

```text
T1
TOML:
  fLineVel  = 100
  fCutAcc   = 1000
  fCutJerk  = 10000

Tool::SetFromTable()
    ↓
assert arcAcc  == lineAcc
assert arcJerk == lineJerk
    ↓
freezeToolExecutionRecipe()
    ↓
PASS
```

Negative：

```text
T2 arcVelocity > 0
=> reject

T3 arcAcc != lineAcc
=> reject

T4 arcJerk != lineJerk
=> reject
```

并保留当前：

```text
NaN/Inf
path-changing legacy fields
fallback Tool
pause/no-replay
```

回归。

---

# 7. 文档状态

当前文档仍有事实滞后：

```text
B2_S1_IMPLEMENTATION.md
```

仍描述“收口工作区 / 尚无收口 commit”。

实际收口已提交：

```text
27a867413544a4323e366440a16a83af8fb3c3a8
```

`STATUS-MATRIX.md` 也仍以：

```text
0b6b3ae + S1 收口工作区
```

描述 B2。

这些为 P2，不阻塞算法修复，但建议与 F03 同一提交修正。

---

# 8. 测试证据

仓库收口记录：

```text
Debug full build        PASS
direct regressions      11/11 PASS
elapsed                 8.04 s

ASan full build         PASS
same suite              11/11 PASS
elapsed                 20.44 s
sanitizer reports       none

architecture check      PASS
git diff --check        PASS
```

当前 GitHub commit 没有远端 workflow/status。

因此以上为：

```text
repo-recorded local Windows Debug/ASan evidence
```

不是独立远端 CI。

---

# 9. Release Matrix

| Contract | Result |
|---|---|
| Exact FinalMotionPlan cutover | PASS |
| FrozenToolExecutionRecipe architecture | PASS |
| execution identity completeness | PASS |
| path-changing legacy rejection | PASS |
| Tool real-load compatibility | **OPEN / P1** |
| PreparedExecutionSection | PASS |
| pause/resume same-run | PASS |
| Stop dominates Pause | PASS |
| no Start replay | PASS |
| no real legacy fallback | PASS |
| DeviceCommandQueue authority | PASS |
| controller qualification fail-closed | PASS |
| collision policy | PASS |

最终：

```text
B2.S1 = PASS_WITH_PATCH
B2.S2_RELEASE = NO
```

关闭 F03 后：

```text
B2.S1 = PASS
B2.S2_RELEASE = YES
```

无需再修改 pause architecture，无需重跑 B1 Gate，无需提前执行 B2 Stage Gate。
