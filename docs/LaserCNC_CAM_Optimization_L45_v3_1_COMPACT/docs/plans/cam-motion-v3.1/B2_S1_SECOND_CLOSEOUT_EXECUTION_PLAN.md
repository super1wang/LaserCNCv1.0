# LaserCNC CAM Motion v3.2.1 — B2.S1 第二次收口执行规划

> 分支：`codex/cam-motion-v3.2.1-b1`  
> 审计 HEAD：`27a867413544a4323e366440a16a83af8fb3c3a8`  
> 当前：`B2.S1 = PASS_WITH_PATCH`  
> 目标：`B2.S1 = PASS` / `B2.S2_RELEASE = YES`  
> 本轮只有一个代码 P1：`F03 production Tool load compatibility`

---

# 1. Scope

只处理：

```text
C1 production Tool::SetFromTable → FrozenToolExecutionRecipe compatibility
C2 direct regression
C3 documentation truth
```

禁止扩展到：

```text
PhysicalAxes lowering
RTCP lowering
controller qualification
Group / LookAhead
pause architecture redesign
B2 Stage Gate
HIL / real-machine validation
```

---

# 2. Frozen Invariants

保持以下已经通过的合同：

```text
INV-01 FinalMotionPlan remains the only real trajectory
INV-02 FrozenToolExecutionRecipe remains S2 tool input
INV-03 S2 never receives legacy Tool trajectory-changing semantics
INV-04 independent native arc profile remains unsupported
INV-05 pause section architecture remains unchanged
INV-06 no automatic replay
INV-07 DeviceCommandQueue remains single device authority
INV-08 production controller remains Unavailable / revision 0
```

---

# 3. C1 — Fix Production Arc Compatibility Mirror

## Step 1 — Preserve arc-velocity rejection

继续拒绝：

```text
m_dArcVelocity != 0
```

理由：

```text
native / independent arc feed is not qualified in B2.S1
```

不要把它加入 FrozenToolExecutionRecipe。

---

## Step 2 — Treat existing acc/jerk mirror as inert compatibility data

当前 Tool loader 自动：

```cpp
m_dArcAcc  = m_dLineAcc;
m_dArcJerk = m_dLineJerk;
```

因此修改 frozen capture 为：

```text
accepted:
  m_dArcAcc  == m_dLineAcc
  m_dArcJerk == m_dLineJerk

rejected:
  m_dArcAcc  != m_dLineAcc
  m_dArcJerk != m_dLineJerk
```

可使用 exact equality，因为它们是直接赋值产生，不是计算结果。

---

## Step 3 — Do not expose mirror fields to S2

`FrozenToolExecutionRecipe` 不需要增加：

```text
arcAcceleration
arcJerk
```

除非未来存在正式 qualified native-arc capability。

当前：

```text
lineAcceleration
lineJerk
```

仍是唯一执行字段。

也就是说：

```text
legacy mirrored arc acc/jerk
= capture compatibility check only
= not execution authority
```

---

# 4. C2 — Production-Style Regression

新增测试必须从真实解析入口开始，而不是手工构造 Tool。

## Positive case

```text
toml::table toolTable:
  fLineVel = 100
  fCutAcc  = 1000
  fCutJerk = 10000
```

执行：

```text
Tool tool;
tool.SetFromTable(toolTable);
```

断言：

```text
tool.m_dLineAcc  == 1000
tool.m_dLineJerk == 10000
tool.m_dArcAcc   == 1000
tool.m_dArcJerk  == 10000
```

然后：

```text
freezeToolExecutionRecipe(tool, ...)
```

必须成功。

---

## Negative 1

```text
tool.m_dArcVelocity = nonzero
```

必须拒绝。

---

## Negative 2

```text
tool.m_dArcAcc = tool.m_dLineAcc + delta
```

必须拒绝。

---

## Negative 3

```text
tool.m_dArcJerk = tool.m_dLineJerk + delta
```

必须拒绝。

---

## Preserve existing regressions

继续保留：

```text
Frozen DTO every-field identity test
NaN / Inf test
path-changing legacy-field rejection
fallback recipe rejection
recipe mutation rejection
pause/resume six scenarios
Stop precedence
Start failure no replay
section identity
```

---

# 5. C3 — Documentation Truth

更新：

```text
B2_S1_IMPLEMENTATION.md
STATUS-MATRIX.md
```

将状态修正为：

```text
S1 initial: 0b6b3ae...
S1 closeout: 27a8674...
S1 F01/F02 closed
F03 closeout: <new commit>
```

最终：

```text
B2.S1 PASS
B2.S2_RELEASE YES
```

不要再写“工作区尚未提交”。

---

# 6. Validation

只运行直接受影响 suite。

最低：

```text
process_cutting_safety
process_current_schema / tool-related regression if directly touched
process_workflow
device_command_queue
device_wait
confirmed_motion_stop
```

如果修改仅 frozen recipe + tests，可以缩小到：

```text
process_cutting_safety
tool/settings serialization-related direct regression
```

再执行：

```text
Debug affected build/tests
ASan affected tests
scripts/check_architecture.ps1
git diff --check
```

不要执行：

```text
B1 Stage Gate
B2 V-017..V-023 Stage Gate
HIL
real controller
```

---

# 7. Exit Criteria

```text
[ ] Tool loaded through SetFromTable with normal cut acc/jerk can freeze
[ ] arc compatibility mirror does not enter S2 DTO identity
[ ] independent arc velocity rejects
[ ] independent arc acceleration rejects
[ ] independent arc jerk rejects
[ ] existing FrozenTool identity tests still pass
[ ] existing pause/no-replay tests still pass
[ ] no new legacy execution fallback
[ ] no new controller qualification
[ ] docs reflect committed reality
[ ] no unresolved ESCALATE
```

全部满足：

```text
B2.S1 = PASS
B2.S2_RELEASE = YES
```

然后自动进入 B2.S2，不再开启第三轮 S1 架构收口。

---

# 8. B2.S2 Handoff Reminder

S2 此后只能消费：

```text
PreparedDeviceProgram
PreparedExecutionSection
FrozenToolExecutionRecipe
MachineAxisLayout
FinalMotionPlan
frozen ControllerMotionMode / qualification
```

S2 负责：

```text
PhysicalAxes lowering
RTCP lowering
explicit axis mapping
unwrapped turn preservation
trace/userTag identity
feed/dynamics unit qualification
unsupported cell rejection
```

S2 仍不得：

```text
re-run IK
re-plan contour
re-enable native arc semantics
restore legacy compensation
reselect candidate
modify pause section identity
```
