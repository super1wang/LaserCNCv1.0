# B1 R1 收口记录（2026-09-17）

审计基线 `aa8fd34625bca3581898915b12140f9cc5510cf1`，分支 `codex/cam-motion-v3.2.1-b1`。
本轮代码待人工 review；未新建提交或推送。R1 的人工复核结论与 B2_RELEASE 不由本记录代签。

## C1 / F01：结构边界与工艺事件

`MotionProcessFence` 增加 `changesLaserState` 与 `requiredStop`：仅 `changesLaserState=true`
时消费 `laserEnabledAfterFence`，结构 marker 不发 IO。Raw builder 仅在连续 Cutting 轮廓的真正
开始/结束产生开关光；source edge、departure seam、结构屏障不产生额外工艺变更。
原有 phase/entry `-1` 位置保持；不新增穿孔或机械停止配方。

Full5D/Reduction 保留全部真实事件；成本只计 requiredStop，`costRevision=2` 纳入冻结参数。
block hash v4 / plan hash v3 包含新语义；旧 hash 不再自洽。FinalMotionPlan 为临时派生数据，
没有把旧持久化 fence 静默升级成新计划的入口。finalizer 校验事件位置/次序、状态变更、
source span 范围、layout/mask/class/优化状态组合，不能换 Optimized 标签绕过降维资格。
真正三轴布局仍合法。

## C2 / F02：当前发布身份与任务

沿用 owner capture、ToolpathGenerationStamp 和 TaskManager。发布 key 包括当前 context、
配置变更代际与 travel key；实际已解顺序和结果 revision 单独检查。
CamConfig 的 load/readFrom、赋值和 motion 相关 setter 均参与变更时钟（无效/no-op 设置不推进，
纯法线显示和渲染质量 setter 不影响 motion）；复制得到独立时钟，赋值保留并推进
目标对象时钟。后台 provider 只读冻结 record 和原子时钟，不访问 mutable 配置或阻塞调用 owner。
配置变化后即使几何 revision 不变，committed API 也返回明确 NotReady（空计划+原因），目录仍可读。

公共 provider 使用 `cam.motion.compile` TaskManager 任务运行冻结 Full5D/Reduction；取消令牌传到
两层算法及派生重建，配置变更也使 worker 取消。owner 再比较 key/order/context/源状态后一次采纳。
任务只持有冻结输入和结果；adapter 销毁后取消并断开回调，没有 worker 借用模块对象。
同步导出也使用同一已编译缓存；同 key 的读取不重新选择候选。
碰撞 worker 回传 verifiedMotionPlanHash，只有 exact hash 相同才能更新附件，proof-only 通知不重算优化。

`verifyProvider` 从实际注册服务读取，覆盖 mode/DOF/Z 的真实配置文件重读、旧计划拒绝、真实迟到
编译结果、TaskManager abort、取消后重试、并发 reader、配套 plan/context/report 和编译调用计数。

## C3 / F03：B1 Stage Gate

Ninja Debug 完整构建通过。合并 Gate **12/12 通过，51.70 秒**；随后仅补强 real-FK 测试的
X 轴实际移动量（0→0.25→0.5 mm 相对位移），生产代码未改，pipeline 单项复跑 **1/1，6.82 秒**。
ASan 完整构建通过，最终同组 **12/12 通过，52.76 秒**，无 sanitizer 报错。
架构检查和 `git diff --check` 通过。

命令（两条生成树顺序构建；各自独立运行）：

```powershell
cmake --build --preset acs-gtn-debug --parallel 16
cmake --build --preset asan --parallel 16
ctest --test-dir build-cmake --build-config Debug --output-on-failure -R '^lcnc_(dof_reduction|full5d_optimizer|cam_algorithm_pipeline|cam_motion_foundation|cam_motion_plan_contract|cam_lead_in|project_package|task_manager|cam_data_manager|machine_topology_solver|rtcp_reference_transform|toolpath_snapshot_concurrency)_test$'
# ASan 使用完全相同的 -R 集合，将 test-dir 改为 build-cmake-asan。
```

构建在 VsDevCmd x64 环境完成。CTest inventory 已确认上述 target 名称；开发 targeted 失败与
本次合并 Gate 分开记录，没有把早期 S3 7/7 拼接成 B1 Gate。

| V-row | 实际 case / target | 本轮验证范围 |
|---|---|---|
| V-007 | cam_lead_in: verifySourceSeamAndAnchor / verifySourceTopologyAndTrimmedGeometry；pipeline 事件投影 | 接缝、屏障、winding/拓扑、结构边界不增生 IO |
| V-008 | cam_lead_in: trimmed line/arc/circle、spline bounds、sampling counterexamples | 保留真实 OCC source，不由稀疏点猜测 primitive |
| V-009 | full5d: liftPeriodicObservation、351→711、entry refinement；pipeline closed traversal | 多圈、旋转限位、反向及 native periodic source |
| V-010 | full5d: Off/strict affine/normal denoise；pipeline frozen FK | 已批准的严格子集与派生重建 |
| V-011 | full5d: rotary step/budget/chord bound；cam_lead_in bounded refinement | 硬预算与 unknown bound 拒绝，动态仍 audit-only |
| V-012 | full5d: affine merge、reversal/midpoint/fence 反例；pipeline 显式 Stop | 保护真实工艺事件及源 ownership |
| V-013 | dof_reduction: C-only/U+C/3D/4D/1001 increments；pipeline real FK | 真实 FK + 隔离测试资格；生产未资格保持拒绝 |
| V-014 | dof_reduction: 数值 Z/末边界/缺 envelope；pipeline real FK | XYZ、AC table、BC table 的生产解析 bound |
| V-015 | dof_reduction: stable hash、mode/capability/dynamics mismatch | 确定性、真实 stop 成本语义与 honest fallback |
| V-016 | pipeline: verifyProvider + actual async solve；motion_plan_contract；snapshot_concurrency | 公共缓存失效、late/cancel、proof-only、原子 record |

直接回归加入 task_manager、cam_data_manager、machine_topology_solver、rtcp_reference_transform、
toolpath_snapshot_concurrency；架构检查另运行。无需新增 B2/B3/B4 或实机 Gate。

初次 targeted 运行中 motion_plan_contract 的旧 fixture 未填写 axisMask，被新 finalizer 正确拒绝；
补齐 fixture 的真实布局及 class 后重跑。未降低算法阈值或修改超时。

## Metrics 与范围

以下为本轮 Debug 输出，位置量为算法报告的连续偏差上界，时间是整数毫秒诊断（0 表示不足 1 ms）。

| 路径 / 资格来源 | knots；blocks（前→后） | selected mask | 位置/姿态上界 | 真实 laser / stop 事件 | 编译 ms | planHash 前缀 |
|---|---|---|---|---|---|---|
| C-only / test-only qualification | 3→3；1→1 | 16（C） | 0 mm / 0° | 2 / 0，前后相同 | 1 | c1024d527a81ef36 |
| XYZ frozen FK + test-only admission | 3→3；1→1 | 1（X） | 3.55271e-15 mm / 0° | 0 / 0 | 0 | ed2d516db2880598 |
| AC table frozen FK + test-only admission | 3→3；1→1 | 1（X） | 7.10543e-15 mm / 0° | 0 / 0 | 1 | b66f6b63291c59e6 |
| BC table frozen FK + test-only admission | 3→3；1→1 | 1（X） | 7.10543e-15 mm / 0° | 0 / 0 | 1 | 8cf340870b7e1cb1 |
| 生产 owner Off / Unavailable | 3→3；3→3 | 原 XYZ 布局 | Not measured | 2 / 0，前后相同 | Not measured | 3d4424492734a390 |

C-only 的 rotary travel 为 0.02°，reversals=0，max rotary delta=0.01°，候选数=15；物理轴序列
保持不变，merge ratio=0。其余表内路径同样未减少节点；对应 rotary 指标、候选数和 command count
未单独输出计量，记为 Not measured。上述边界用例不代表整机吞吐收益。
Conservative/Full 的生产未资格场景在 public-provider 及 Full-policy 回归中验证诚实拒绝，未汇报降维收益。
相同输入重复 hash 断言、proof-only 优化调用计数不变及事件前后投影断言均在测试中执行。

测试资格只用于离线 fixture，生产仍 `Unavailable / revision 0`，默认 Off。
非零平滑、通用生产区间 bound、process Z whitelist、controller lowering、HIL 与实机不在本次通过范围。
worker 私有 FK 状态复用、candidate invariant cost 复用、最大 affine merge 继续 Deferred。
Raw 构造/空程准备及独立冷同步导出仍在 owner；provider 的最终优化已进入任务，未测 GUI 响应性。

## 交付状态

F01/F02 已修复并完成对应回归；F03 的既定 B1 Gate 已执行并通过。
**B1 Stage Gate Passed / R1 Review Pending；B2_RELEASE=NO，等待用户人工复核。**
本次结果未把测试资格升级为设备资格，未实施 B2，未新建 Git 提交。
