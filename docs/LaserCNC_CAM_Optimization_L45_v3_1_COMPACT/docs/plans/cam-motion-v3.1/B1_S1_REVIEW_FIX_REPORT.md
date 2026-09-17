# B1.S1 复审修复记录（v3.2.1）

日期：2026-09-17。基线：`3722c128910e74a90500880896c7b7f29d6007d2`。
远端：`https://github.com/super1wang/LaserCNCv1.0.git`；分支：`codex/cam-motion-v3.2.1-b1`。
本记录替代上一轮收口报告中的放行结论，不改写该报告的历史验证结果。

结论：**B1.S1 = PASS（本次修补后的软件 Soft Check）**。B1 整体仍 In Progress；按原规划允许进入 controller-independent S2，本轮没有实施 S2。

## 修复范围

### C1：计算前捕获，而非发布时补身份

- 全局生成在提交 TaskManager 前捕获 `shared_ptr<const MotionCompilationInput>`；源、旧轮廓、选面、提取参数及光束方向同样按值捕获。worker 无 owner/config/service 可变对象访问。
- 数值输入包含采样策略、离线轴、模式/layout/锁轴目标、安装变换、刀头几何、工件挂载、形状绑定和空程参数。身份覆盖实际离线轴值及模式定义，不只依赖配置服务摘要。
- 全局生成最终采样、自动排序、有序求解、分步求解、单轮廓重算、偏置/空程求解及 FinalMotionPlan 共享计算输入。发布保留原始 context，而不是计算完成后另行生成身份。
- owner 独立重新捕获当前 authority；策略指纹由当前值重新计算，不复制旧 hash。源 revision/order 在 worker 接纳或已提交结果 revision 门禁单独核对；计算产生的新 revision 不冒充外部 authority 变化。
- 手工顺序位于独立 LayerContainer：捕获其策略/顺序身份；异步求解回调显式比对提交时顺序；导出必须匹配已求解结果的完整顺序，不能只依赖旧 toolpath revision。
- 多轮廓单项重算不得在 machine/policy authority 变化后替其它旧解重新签发身份；此时要求完整有序重算。
- Controller qualification 保持 `Unavailable`、revision `0`，PhysicalAxes 仅 legacy requested mode。RTCP/资格依赖的 admission 不放开。

### C2：完整区间证明、源覆盖与硬预算

- 新增 `geometry_source_sampler`，将完整性拆成 `sourceCoverageComplete && requiredFeaturesPreserved && refinementCriteriaSatisfied`，保留明确失败原因。
- 非退化源边采样失败、端参数未覆盖、必需屏障不存在/越界/预算不足均不能完整。OCC 标记的退化边无正长度源区间，允许跳过；其上的必需参数锚点无法解析时仍不完整。
- 每个最终同源边相邻区间同时满足弦差上界、整个区间切向/法向变化界和最终端点角度门槛。不能用“首—中、中—尾各合格”替代“首—尾合格”。
- 直线、圆、椭圆使用解析导数界；B-spline 保留原始 knot 边界作为硬屏障，使用分段有理/非有理 Bézier 控制点凸包约束一、二阶导数，不跨非 C1 节点伪证。平面/圆柱/球面法向使用同源精确投影及方向锥；法向选择切换、向外翻转、横截面修正不能证明时继续细分或 incomplete。
- 未支持的曲线/曲面或未知内部振荡不通过固定探测点获得 certificate。最大深度、最小参数跨度或硬预算用尽也保持 incomplete。
- 先保留所有原始种子槽位，每个屏障/引入锚点/递归中点消耗一个剩余槽位；最终输出绝不超过原始种子数乘 multiplier，未处理区间端点不会突破上限。
- `.lcnc` 保存三项 evidence、失败原因和 proof version 2；旧文件缺少当前证明版本时保留显示点，但要求重新采样。

### C3：失效后的生产消费门禁

- 保留集中 `replaceGeometrySamples` 清除全部 MachineCoord/SolvedPose 和 lead-in 求解结果。
- 实际 `exportToolpathSnapshotForOrder` 在偏置、空程、计划发布前检查采样证据、最新求解、冻结 authority 和已提交 revision。旧 pose 或旧 authority 不生成 FinalMotionPlan。

## 回归覆盖

- `cam_lead_in_test`：0/4/8°、未知内部法向振荡、源覆盖失败、缺失/越界屏障、全局硬预算、最终相邻角度；保留闭合、反向、裁剪和缝合测试；增加有理/非有理 spline 弦差与切向验证。
- `cam_algorithm_pipeline_test`：实际 CAM owner 捕获后策略变化拒绝；实际 TaskManager 求解任务提交后修改策略，owner 回调必须丢弃；同一输入用于真实 solve 并进入 FinalMotionPlan context hash；发布前策略变化拒绝；实际导出拒绝几何失效旧解；权威重算更新数值并恢复门禁。
- `project_package_test`：三项采样证据/失败原因 round-trip，已有屏障及不完整状态 round-trip。
- 半球真实 STEP 制造流程保持原来的成功标准，不以修改期望来掩盖采样回归。

## 验证状态

最终版本验证结果（2026-09-17）：

| 检查 | 结果 |
| --- | --- |
| 日常 `acs-gtn` / `acs-gtn-debug` Ninja Debug 全程序构建 | PASS |
| `ctest --test-dir build-cmake --build-config Debug --output-on-failure --parallel 4` | **52/52 PASS，290.37 s** |
| `asan` preset 全程序构建，独立 `build-cmake-asan` / `x64/ninja-asan/Debug` | PASS |
| ASan 定向 CTest：project_package、cam_lead_in、cam_algorithm_pipeline、flow_cam_pipeline、cam_motion_foundation | **5/5 PASS，33.95 s** |
| `git diff --cached --check` | PASS |

完整 52 项包含架构层级/Process OCC 边界、翻译、B0 motion plan/evaluator、旧制造流程、机床安全索引、SDK 仿真及启动 smoke。ASan 定向命令为：

```powershell
ctest --test-dir build-cmake-asan --build-config Debug --output-on-failure -R "lcnc_cam_lead_in_test|lcnc_cam_algorithm_pipeline_test|lcnc_project_package_test|lcnc_flow_cam_pipeline_test|lcnc_cam_motion_foundation_test"
```

首轮全量为 51/52：半球 STEP 的 B-spline 边缺少导数界、且存在必须保留的 knot 边界而被安全拒绝；补齐上述区间证明后，保持原用例成功标准，最终全量复测通过。没有改为容忍半球求解失败，也没有关闭新采样门禁。

交付为本分支的一个本地修补提交；本轮不执行 push。既有未跟踪模型、压缩包、审计附件与 `.codex/` 未纳入提交。

## 边界

本轮仅 S1 修复，不实施 S2，不执行任何实体机动作。软件测试、SDK 测试和 ASan 不代表控制器资格或实机加工批准。按照本次审阅意见，修复及 accumulated suite 通过后可直接进入 controller-independent S2，不另设正式 Stage Gate；资格依赖功能始终关闭。
