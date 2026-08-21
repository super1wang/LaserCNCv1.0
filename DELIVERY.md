# LaserCNC 当前交付状态

更新日期：2026-08-21

源码基线：`47e5408`

## 当前结论

当前版本可作为继续开发、自动化回归和 SDK 模拟器集成的基线；不建议标记为实体机生产发布。完整原因见 [AUDIT.md](AUDIT.md)，实施顺序见 [todo.md](todo.md)。

本轮文档更新没有修改业务源码，也没有宣称修复审计发现。它完成了全项目文件级复核、架构事实同步、历史文档去重和最新 TODO 重排。

## 已确认能力

- Kernel 按 `cad -> cam -> {simulation, process}` 编排模块，失败/关闭按反向顺序清理。
- 每工作区使用统一 Workpiece+CAM XCAF 文档与独立 `CamDataManager`；机台由独立 `MachineWorkspace` 持有。
- CAM 唯一生产轮廓顺序、偏置、Retract/Traverse/Approach、物理轴坐标和碰撞验证快照。
- Process 只消费 OCC-free 快照，对 Pending/Indeterminate/过期碰撞状态失败关闭；真实控制器失败不回退 PureSimulation。
- 工程包 v4 具备 `tools.toml`、staging 原子替换、失败保留旧包、机台指纹门禁和受控离线升级工具。
- CAM 已按 contracts/pipeline/toolpath/collision/machine/display/integration 等目录拆分，求解失败不提前破坏已提交运动计划。
- 自动化测试按 algorithm/flow/safety/sdk-integration/support 分层，并使用真实 `model/半球.stp` 覆盖制造流程。

## 本轮验证

| 验证 | 结果 |
| --- | --- |
| 架构脚本 | 通过。 |
| 日常 ACS+GTN Debug 构建 | 通过。 |
| 日常完整 CTest | 35/35 通过，57.08 秒。 |
| real-laser Debug 构建 | 可链接，但暴露 ULTRON 未初始化使用/缺失返回及多项旧协议告警，不满足真实激光启用条件。 |

## 发布阻断项

1. 修复并回归真实激光 ULTRON 的释放后使用、未初始化变量和缺失返回。
2. 给 ACS/GTN 无界等待增加 deadline/cancellation，证明 Stop/关机不被轮询或供应商调用无限阻塞。
3. 完成 CAD detached 文档事务，禁止 worker 直接写活动 XCAF。
4. 完成连续段碰撞安全证明、完整机台性能门限和碰撞后重规划策略。
5. 补 GUI、ASan/Application Verifier、长稳和物理 ACS/GTN/激光证据。

自动化通过、SimulatorCMHP、GUI 人工验收和物理机证据是四类独立证据，交付记录必须分别标注。
