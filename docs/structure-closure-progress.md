# LaserCNC 模块入口瘦身执行进度

更新日期：2026-08-03

本文件记录已批准的模块入口瘦身计划的阶段性进度。完成标记只代表源码和
列出的自动化验证；不会替代 ACS、GTN 或激光物理硬件验证。

| 阶段 | 范围 | 状态 | 证据 / 后续 |
| --- | --- | --- | --- |
| 1 | 共用模块任务生命周期骨架 | 已完成 | `ModuleTaskScope` 统一 CAD、CAM、Process 的任务跟踪、协作取消和有限等待；见 v1.3.2。 |
| 2 | Process typed executor 与入口下沉 | 进行中（主安全事务已修） | typed runtime、连接、预检、状态、workflow、手动运动和交互 IO service 已落地。普通 Stop 现等待安全输出/断开 completion 后才进入 `Stopped`；E-stop 开启 stop-only 门禁，交互 IO 被锁定，恢复经 executor 复核；状态服务以 generation 丢弃旧轮询 completion。仍需补全 Stop/E-stop 的失败、超时与 100 次关闭回归。 |
| 3 | CAM service 下沉 | 进行中（immutable/投影回归待修） | `ToolpathGenerationService` 的 stale-result 校验已落地；自动加工面以 workpiece entry、role、签名复用稳定 ID；AIS 按 document 保存 owning context 并在 context 更换前清理旧对象。仍需移除 mutable entries、增加双工作区 offscreen 回归，并迁移机台标定与其余投影。 |
| 4 | CAD service/controller 下沉 | 进行中（事务待修） | `CadDocumentIoService` 已承接新建、保存、关闭、STEP 导出、STEP/IGES/STL/BREP 解析及显示网格准备。关闭前现取消模块拥有任务，项目管理器直关 workspace 也受关闭守卫约束：任务超时会拒绝本次关闭而保留 document；非活动文档保存被明确拒绝，STL/BREP 在最后取消检查后才提交；但 worker 仍持有并直接修改裸 `LcncDocument*`，STEP/IGES 仍非 detached 原子提交。 |
| 5 | MainWindow controller 下沉与总门禁 | 进行中 | 工程树读取已开始使用 CAD/CAM projection contract，Workspace/ViewState/TaskPanel/ProjectExplorer controller 已承担部分职责。但 `AppContext`、MainWindow、commands、module UI 和新 `CadTaskPanelController` 仍直接依赖具体 Module；预览与执行参数映射还有重复。下一阶段以切断 concrete Module 耦合为完成条件，不再设置硬性行数目标。 |

## 当前约束

- 模块停止必须先请求其拥有任务的协作取消，并在有限预算内等待；超时后由
  调用方保留相关运行时对象并进入既有安全错误流程。
- 本阶段不改变 `.lcnc v4`、CAM v4、workflow 当前 schema 或 Process settings
  schema v2，也不恢复历史格式兼容。
- `SimulatorCMHP` 仅作为 ACS SDK 模拟器自动化证据，不等同于物理设备验证。
- 2026-08-03 文件级审阅的缺陷、证据和优先级以 `AUDIT.md` 与 `todo.md` 为准；此前版本交付文档只记录当时验证，不覆盖后续审阅发现。
