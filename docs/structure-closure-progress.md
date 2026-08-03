# LaserCNC 模块入口瘦身执行进度

更新日期：2026-08-03

本文件记录已批准的模块入口瘦身计划的阶段性进度。完成标记只代表源码和
列出的自动化验证；不会替代 ACS、GTN 或激光物理硬件验证。

| 阶段 | 范围 | 状态 | 证据 / 后续 |
| --- | --- | --- | --- |
| 1 | 共用模块任务生命周期骨架 | 已完成 | `ModuleTaskScope` 统一 CAD、CAM、Process 的任务跟踪、协作取消和有限等待；见 v1.3.2。 |
| 2 | Process typed executor 与入口下沉 | 未开始 | 保持现有安全关闭顺序，迁移 runtime 外的 SDK 指针与设备锁访问。 |
| 3 | CAM service 下沉 | 未开始 | 拆分加工面 pipeline、生成、标定与显示投影。 |
| 4 | CAD service/controller 下沉 | 未开始 | 拆分文档 IO、建模和选择控制器。 |
| 5 | MainWindow controller 下沉与总门禁 | 未开始 | 收敛工作区、工程树、视图状态及 CAD UI 控制器，并执行矩阵验证。 |

## 当前约束

- 模块停止必须先请求其拥有任务的协作取消，并在有限预算内等待；超时后由
  调用方保留相关运行时对象并进入既有安全错误流程。
- 本阶段不改变 `.lcnc v4`、CAM v4、workflow 当前 schema 或 Process settings
  schema v2，也不恢复历史格式兼容。
- `SimulatorCMHP` 仅作为 ACS SDK 模拟器自动化证据，不等同于物理设备验证。
