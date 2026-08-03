# LaserCNC 模块入口瘦身执行进度

更新日期：2026-08-03

本文件记录已批准的模块入口瘦身计划的阶段性进度。完成标记只代表源码和
列出的自动化验证；不会替代 ACS、GTN 或激光物理硬件验证。

| 阶段 | 范围 | 状态 | 证据 / 后续 |
| --- | --- | --- | --- |
| 1 | 共用模块任务生命周期骨架 | 已完成 | `ModuleTaskScope` 统一 CAD、CAM、Process 的任务跟踪、协作取消和有限等待；见 v1.3.2。 |
| 2 | Process typed executor 与入口下沉 | 进行中 | `ProcessConnectionService` 接管连接/断开队列事务，`ProcessPreflightService` 接管 generation 化预检，`ProcessStatusService` 接管 150 ms 控制器、2 s 外设与安全监控启停调度，`ProcessWorkflowService` 独占当前 schema 流程文档、文件读写及变更出口；流程树与 `MainWindow` 通过 workflow 契约接线。普通切割 sink 生命周期和轮廓前二次健康门禁已收回 typed runtime。runtime 外原始设备指针/锁 API 已清零并由架构扫描防回退。仍需继续瘦身运行状态与手动控制 UI facade 组合。 |
| 3 | CAM service 下沉 | 未开始 | 拆分加工面 pipeline、生成、标定与显示投影。 |
| 4 | CAD service/controller 下沉 | 进行中 | `CadDocumentIoService` 已接管新建、保存、关闭、STEP 导出及 STL 导入后台任务；STEP/IGES/BREP 与工程包导入事务、建模和选择控制器仍待迁移。 |
| 5 | MainWindow controller 下沉与总门禁 | 未开始 | 收敛工作区、工程树、视图状态及 CAD UI 控制器，并执行矩阵验证。 |

## 当前约束

- 模块停止必须先请求其拥有任务的协作取消，并在有限预算内等待；超时后由
  调用方保留相关运行时对象并进入既有安全错误流程。
- 本阶段不改变 `.lcnc v4`、CAM v4、workflow 当前 schema 或 Process settings
  schema v2，也不恢复历史格式兼容。
- `SimulatorCMHP` 仅作为 ACS SDK 模拟器自动化证据，不等同于物理设备验证。
