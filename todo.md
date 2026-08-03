# LaserCNC 剩余工作

更新日期：2026-08-03
当前审计：[AUDIT.md](AUDIT.md)

本文件只保留未完成事项；已经完成的工作与验证证据不再重复维护。

## P0：安全与发布门禁

- [ ] 扩展 `SimulatorCMHP` SDK 集成测试，使单个测试进程内可稳定重复完整的连接、建轴、使能、轮询、回零、短刀路、暂停、Stop、急停、安全输出复位和断开循环；当前只完成真实 SDK/`Simulator.prg` 初始化、100 次设备线程会话命令及同线程断开/销毁，供应商 RPC 释放窗口仍会阻止稳定的 100 次进程级重连。
- [ ] ACS/GTN 真机各执行不少于 100 次连接/断开和加工停止循环，验证激光、红光、吹气及运动输出安全复位。
- [ ] 逐项确认供应商调用的 timeout/abort 能力；对不可中断调用定义停止发新命令、对象保活和人工恢复流程。
- [ ] 在禁用第三方输入法注入的受控环境执行 ASan GUI、Application Verifier/页堆和 8 小时资源趋势；ASan 构建及全部 CTest 已通过，但不替代这些交互与长稳证据。

## P1：结构收口

- [ ] 继续将 `process_module.cpp`（当前约 2,286 行）收敛至 900 行入口门限；`ProcessConnectionService`、`ProcessPreflightService`、`ProcessStatusService`、`ProcessRunCoordinator` 和 `ProcessWorkflowService` 已落地，workflow 当前格式文档/读写/变更通知及流程树文件操作已脱离模块入口。剩余运行状态组合、手动控制 facade 与 UI 事件出口仍需下沉，且必须保持既有关闭顺序。
- [ ] 完成 `cam_module.cpp`（当前约 6,560 行）的职责下沉；`ToolpathGenerationService` 已实现并覆盖 stale-result 拒绝，`CamDisplayProjectionService` 已接管加工面 AIS 投影；machining-face pipeline、machine calibration 和其余刀路/机台投影仍待独立 service。
- [ ] 继续将 CAD 入口收敛为生命周期与 facade：`CadDocumentIoService` 已接管新建、保存、关闭、STEP 导出、STEP/IGES/STL/BREP 解析及显示网格准备；工程包导入仍需迁入服务，且必须保持成功前不替换活动工程；草图/特征流程和选择刷新仍待 `CadModelingController`、`CadSelectionController` 下沉。
- [ ] 收敛 real-laser 配置中旧厂商协议适配器的项目 `/W4` 告警，使该配置也能启用 `/WX`；ACS+GTN 质量预设已达到 `/W4 /WX`。

## P1：自动化回归

- [ ] 增加工作流专用线程、PureSimulation/SDK 线程亲和、轮询关闭和 Process 并发关闭测试，并分别重复运行 100 次；队列 lifecycle 和运行状态迁移已有独立回归。
- [ ] 补齐 CAM 实际异步任务入口的全局生成/当前轮廓重算集成测试；当前 service 级测试已覆盖成功、取消、工件/加工面/参数/轮廓 revision 变化及陈旧结果拒绝。
- [ ] 测量大模型轮廓提取、面分类、IK 和 GUI 提交阶段的取消延迟与最长卡顿。

## 每次提交最小检查

- [ ] `git diff --check`
- [ ] `cmake --preset acs-gtn`
- [ ] `cmake --build --preset acs-gtn-debug --parallel 16`
- [ ] `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
- [ ] `scripts/check_architecture.ps1 -Root .`
- [ ] `scripts/run_quality_gates.ps1`
- [ ] 涉及真实硬件时执行输出安全、Stop 优先级和断开检查
