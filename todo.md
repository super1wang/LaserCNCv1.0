# LaserCNC 剩余工作

更新日期：2026-08-19
当前审计：[AUDIT.md](AUDIT.md)

本文件只保留未完成事项；已经完成的工作与验证证据不再重复维护。

## P0：安全与发布门禁

> 碰撞检测、安全域缓存和首段/空程规划的完整分阶段事项已拆分到 `docs/collision_detection_todo.md`；本节仅保留发布门禁摘要。

- [ ] 优化完整机台模式的碰撞校验性能。非机台模式人工测试基本通过，但选择完整运动轴和多个被动轴时仍有严重耗时；需要按轴刚性组统计 BVH 候选、网格拒绝率和精确复检次数，并增加代表性 AC/BC/摆头自动性能夹具。
- [ ] 完成机台模型加载、碰撞源选择和离线仿真的 GUI 与代表性样件验证。当前已具备 detached 机台解析、主动/被动碰撞源、CAM 完整路径校验、独立 OCC 沙箱和碰撞时间线，但这些自动化/静态证据不能作为真机安全证据。
- [ ] 将机台环境校验从离散姿态检查升级为连续段扫掠或保守细分，并在碰撞后生成新的抬高/侧移候选路径；当前外表面网格 BVH 和候选精确距离检查仍不能证明采样点之间安全，也不会主动绕开机台障碍。
- [ ] 完成碰撞对配置与诊断：验证 Z/摆头轴子树中的未直接归轴子件，支持持久化的主动/被动对覆盖和显式忽略对，并输出失败部件、姿态和最小间隙。当前碰撞源由运动学挂接关系推导，尚无成对覆盖配置。
- [ ] 完成无激光、低速条件下的 AC/BC 转台和摆头验证：机台模型显示/隐藏结果一致，真实切割头与轻量刀嘴基准一致，夹具/卡盘随轴链正确变换，碰撞计划硬阻止加工，并记录安装误差和各控制器实际同步插补行为。
- [ ] 扩展 `SimulatorCMHP` SDK 集成测试，使单个测试进程内可稳定重复完整的连接、建轴、使能、轮询、回零、短刀路、暂停、Stop、急停、安全输出复位和断开循环；当前只完成真实 SDK/`Simulator.prg` 初始化、100 次设备线程会话命令及同线程断开/销毁，供应商 RPC 释放窗口仍会阻止稳定的 100 次进程级重连。
- [ ] ACS/GTN 真机各执行不少于 100 次连接/断开和加工停止循环，验证激光、红光、吹气及运动输出安全复位。
- [ ] 逐项确认供应商调用的 timeout/abort 能力；对不可中断调用定义停止发新命令、对象保活和人工恢复流程。
- [ ] 在禁用第三方输入法注入的受控环境执行 ASan GUI、Application Verifier/页堆和 8 小时资源趋势；ASan 构建及全部 CTest 已通过，但不替代这些交互与长稳证据。

## P1：结构收口

- [ ] 完成 Process 运行安全事务下沉，不再以行数为目标：让 `ProcessRunCoordinator` 真正拥有 run/preflight/Stop/Emergency/Recovery completion 和合法状态迁移；`ProcessModule` 只转发 facade 与信号。motion sink 生命周期仍应继续隐藏到 typed executor 私有 contract。
- [ ] 完成 CAM 高收益下沉：`MachiningFacePipelineService` 不再暴露 mutable `entries()`，下游只消费 immutable snapshot；已为等价自动面重算复用稳定 ID，`CamDisplayProjectionService` 已按 document 记录 owning AIS context。仍需双工作区 offscreen 回归，并继续迁移机台、轴导引、刀路和 travel path 投影；machine calibration 独立成 service。
- [ ] 完成 CAD 文档事务：异步 reader 只生成 detached 结果，generation 匹配后在文档所属线程一次提交；`CadModule` 关闭及项目管理器关闭 workspace 均先取消并等待模块任务，超时会拒绝关闭并保留 document，保存已拒绝非活动文档；仍需文档级任务归属与显式 workspace/document identity 的保存/导出 API。随后下沉 `CadModelingController`、`CadSelectionController`。
- [ ] 继续收敛 MainWindow 和跨模块调用：`AppContext`、commands、module UI、`DialogOptions`、`CadTaskPanelController` 改用 facade/service/controller contract，迁移后删除具体 Module 的公共 service 注册。合并 MainWindow/controller 重复的 primitive/feature/transform 参数映射，避免预览与执行 schema 漂移。
- [ ] 收敛 real-laser 配置中旧厂商协议适配器的项目 `/W4` 告警，使该配置也能启用 `/WX`；ACS+GTN 质量预设已达到 `/W4 /WX`。

## P1：自动化回归

- [ ] 扩展 Stop 安全事务和 Emergency 锁存回归：已覆盖 E-stop 后交互数字输出拒绝；仍需覆盖 workflow/非 workflow 的排队失败、执行失败、超时、轴使能拒绝与恢复前完整健康复核；`SimulatorCMHP` 每轮验证所有安全输出保持关闭。
- [ ] 增加 CAD 文档 IO 集成测试：已有/新建工作区的成功、失败、取消、关闭并发、非活动文档保存和失败不提交；用线程检查证明 worker 不直接修改活动 XCAF 文档。
- [ ] 增加双工作区 AIS 投影回归及各并发关闭场景循环 100 次；Process status 快速 stop/start 旧 completion、DeviceCommandQueue completion 抛异常、CAM 等价自动面稳定 ID 已有独立回归。
- [ ] 增加工作流专用线程、PureSimulation/SDK 线程亲和、轮询关闭和 Process 并发关闭测试，并分别重复运行 100 次；队列 lifecycle 和运行状态迁移已有独立回归。
- [ ] 补齐 CAM 实际异步任务入口的全局生成/当前轮廓重算集成测试；当前 service 级测试已覆盖成功、取消、工件/加工面/参数/轮廓 revision 变化及陈旧结果拒绝。
- [ ] 增加完整机台校验与离线仿真回归：覆盖碰撞源空选择持久化、校验任务被新顺序替换、工件安装变换、异步退出释放，以及 AC/BC/摆头代表性模型的碰撞时间线结果。
- [ ] 测量大模型轮廓提取、面分类、IK 和 GUI 提交阶段的取消延迟与最长卡顿。

## P2：规范与门禁

- [ ] 为 `CadDocumentIoService`、`ProcessWorkflowExecutor`、`CamModule` 等仍未记录的 catch 补 `LCNC_ERR`；架构检查增加“catch 必须有错误日志”的可维护规则。
- [ ] 将 `ProcessManualMotionService`、`ProcessInteractiveIoService` 的新增可见文本加入 `translations/lasercnc_zh_CN.ts`，补齐相邻中文翻译注释，并增加新增 `tr()` catalog 检查。
- [ ] 在上述结构债务清理后继续扩展架构扫描：禁止 app/commands/module UI 获取具体 Module，并禁止 CAD worker 直接写活动文档。Process public runtime/sink contract 的 `ProcessModule*` 门禁已落地。

## 每次提交最小检查

- [ ] `git diff --check`
- [ ] `cmake --preset acs-gtn`
- [ ] `cmake --build --preset acs-gtn-debug --parallel 16`
- [ ] `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
- [ ] `scripts/check_architecture.ps1 -Root .`
- [ ] `scripts/run_quality_gates.ps1`
- [ ] 涉及真实硬件时执行输出安全、Stop 优先级和断开检查
