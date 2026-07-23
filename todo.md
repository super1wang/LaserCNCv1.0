# LaserCNC 剩余工作计划

更新日期：2026-07-23
基线提交：`a1090c1 refactor: unify asynchronous task execution`

本文件只跟踪尚未完成的事项。已完成范围、当前任务模型和验证证据见
`DELIVERY.md` 与 `ARCHITECTURE.md`。

## 当前基线

- [x] `TaskManager` 统一描述通用后台任务，支持优先级、状态、进度、取消和有界等待。
- [x] `DeviceCommandQueue` 提供 Stop > Workflow > Interactive > Normal > Polling
  的专用设备线程与轮询合并。
- [x] 工作流拥有独立单线程池，并以 Workflow 优先级访问设备。
- [x] 手动运动、IO、设置下发、加工预检、回零、轮询和工作流设备调用不在 GUI
  线程执行。
- [x] 全局刀路生成与当前轮廓重算由通用线程池执行，支持进度、取消和陈旧结果丢弃。
- [x] 连接后启动轮询，断开或模块停止后停止轮询并收口线程生命周期。
- [x] ACS+GTN Debug 构建、5 项 CTest、架构检查和 8 秒启动冒烟通过。

## P0：集成测试与发布安全门禁

### 1. 设备并发与优先级验证

- [ ] 为 ACS、GTN、SimulatorCMHP 建立可重复的连接、轮询、回零、加工、暂停、
  Stop、急停、断开和退出序列测试。
- [ ] 记录设备队列执行序列，证明待执行命令严格满足
  Stop > Workflow > Interactive > Normal > Polling，同优先级保持 FIFO。
- [ ] 验证轮询高频触发时按 key 合并，不产生队列积压，不降低工作流轮廓切换效率。
- [ ] 验证连续点动 Stop、普通 Stop 和急停在当前不可中断供应商调用返回后立即执行。

验收：仿真器压力测试自动化通过；ACS/GTN 真机各完成不少于 100 次连接/断开和
完整加工停止循环，无竞态、残留输出、残留线程或句柄增长。

### 2. 不可中断 SDK 调用与关机故障注入

- [ ] 为连接、断开、回零、缓冲启动和状态查询逐项确认供应商超时/abort 能力。
- [ ] 对不能强制中断的调用定义最大等待时间、停止发新命令、保持对象存活和人工
  恢复流程。
- [ ] 注入工作流执行中、设备命令执行中、TaskManager 任务执行中关闭模块/应用的场景。
- [ ] 覆盖 CAD/CAM/Process `init/start` 失败和 rollback，验证 `stop()` 幂等。

验收：超时路径无 UAF、双重释放或死锁；活动 SDK 调用未退出时不销毁其 Service/
控制器，日志明确记录保留原因与恢复建议。

### 3. 线程模型自动化回归

- [ ] 扩展 `lcnc_device_command_queue_test`：覆盖所有优先级、FIFO、轮询合并、
  shutdown 清队列和等待者完成语义。
- [ ] 增加工作流线程测试：确认插件不在 GUI/通用线程池执行，暂停和停止只在
  checkpoint 生效，Stop 不被轮询饿死。
- [ ] 增加 PureSimulation 测试：所有 `QTimer` 操作发生在 ticker 所属线程，
  工作流暂停/恢复/停止不会触发 Qt 跨线程警告。
- [ ] 增加 Process 关机测试：工作流、设备队列和 TaskManager 同时活动时按既定
  顺序退出。

验收：上述测试加入 CTest，重复运行 100 次无偶发失败。

### 4. 内存与长稳发布门禁

- [ ] 在禁用第三方输入法注入的环境执行 ASan GUI 启动、工程开关、CAM 生成、
  PureSimulation 和退出。
- [ ] 执行 Application Verifier/页堆，覆盖 Qt、OCC 与供应商 DLL 场景。
- [ ] 运行 8 小时工程/CAM/仿真循环，采集私有字节、工作集、句柄、线程、
  QObject、OCC Handle、future 和供应商句柄趋势。

验收：ASan/Application Verifier 无项目错误；稳定窗口内资源无持续单调增长。

## P1：响应性与正确性回归

### 5. CAM 异步任务

- [ ] 为全局生成和当前轮廓重算增加自动化测试：成功提交、用户取消、工件替换、
  参数修改、源形状修改和刀路版本变化均得到正确结果。
- [ ] 测量单个超大工件在 `extractContours`、面分类和 IK 阶段的取消延迟；必要时
  向算法内部增加更细粒度 checkpoint/进度回调。
- [ ] 测量 GUI 提交阶段的 XCAF 写入、轮廓 relink、树同步和渲染刷新；若单次超过
  交互阈值，拆分为批量提交或分帧刷新。
- [ ] 通过实际项目统计其余 CAD/CAM command 耗时；仅将超过阈值的一次性任务迁入
  TaskManager，短命令继续同步执行。

验收：取消延迟和 GUI 最长卡顿有可重复数据；典型/最大项目均无未解释的 GUI 阻塞。

### 6. Process 快照与加工顺序

- [ ] 为 CAM OCC-free 快照缓存增加并发读取测试，覆盖生成、清空、图层变更和当前
  轮廓重算后的 revision/内容一致性。
- [ ] 为 `ProcessToolpathService` 和切割计划增加顺序、图层启用、工具绑定、连续性
  与陈旧 revision 回归。
- [ ] 审核 `ToolFactory`/项目工具快照在设置重载与工作流同时发生时的同步边界，
  加工运行中禁止会改变当前执行快照的设置操作。

验收：工作流线程不访问 OCC/XCAF/GuiDocument；加工期间使用不可变快照，修改只影响
下一次启动。

### 7. 设备队列完成与取消语义

- [ ] 为被 coalesce 替换或 shutdown 丢弃的带 completion 命令定义明确结果
  （Superseded/Cancelled/Shutdown），确保任何等待者都能结束。
- [ ] 将设备命令错误类型从自由文本扩展为稳定错误码，并保留供应商诊断文本。
- [ ] 评估断开设备的一次性 TaskManager 策略是否继续保留；若迁入设备队列，必须
  同时保留阶段进度、取消检查和 SDK 超时后的对象保留策略。

验收：无 completion 丢失、永久等待或仅靠日志判断状态的调用路径。

## P2：结构与维护

### 8. Process facade 收口

- [ ] 将连接/断开、手动运动、加工预检、轮询投影和运行状态从
  `process_module.cpp` 下沉为独立服务，目标文件小于 800 行。
- [ ] 继续替换 `Service`、`MotionControl`、`LaserDevice`、`ToolFactory` 的
  PascalCase/legacy 接口，公共 facade 不暴露设备实现指针。
- [ ] 移除剩余 `MessageModule` UI 队列，通知统一为主线程事件/信号并写入
  `lcnc::Logger`。
- [ ] 将 BDAQ 与真实激光适配器进一步拆为 option-gated 私有 target，并补齐
  all-off、ACS、GTN、ACS+GTN、real-laser 构建矩阵。

### 9. 领域测试与历史兼容

- [ ] 增加 CamDataManager、轮廓排序、Process 状态机和工程保存中断测试。
- [ ] 使用真实 v1/v2/v3 工程样本验证 `lcnc_project_upgrade`、`tools.toml`、
  跨机恢复和机台指纹提示。
- [ ] 明确 CAM JSON、旧 workflow tree 和旧字段读取的删除版本；旧格式只允许通过
  离线升级器进入当前 schema。

### 10. CI 与交付门禁

- [ ] 在 CI/本地一键脚本执行 `git diff --check`、ACS+GTN Debug 构建、CTest、
  架构扫描和启动冒烟。
- [ ] 对 all-off、ACS、GTN、ACS+GTN 和 ASan preset 建立干净配置验证，避免同一
  `build/` 树切换 preset 后遗留错误缓存。
- [ ] 发布清单明确区分“构建/自动化通过”“仿真器验证通过”“真机验证通过”和
  “生产发布通过”，不得用静态审计替代现场安全验收。

## 每次提交最小检查

- [ ] `git diff --check`
- [ ] `cmake --preset acs-gtn`
- [ ] `cmake --build --preset acs-gtn-debug --parallel 16`
- [ ] `ctest --test-dir build --build-config Debug --output-on-failure`
- [ ] `scripts/check_architecture.ps1 -Root .`
- [ ] Process OCC 扫描为 0，禁用文档 API 无新增调用
- [ ] 涉及运行时生命周期时执行启动/退出冒烟
- [ ] 涉及真实硬件时执行安全输出、Stop 优先级和断开检查
