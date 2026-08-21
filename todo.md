# LaserCNC 待办

更新日期：2026-08-21

来源：[AUDIT.md](AUDIT.md)

本文件只维护尚未完成的工作。已完成事项由源码、测试和 `docs/versions/` 中的交付记录承载，不在这里重复累积。

## P0：真实设备与数据安全

- [ ] 修复 `UltronLaserDevice::InitLaser()`：消除释放后使用、未初始化变量、局部变量遮蔽和缺失返回值；将协议解析改成有边界、可测试的纯函数。
- [ ] 为 ACS `StopBuffer()`、GTN 运动完成/停止等待和规划停止循环增加超时、取消、退避与确定的失败状态；任何供应商 SDK 等待都不得无限占用设备租约。
- [ ] 统一设备队列的全局安全语义：Stop/关闭必须能够打断或超越状态轮询、外设轮询和普通命令；在完成前不得宣称 Stop 具有进程级抢占能力。
- [ ] 增加真实激光假传输层测试，以及 ACS/GTN “设备永不完成”故障注入测试，验证有限时间退出和安全输出关闭。
- [ ] CAD STEP/IGES/STL/BREP 导入改为后台构造独立形状或临时文档，再由文档线程按 workspace generation 原子提交；worker 不得直接修改活动 XCAF 文档。
- [ ] CAD 导入、网格和大模型任务改为按 document/workspace 记录 task-id；关闭工程、切换工程和模块停止时取消并有限等待。
- [ ] 完成全机碰撞的连续扫掠或保守自适应细分、路径重规划和碰撞对可解释输出；详细分阶段工作见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)。
- [ ] 对真实机台执行低速验证：首段、轮廓间空程、急停/断连、重启恢复、轴限位和激光安全输出。自动化、SDK 仿真和 GUI 检查不能替代此项。

## P1：架构收敛

- [ ] 让 app/commands/UI 只依赖 `ICadFacade`、`ICamFacade`、`IProcessFacade` 与专用查询/命令服务；移除对具体 `*Module` 的直接查找。
- [ ] 为 Simulation 提供 CAM 场景与机床配置的窄只读 provider，移除对 `CamModule` 具体类型的依赖。
- [ ] 将 Process 的 prepare/start/pause/resume/stop 状态机抽成 `ProcessRunCoordinator`；继续拆分 `ProcessModule`、ACS、GTN 和设置对话框热点文件。
- [ ] 明确三条设备命令队列与 `ProcessDeviceCoordinator` 的边界，或合并成能保证全局优先级的单一调度器；补充所有权、关闭顺序和阻塞预算文档。
- [ ] 替换 ServiceRegistry 中指向模块自身的 no-op-deleter `shared_ptr` 注册方式，使用非拥有引用或生命周期明确的服务对象。
- [ ] 收紧 `MachiningFacePipelineService`：不向外暴露可变容器引用，CamModule 不长期保存内部 entry 引用。
- [ ] 继续将 `cam_module_toolpath.cpp`、`cam_module_collision.cpp`、`cam_module_pipeline.cpp` 拆为服务/控制器，并保持 CamModule 只做生命周期和协调。
- [ ] 将 CAD 文件操作和特征创建从 `CadModule` 拆到 document-scoped service；继续拆分 `MainWindow` 和 `DialogOptions`。
- [ ] 建立唯一版本源，由 CMake、`QApplication`、About、包 manifest 和交付文档共同生成或读取；消除 `1.2.1`、`1.0.0` 与交付编号漂移。
- [ ] 将 CMake 模块依赖从宽泛 `PUBLIC` 收紧为实际需要的 `PRIVATE`/接口依赖，并用 gate 防止传播依赖掩盖 include 越层。

## P1：回归与发布证据

- [ ] 增加 CAD 导入取消、关闭工程竞争、失败提交不污染文档的自动化测试。
- [ ] 增加双工程反复打开/关闭、CAM 重算、模拟开始/停止和 Process 连接/断开循环测试，观测线程、句柄和私有字节增长。
- [ ] 增加全机连续扫掠、首段重规划、关键碰撞对和缓存失效的真实 `model/` STEP 回归，并记录耗时基线。
- [ ] 对所有权、异步、OCC 或供应商 SDK 改动运行 ASan preset；Application Verifier 仅在明确的交互测试窗口使用。
- [ ] 建立 GUI 验收表，覆盖项目生命周期、三域显示、国际化、CAM 预览、Simulation 和 Process 状态转换。

## P2：规范与工程卫生

- [ ] 补齐 Process 设备公共头的 `#pragma once`，移除公共头中的 `using namespace`。
- [ ] 引入可执行的格式检查；拆分超长函数和压缩单行，清理混合 Tab/空格与极端长行。
- [ ] 合并 `src/modules/process/setting/` 与 `settings/`，隔离或包装供应商 `bdaqctrl.h`，避免其污染自有代码指标。
- [ ] 用 `<numbers>` 或统一常量替换多处 `M_PI` 宏定义。
- [ ] 抽取 CMake 测试目标、运行库部署和 variant 配置 helper，降低 1300 行根 `CMakeLists.txt` 的重复。
- [ ] 扩展 `architecture_checks`：禁止 app/commands 新增具体模块依赖，检查 CAD worker 修改活动文档、无界 SDK 等待、公共头 `using namespace` 和缺失 include guard。
- [ ] 持续完成可见 UI/日志文本的英文源文本、邻接中文注释和 `translations/lasercnc_zh_CN.ts` 同步。

## 每次提交的最低门禁

- [ ] `git diff --check`
- [ ] 按 [BUILD.md](BUILD.md) 使用对应 preset 构建，且 Ninja 与 VS 生成树不混用、不并发。
- [ ] `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
- [ ] 分层、旧 API、Process OCC-free 和 architecture_checks 通过。
- [ ] 对涉及域完成对应 smoke；真实设备结论只来自真实设备验证。
