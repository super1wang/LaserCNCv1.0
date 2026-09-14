# LaserCNC 待办

更新日期：2026-08-24

来源：[AUDIT.md](AUDIT.md)

本文件只维护本次审计整改后仍未完成的工作。P0-1～P0-3 的历史整改见 [v1.5.9 交付记录](docs/versions/v1.5.9-2026-08-21.md)，当前连续碰撞闭环见 [v1.6.0 交付记录](docs/versions/v1.6.0-2026-08-24.md)。

## 下一版本 P0：完整机台碰撞闭环

- [x] 使用 `.lmsi` supercover 与 Coal 保守自适应细分覆盖 Rapid、LeadIn、Cutting、Traverse 全部运动边。
- [x] 引入可执行边证书和固定/连续点动许可证，CAM 绑定路点、阶段、包键和环境代际，Process 只消费完整证书。
- [x] 完成机台包/Job Overlay 缓存失效、构建中失败关闭、真实机台性能门限和热区反馈续建。
- [x] 使用 `精简ac转台.stp` 与 `半球.stp` 建立安全包、连续证书、Coal 回退和 Process 契约回归；详细状态见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)。
- [x] 完成工作区审查补强：构建中保持全局碰撞意图并失败关闭；混合实体/开放面使用独立封闭实体包含 BVH；同步多轴拒绝混用绝对/相对模式。
- [ ] 增加碰撞后自动绕障/重规划和关键 Coal 碰撞对解释；当前系统会安全阻断，不会自动改写 CAM 权威路径。
- [ ] 完成夹具正式数据契约、扩大 exact 反向审计，并完成 GUI/SDK/无激光低速实体机验收。

软件连续证书链已经成立，但在夹具、扩大审计和实体机门禁完成前，不得声明整机生产安全。真实加工继续对 Pending、Indeterminate、Collision、BoundaryUnknown、过期或 `complete=false` 快照失败关闭。

## P1：后续结构优化

- [ ] 将 `MainWindow` 中跨模块 UI 接线逐步下沉为 workspace controller/presenter。App 作为组合根可以持有模块实例，但业务调用不得重新越过 facade/contract。
- [ ] 将 Process 的 prepare/start/pause/resume/stop 编排继续下沉到 `ProcessRunCoordinator`，缩减 `ProcessModule` 状态面。
- [ ] 继续拆分 `cam_module_toolpath.cpp`、`cam_module_collision.cpp`、`cam_module_pipeline.cpp`、ACS/GTN 适配器和 `DialogOptions`；每次拆分保持事务、revision 和取消语义不变。
- [ ] 补双工作区反复打开/关闭、AIS 投影隔离、CAD 导入取消与关闭竞争、Simulation 开停循环测试。
- [ ] 为不可由软件中断的供应商 SDK 调用建立进程外看门狗或厂商级硬超时策略；当前代码 deadline 只能约束可返回的轮询调用。

## P1：发布与实体机证据

- [ ] 完成 GUI 验收：工程生命周期、三域显示、国际化、CAD 导入、CAM 预览、Simulation 和 Process 状态转换。
- [ ] 对真实 ACS/GTN、激光器和 IO 做低速验证：首段、轮廓间空程、急停/断连、重启恢复、轴限位和激光安全输出。
- [ ] 完成 8 小时资源趋势与重复连接/断开压力测试，记录线程、句柄和私有字节增长；Application Verifier 仅在明确测试窗口使用。
- [ ] 真实激光协议测试增加可替换传输层和硬件回环；当前纯协议测试不等同于串口/设备联调。

## P2：持续工程卫生

- [ ] 将 `.clang-format` 纳入增量格式门禁，逐批清理未触及的历史超长函数和混合缩进，避免全仓机械格式化掩盖功能差异。
- [ ] 继续提取 CMake target/runtime/ASan 部署 helper，降低根 `CMakeLists.txt` 的维护密度。
- [ ] 增加 catch 日志、Qt 翻译 catalog 和修改文件格式检查；现有架构门禁已覆盖分层、Process OCC、具体跨模块依赖、目录归一、公共头命名空间污染和孤儿源码。
- [ ] 持续同步可见 UI/日志文本、邻接 `中文翻译：...` 注释和 `translations/lasercnc_zh_CN.ts`。

## 每次提交的最低门禁

- [ ] `git diff --check`
- [ ] 按 [BUILD.md](BUILD.md) 使用对应 preset 构建，且 Ninja 与 VS 生成树不混用、不并发。
- [ ] `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
- [ ] `scripts/check_architecture.ps1 -Root .`
- [ ] 所有权、异步、OCC 或设备 SDK 改动运行对应 ASan/variant 测试。
- [ ] 自动化、SDK 仿真、GUI 和物理机证据分别记录，不互相替代。
