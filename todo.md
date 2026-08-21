# LaserCNC 待办

更新日期：2026-08-21

来源：[AUDIT.md](AUDIT.md)

本文件只维护本次审计整改后仍未完成的工作。P0-1～P0-3、已收口的 P1/P2 项及其验证记录见 [v1.5.9 交付记录](docs/versions/v1.5.9-2026-08-21.md)。

## 下一版本 P0：完整机台碰撞闭环

- [ ] 完成连续扫掠或可证明保守的自适应细分，覆盖首段、轮廓内、轮廓间和回退路径。
- [ ] 引入可执行边证书，保证 CAM 验证路点、阶段和 AC/BC 插补顺序与 Process 实际消费完全一致。
- [ ] 完成碰撞后重规划、关键碰撞对解释、缓存失效和完整机台性能门限。
- [ ] 使用真实 `model/` STEP 建立 C0～C3、首段重规划和连续段回归；详细计划见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)。

在以上门禁完成前，真实加工继续对 Pending、Indeterminate、Collision、过期或 `complete=false` 快照失败关闭，不得把离散点验证表述为连续路径安全证明。

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
