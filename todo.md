# LaserCNC 剩余工作

更新日期：2026-07-30
当前审计：[AUDIT.md](AUDIT.md)

本文件只保留未完成事项；已经完成的工作与验证证据不再重复维护。

## P0：安全与发布门禁

- [ ] 对 ACS、GTN、SimulatorCMHP 建立连接、轮询、回零、加工、暂停、Stop、急停、断开和退出的可重复测试。
- [ ] ACS/GTN 真机各执行不少于 100 次连接/断开和加工停止循环，验证激光、红光、吹气及运动输出安全复位。
- [ ] 逐项确认供应商调用的 timeout/abort 能力；对不可中断调用定义停止发新命令、对象保活和人工恢复流程。
- [ ] 验证 DeviceCommandQueue 的 `Stop > Workflow > Interactive > Normal > Polling`、同级 FIFO、轮询合并和 completion 完成语义。
- [ ] 在禁用第三方输入法注入的环境执行 ASan GUI、Application Verifier/页堆和 8 小时资源趋势。
- [ ] 执行 all-off、ACS-only、GTN-only、ACS+GTN、real-laser 和 ASan 干净构建矩阵。

## P1：结构收口

- [ ] 将 `process_module.cpp` 的连接会话、加工预检、轮询投影和运行状态下沉为独立服务；迁移时保持设备租约与关闭顺序。
- [ ] 让 DeviceCommandQueue 逐步成为唯一 SDK 执行入口；在所有直接路径迁移并完成真机验证前保留 `ProcessDeviceCoordinator`。
- [ ] 拆分 `cam_module.cpp` 的 machining-face pipeline、toolpath generation、machine calibration 和 display projection。
- [ ] 拆分 `main_window.cpp` 的 workspace presenter、project explorer controller 和 view-state controller。
- [ ] 继续将 `cad_module.cpp` 的导入/导出、草图、特征和选择刷新下沉到现有 services。
- [ ] 为项目代码建立分 target 的 `/W4` 告警基线，供应商头继续使用 external warning policy。
- [ ] 统一 `core/algorithms/cad` 的 OCC 异常契约：纯算法抛出、调用方记录，或明确采用无异常结果；不得混用。
- [ ] 分批迁移 45 个 Process legacy PascalCase 文件/目录及其旧式 API；每批必须保持硬件开关构建通过。

## P1：自动化回归

- [ ] 扩展设备队列测试：全部优先级、FIFO、coalesce、shutdown、Superseded/Cancelled/Shutdown 结果。
- [ ] 增加工作流线程、PureSimulation 线程亲和与 Process 并发关闭测试，并重复运行 100 次。
- [ ] 增加 CAM 全局生成/当前轮廓重算的成功、取消、工件替换、参数变化和陈旧结果测试。
- [ ] 增加 CamDataManager、轮廓排序、Process 状态机、工具快照并发读取和工程保存中断测试。
- [ ] 测量大模型轮廓提取、面分类、IK 和 GUI 提交阶段的取消延迟与最长卡顿。

## P2：兼容与维护

- [ ] 为 CAM JSON、旧 workflow tree、旧工具字段分别确定 schema、fixture、离线升级路径和删除版本。
- [ ] 使用真实 v1/v2/v3 工程样本验证离线升级、工具快照、跨机恢复和机台指纹提示。
- [ ] 将 CI/本地门禁统一为 preset 配置、Debug 构建、CTest、架构扫描、空白检查和启动冒烟。

## 每次提交最小检查

- [ ] `git diff --check`
- [ ] `cmake --preset acs-gtn`
- [ ] `cmake --build --preset acs-gtn-debug --parallel 16`
- [ ] `ctest --test-dir build --build-config Debug --output-on-failure`
- [ ] `scripts/check_architecture.ps1 -Root .`
- [ ] 涉及真实硬件时执行输出安全、Stop 优先级和断开检查
