# LaserCNC 当前交付状态

复核日期：2026-07-30

## 结论

当前工作区已完成一轮全源码架构审计和可证明死代码清理，ACS+GTN Debug 构建、7 项 CTest、架构门禁与空白检查通过，可作为集成测试候选。

当前不是生产发布版本。真机安全、供应商阻塞故障、ASan/Application Verifier 和长时间资源趋势仍未完成。

## 已实现并验证

- 删除 5 组无消费者的平行/占位实现，共 10 个文件。
- 清理工程树中已迁移到独立机台树的节点类型、选择、显隐和菜单死分支。
- TOML、工具、流程和设置命令异常统一写入 `lcnc::Logger`。
- 架构脚本新增 pure-algorithm、设备公共头和 settings 注入门禁。
- `build/` 已恢复为 Ninja Multi-Config 唯一生成树。
- ACS+GTN Debug 成功生成 `x64/Debug/LaserCNC.exe`。
- CTest 7/7 通过。

完整问题、证据和发布判断见 [AUDIT.md](AUDIT.md)。

## 仅静态或构建验证

- Process OCC-free、core/view 分层、淘汰 API、settings singleton、CMake 源文件收录。
- DeviceCommandQueue 与 ProcessDeviceCoordinator 的结构关系。
- real-laser 源码的异常日志修正；当前 ACS+GTN preset 未编译真实激光实现。

## 仍需外部环境/真机验证

- ACS、GTN、SimulatorCMHP 和真实激光器的连接、回零、加工、暂停、Stop、急停和断开。
- 不可中断供应商调用的超时、对象保活与关机故障注入。
- all-off、ACS-only、GTN-only、real-laser、ASan 构建矩阵。
- GUI 启动/退出、Application Verifier/页堆和 8 小时资源趋势。

实施顺序见 [todo.md](todo.md)，架构事实见 [ARCHITECTURE.md](ARCHITECTURE.md)。
