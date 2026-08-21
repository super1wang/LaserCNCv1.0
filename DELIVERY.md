# LaserCNC 当前交付状态

更新日期：2026-08-21

产品版本：`1.5.9`（由 CMake 单一版本源生成）

## 当前结论

本次已经关闭审计 P0-1～P0-3，并完成当前范围内的 P1/P2 架构与工程卫生整改。代码可作为后续功能开发和下一版本完整碰撞工作的稳定基座；P0-4 连续碰撞、GUI/长稳及物理设备验证仍是实体机生产发布门禁。

## 本次收口

- 真实激光：ULTRON 协议改为有界纯函数和 RAII；Raycus/QCW 清除裸缓冲区、参数检查错误及不确定返回。
- 设备稳定性：ACS/GTN 等待统一 deadline/cancellation；状态轮询与工作流共用全局优先级设备队列。
- CAD：STEP/IGES/STL/BREP 在 worker 构造 detached payload，文档所有者线程提交；任务按文档跟踪和取消。
- 跨模块：Process/Simulation 不再依赖具体 `CamModule`；离线仿真消费 revision 化不可变快照。
- 所有权：`ServiceRegistry` 显式注册 borrowed service；CAM pipeline 不再外泄可变 entry 容器。
- 目录与规范：供应商 BDAQ 头迁入 `3rd/`，`setting/` 合并到 `settings/schema/`，设备头统一 `#pragma once`，增加统一数学常量、`.clang-format` 和架构门禁。
- 构建：模块实现/UI 链接依赖尽量收紧为 `PRIVATE`；CMake 版本、应用版本、模块信息和工程包 fallback 统一为 `1.5.9`。

## 验证

| 验证 | 结果 |
| --- | --- |
| `git diff --check` | 通过；仅有 Git 行尾转换提示。 |
| `scripts/check_architecture.ps1 -Root .` | 通过。 |
| 日常 ACS+GTN Debug 构建 | 通过。 |
| 日常完整 CTest | 39/39 通过，59.97 秒。 |
| real-laser Debug 构建 | 通过；原 ULTRON 未初始化/缺失返回和 Raycus/QCW 协议告警已消除。 |
| ASan | 全量构建通过；本轮关键所有权、协议、等待、状态和 CAD detached 导入 7/7 通过，6.10 秒。 |

## 仍未放行

1. P0-4：完整机台连续段碰撞证明、重规划和性能门限。
2. GUI 人工验收、8 小时资源趋势和重复连接/开关压力测试。
3. 真实 ACS/GTN、激光器和 IO 的低速加工及安全停机验证。
4. 供应商函数自身永不返回时的厂商级超时或进程外看门狗。

自动化、SDK 仿真、GUI 和物理机是四类独立证据。完整审计见 [AUDIT.md](AUDIT.md)，剩余工作见 [todo.md](todo.md)。
