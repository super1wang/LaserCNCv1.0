# LaserCNC 全项目文件级代码与架构审计

审计日期：2026-08-21

源码基线：`47e5408`（`main`）

审计前工作区：干净

## 结论

项目的总体方向合理：Kernel/模块生命周期、统一工程文档、独立机台工作区、workspace-bound view、CAM 权威快照、Process OCC-free 边界和自动化分层均已成立。最新 CAM 拆分也明显降低了单个入口实现文件的密度。

但当前仍是“可继续集成开发，不可据此声明实体机生产就绪”。本轮发现 4 组 P0：真实激光适配器存在可静态证明的未定义行为；ACS/GTN 有无 deadline 的等待环；CAD worker 直接修改活动 XCAF 文档；完整机台碰撞仍缺连续段安全证明。另有 concrete Module 耦合、入口/头文件过大、版本漂移和规范执行不足等 P1/P2 债务。

日常 ACS+GTN Debug 构建与 35/35 CTest 全部通过，说明现有自动化基线稳定；这些测试没有覆盖上述真实激光代码、供应商卡死、CAD 活动文档并发写或物理机连续碰撞，因此绿色结果与审计问题并不矛盾。

## 1. 范围和方法

本轮扫描并抽查全部已跟踪源码、测试、CMake、维护脚本和项目文档：

| 范围 | 文件/规模 | 审阅重点 |
| --- | ---: | --- |
| `src/` 自有 `.h/.cpp` | 440 文件，约 90,414 行 | 分层、职责、所有权、并发、异常、设备和规范；不含 5,011 行供应商 `bdaqctrl.h`。 |
| `tests/` | 30 个源码/脚本，约 5,473 行 | 算法、流程、安全、SDK、GUI/长稳覆盖缺口。 |
| CMake/脚本 | `CMakeLists.txt` 1,324 行及 presets/scripts | target 依赖、可选变体、源文件收录和门禁真实性。 |
| 文档 | 审计前 60 份版本记录及根目录/专题文档；本轮新增第 61 份记录 | 与当前源码、构建和测试事实交叉核对。 |

执行的方法包括：文件清单与行数热点、include/target 依赖、淘汰 API、Process OCC、跨模块具体类型、TaskManager/QtConcurrent、SDK 锁与等待环、裸内存、头文件规范、长行/缩写实现、版本值和文档引用扫描；随后运行日常构建、完整 CTest 和 real-laser 独立构建。

## 2. P0：必须优先关闭

### P0-1 真实激光 ULTRON 初始化存在未定义行为

位置：`src/modules/process/device/laser/ultron_laser_device.cpp:390-466`

- `charArray` 在循环内 `delete[]` 后继续被 `std::copy` 和 `QByteArray` 使用，形成释放后写/读。
- 外层 `ivalue` 未初始化，内层又声明同名变量；循环条件读取未初始化的外层值。
- `InitLaser()` 存在非所有路径返回值。

real-laser `/W4` 构建实际报告 C4456、C4700、C4715，静态源码还能直接证明释放后使用。该变体虽然最终链接成功，但不能进入真实激光调试或验收。应改用 `QByteArray`/`std::array` RAII，消除裸数组、变量遮蔽并为全部路径返回明确结果；增加协议帧、超时、失败响应和 ASan 回归。

### P0-2 Stop/关机可能被无界供应商等待阻塞

位置示例：

- `acs_motion_control.cpp:1155-1156`：`StopBuffer()` 无 deadline 等待 buffer 结束。
- `gtn_motion_control.cpp:615-618`、`638-641`、`1517-1520`：停止后无 deadline 等待轴停止。
- `gtn_motion_control.cpp:1726-1729`：建坐标系前无 deadline 等待。
- `gtn_motion_control.cpp:2092-2099`：忙轮询规划停止，无 sleep/deadline。

`ProcessStatusService` 的两个轮询队列和主设备队列共享 `ProcessDeviceCoordinator` 租约；已经运行的调用不可由 Stop lane 抢占。任何无界循环都可能占住租约，使安全停机、断开和模块关闭超时。所有等待必须统一使用 monotonic deadline、stop token、有限轮询间隔和具体错误；为“状态永不结束/SDK 不返回”加入故障注入测试。

### P0-3 CAD worker 直接修改活动 XCAF 文档

位置：`cad_module.cpp` 的 open/import worker 与 `cad_document_io_service.cpp` 的 `import*IntoDocument()`。

异步 lambda 捕获裸 `LcncDocument*`，在 TaskManager worker 中调用 STEP/IGES transfer、`addShapeEntity()` 和网格准备。导入到现有工作区时，GUI/view 可能同时读取同一文档；失败或取消也缺少统一原子提交边界。当前关闭守卫能降低悬空风险，但不能提供 XCAF 线程安全或事务保证。

应让 worker 只产生 detached `TopoDS_Shape`/临时文档和诊断；以 workspace id、document id、generation 校验后在文档线程一次提交。任务所有权从模块级细化到文档级，关闭、切换和新导入只取消相关任务。

### P0-4 完整机台碰撞仍是离散验证

当前 CAM 已能失败关闭 Pending/Indeterminate/过期快照，并有 BVH/精确距离证据，但采样点之间尚无连续段扫掠或保守细分证明，也不会在碰撞后自动重规划。完整机台模式的高耗时尚无稳定性能门限。真实加工必须继续把该能力视为未完成发布门禁，详见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)。

## 3. P1：结构和耦合

| 编号 | 文件/范围 | 发现 | 建议 |
| --- | --- | --- | --- |
| P1-1 | `app/app_context.*`、`main_window.cpp`、module UI/commands | facade 已存在，但 App 仍返回并广泛调用具体 `CadModule/CamModule/ProcessModule`。 | 扩充窄 contract/controller；迁移后取消具体 Module 的公共服务注册。 |
| P1-2 | `simulation_module.cpp` | 直接获取 `CamModule` 读取配置、机台和场景，同时只对刀路使用 contract。 | 增加完整的 immutable offline-simulation snapshot provider。 |
| P1-3 | CMake target graph | CAD/CAM/Simulation/Process/App 依赖多为 `PUBLIC`，实现头和 UI 依赖被传递暴露。 | 拆 contracts target，能 `PRIVATE` 的链接改为 `PRIVATE`。 |
| P1-4 | `ServiceRegistry` 与模块 init | 使用 no-op deleter `shared_ptr(this)` 注册具体模块，表现为共享所有权但实际非拥有。 | 使用显式非拥有句柄，或注册真正独立拥有的 facade/service。 |
| P1-5 | Process 设备拓扑 | 主队列、硬件轮询队列、外设轮询队列通过全局递归 mutex 串行，Stop 优先级不是全局优先级。 | 收敛为单设备 actor，或建立可证明有界、可抢占语义的命令协议。 |
| P1-6 | `MachiningFacePipelineService` | 仍暴露 mutable `entries()`，`CamModule` 长期持有可变引用。 | service 独占写入，只发布 immutable snapshot/revision。 |
| P1-7 | 版本信息 | CMake 为 `1.2.1`、应用为 `1.0.0`、本轮文档交付记录为 `v1.5.9`。 | 用 CMake 单一版本源生成应用、manifest、about 和版本文档元数据。 |
| P1-8 | 测试边界 | 缺 CAD IO 并发/取消事务、双工作区 AIS、真实激光协议、供应商卡死、完整 Stop 故障注入。 | 优先增加失败注入和真实输入端到端测试，再扩展功能测试。 |

## 4. 文件级热点

以下为自有代码中最高风险/最高维护密度文件；行数不是缺陷本身，但与职责数量和变化频率结合后应作为拆分顺序：

| 文件 | 行数 | 主要问题 |
| --- | ---: | --- |
| `cam/toolpath/cam_module_toolpath.cpp` | 2,888 | 多条刀路入口、偏置、顺序、IK、显示状态和提交逻辑仍共享入口状态。 |
| `app/main_window.cpp` | 2,774 | Ribbon、工作区、CAD/CAM/Process 接线、工程树、最近文件和仿真入口集中。 |
| `core/algorithms/cam/laser_toolpath.cpp` | 2,374 | 面分类、轮廓、引线、偏置与显示几何混在同一算法文件。 |
| `process/process_module.cpp` | 2,283 | service 装配、run/stop、UI 状态、仿真和设备事务仍高度集中。 |
| `process/device/motion_control/gtn_motion_control.cpp` | 2,197 | 旧式资源管理、阻塞等待和完整供应商表面集中。 |
| `cad/cad_module.cpp` | 2,086 | 文档 IO 编排、建模、草图、选择和显示协调尚未完全下沉。 |
| `process/device/motion_control/acs_motion_control.cpp` | 1,818 | 控制器会话、回零、buffer、IO 和阻塞等待集中。 |
| `app/dialog/dialog_options.cpp` | 1,760 | 应用、渲染、机台与 Process 配置在单对话框实现内耦合。 |
| `cam/collision/cam_module_collision.cpp` | 1,583 | 碰撞缓存、扫描、首段安全域和异步发布仍依赖 CamModule 大状态面。 |
| `cam/pipeline/cam_module_pipeline.cpp` | 1,583 | 多阶段 pipeline、task watcher、revision 提交和 UI 信号集中。 |
| `simulation/simulation_module.cpp` | 1,491 | session/page/快照/回放/碰撞扫描全部在两个文件内。 |

`bdaqctrl.h` 为 5,011 行供应商头，不应计入自有代码质量指标，长期应迁入明确的 `3rd/` SDK 边界。

## 5. P2：规范和工程化

- 9 个 Process 设备头仍使用传统 include guard 而非规范要求的 `#pragma once`。
- 多个激光公共头包含 `using namespace std;`，会污染所有包含者命名空间。
- 排除供应商头后仍有 319 行超过 140 字符；Process steps/settings 中存在 500-1,488 字符的压缩单行实现。
- 16 个 Process 文件同时混用 tab 和四空格；与新代码风格差异明显。
- `setting/` 与 `settings/` 两个同级目录表达相近职责，增加查找和 include 认知成本。
- 多个源文件重复定义 `M_PI`，应统一为项目 `constexpr` 数学常量。
- `CMakeLists.txt` 手工重复测试 target、runtime deployment、ASan deployment 和标签注册；应提取 helper/CMake module。
- 当前门禁未检查 concrete Module 依赖、供应商有界等待、catch 必须日志、格式/长行或新增 `tr()` catalog 完整性。

建议引入仓库级 `.clang-format`、格式检查 target 和渐进式基线：先要求所有新增/修改文件通过，再分批清理旧设备适配器，避免一次全局格式化掩盖功能差异。

## 6. 已成立的优点

- `scripts/check_architecture.ps1` 当前通过；core/view 反向 include、Process OCC include、淘汰文档 API和孤儿 `.cpp` 均为 0。
- CAM/Process 通过 OCC-free immutable snapshot 分隔，碰撞 Pending/Indeterminate/过期状态能失败关闭。
- 工程包 v4 staging/原子替换、工具快照、机台指纹和离线升级工具有回归。
- TaskManager、DeviceCommandQueue、Process preflight/status/connection/manual motion 和制造流程已有独立可定位测试。
- CAM 最新拆分把 `cam_module.cpp` 缩为生命周期和装配入口，并修复了有序求解的事务提交。
- 自动化明确区分 algorithm/flow/safety/sdk-integration/support 标签，文档也没有把 SimulatorCMHP 冒充物理机证据。

## 7. 验证证据

| 验证 | 结果 |
| --- | --- |
| `scripts/check_architecture.ps1 -Root .` | 通过。 |
| `cmake --build --preset acs-gtn-debug --parallel 16` | 通过，`x64/ninja/Debug/LaserCNC.exe` 链接成功。 |
| `ctest --test-dir build-cmake --build-config Debug --output-on-failure` | 35/35 通过，57.08 秒。 |
| `cmake --preset real-laser` + `cmake --build --preset real-laser --parallel 16` | 构建/链接通过，但出现真实激光源 `/W4` 告警；ULTRON 的 C4456/C4700/C4715 与 P0-1 一致。 |

本轮未执行 ASan、Application Verifier、GUI 人工遍历、8 小时资源趋势或物理 ACS/GTN/激光验证。real-laser 构建通过不代表协议和内存安全通过。

## 8. 发布判断

当前适合作为后续架构收口和功能开发基线；在 P0-1/P0-2/P0-3、连续碰撞门禁及对应失败注入回归完成前，不建议启用真实激光或声明实体机生产安全。实施顺序以 [todo.md](todo.md) 为准。
