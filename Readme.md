# LaserCNC

LaserCNC 是面向五轴激光加工的 Windows 桌面软件，将 CAD、CAM、离线仿真与 Process 加工执行放在统一工程工作区中。项目使用 C++17、Qt 6、OpenCASCADE/XCAF、SARibbon、QuaZip、toml11 与 spdlog。

当前版本为 `1.6.0`。本轮在既有审计整改基线上完成 `.lmsp/.lmsi → Job Overlay → Surface-BVH/Coal → 连续运动证书 → Process O(1)` 软件碰撞闭环，并统一首刀、刀路、固定运动和连续点动的失败关闭入口。代码适合作为继续开发和自动化回归的稳定基座；正式夹具、扩大 exact 审计、GUI/长稳和物理设备验证仍未完成，因此不能据此标记为实体机生产发布。

## 系统组成

| 区域 | 主要职责 |
| --- | --- |
| `src/core/` | Kernel、服务注册、工程/文档、任务、设置、运动学、CAD/CAM 纯算法和无 UI 数据契约。 |
| `src/view/` | OCC 场景、`GuiApplication`/`GuiDocument`、视图控件和渲染器。 |
| `src/modules/cad/` | 工件导入导出、建模、草图、选择、CAD 命令与 UI。 |
| `src/modules/cam/` | 机台、加工面、轮廓、刀路求解、空程、碰撞、显示和 CAM UI。 |
| `src/modules/simulation/` | 冻结 CAM 快照后的只读离线机台仿真。 |
| `src/modules/process/` | 加工流程、预检、运动执行、控制器、激光、IO 与状态监控。 |
| `src/app/` | `MainWindow`、工作区呈现、命令接线、工程树和应用级对话框。 |
| `tests/` | 架构、算法、安全、制造流程、SDK 模拟器与启动回归。 |

模块生命周期由 `Kernel` 按 `cad -> cam -> {simulation, process}` 编排。每个 `ProjectWorkspace` 拥有一个统一的 Workpiece+CAM XCAF 文档、`CamDataManager` 和项目会话；机台模型由 Kernel 的独立 `MachineWorkspace` 持有，不写入 `.lcnc`。

CAM 是轮廓顺序、切割偏置、Retract/Traverse/Approach 空程和最终物理轴坐标的唯一生产者。Process 与离线仿真消费 OCC-free、不可变快照，不得重新排序或再次求解。完整设计与当前偏差见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 构建与测试

构建要求 CMake 3.20+、MSVC 2022 x64、Qt 6.9.1、OpenCASCADE 7.9.0 与 SARibbon。两条生成路线必须隔离：

| 路线 | 生成树 | 运行输出 |
| --- | --- | --- |
| CMake/Ninja | `build-cmake/` | `x64/ninja/<Config>` |
| Visual Studio/MSBuild | `build-vs/` | `x64/vs/<Config>` |

日常 Ninja 构建：

```powershell
cmd /c "call \"E:\vs2022IDE\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --preset acs-gtn && cmake --build --preset acs-gtn-debug --parallel 16"
```

完整自动化测试：

```powershell
ctest --test-dir build-cmake --build-config Debug --output-on-failure
```

截至本轮整改，日常 ACS+GTN Debug 构建和 41/41 CTest 通过。新增回归覆盖机台安全包、真实 AC 转台索引、Coal、混合实体/开放面包含、连续证书、首刀与 Process 失败关闭，同时保留既有协议、设备等待和真实 STEP 流程回归。该证据仍不替代 GUI 人工验收、长时间资源趋势或 ACS/GTN/激光物理硬件验证。唯一构建约定见 [BUILD.md](BUILD.md)，测试分层见 [tests/README.md](tests/README.md)。

## 工程包

桌面应用只接受 format v4 `.lcnc` 包，包含：

```text
project.toml
workpiece.xbf
cam_toolpath.toml
cam_toolpath_points.bin
tools.toml
```

保存使用同目录 staging 后原子替换，失败不得破坏旧包。v1/v2/v3 不在桌面应用内兼容；仓库中的 `lcnc_project_upgrade` 是唯一受控离线升级入口，输出必须是不同的 v4 文件。机台指纹不匹配允许查看和仿真，但必须阻止真实加工。

## 当前维护文档

- [ARCHITECTURE.md](ARCHITECTURE.md)：当前架构事实、所有权、调用方向和已知偏差。
- [AUDIT.md](AUDIT.md)：截至 2026-08-24 的全项目文件级审计、风险分级和验证证据。
- [todo.md](todo.md)：从本次审计生成的未完成工作和验收顺序。
- [DELIVERY.md](DELIVERY.md)：当前交付边界与可声明/不可声明的验证结论。
- [代码规范.md](代码规范.md)：代码、分层、异常、设备和提交规范。
- [docs/collision_detection_todo.md](docs/collision_detection_todo.md)：碰撞安全域与路径规划专项长期计划。
- [docs/versions/](docs/versions/)：按版本保留的历史交付记录，不作为当前事实源。

## 提交前最小门禁

```powershell
git diff --check
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/check_architecture.ps1 -Root .
cmake --build --preset acs-gtn-debug --parallel 16
ctest --test-dir build-cmake --build-config Debug --output-on-failure
```

涉及所有权、并发、OCC 或设备 SDK 时还需运行对应变体、ASan/资源趋势及设备级验收；自动化、GUI、SDK 模拟器和物理机证据必须分别记录。
