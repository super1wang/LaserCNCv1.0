# LaserCNC

LaserCNC 是面向五轴激光加工的 CAD + CAM + Process 一体化 Windows 桌面软件，使用 C++17、Qt 6、OpenCASCADE/XCAF、SARibbon、toml11、spdlog 与 QuaZip。

## 当前架构

- `Kernel` 统一编排核心服务与 `cad -> cam -> process` 模块生命周期。
- `LcncProjectManager` 支持多工作区；每个工作区拥有一个统一的 Workpiece+CAM XCAF 文档、`CamDataManager` 和项目会话。
- 机台模型由 Kernel 的 `MachineWorkspace` 独立持有，跨工程复用且不写入 `.lcnc`。
- `GuiApplication` 为每个工作区拥有一个 `GuiDocument`，显示对象按文档、XCAF entry 和实体类型注册。
- 当前视图统一由 `activeGuiDocument()` 取得；工作区切换使用带 `ProjectWorkspaceId` 的明确通知，避免同义文档 API。
- Process 只消费 CAM 输出的 OCC-free `ToolpathExportSnapshot`，不依赖 OCC 类型。
- Process 的回零顺序、轴定义比较与仿真轴坐标在独立 `process_axis_utilities` 中实现，不混入设备协调或 UI。
- Process 的 ACS/GTN/激光/IO 调用经过 `ProcessDeviceCoordinator` 串行租约；ACS/GTN、激光设备和参数注册表均使用构造注入的设置服务，安全输出复位失败会进入 Error 或 EmergencyStop。
- ProcessModule 持有 `ProcessRuntimeConfiguration`；ACS/GTN 的轴选择、扩展轴和仿真模式通过它传入设备层，`BASE` 伪轴会在配置边界过滤；旧 `DT` 静态运行时状态已删除。
- Process 连接、断开和回零任务具备模块级取消与有界关机等待；超时不会销毁仍被 SDK 调用的设备对象。`SimulatorCMHP` 属于 ACS Simulator 并加载随程序部署的 `Simulator.prg`；PureSimulation 仅可显式选择，启用 ACS 或 GTN 时默认关闭，实体控制器连接失败不会自动切换为仿真。
- 设备停机统一先关闭激光输出，再停止运动和断开控制器。
- UI 设备连接命令统一触发异步全设备连接/断开，不再暴露同步单控制器接口。
- 工具配置切换会替换同索引旧参数；缺失工具会走明确的默认工具逻辑，而不会隐式创建空工具。
- 控制器和激光器公共接口不再包含旧 `MessageModule`；旧日志体系的剩余迁移在 `todo.md` 跟踪。
- 消息提示和设备兼容日志均统一写入 `lcnc::Logger`；旧三日志模块已移除。
- Process 监控与 IO 配置由模块注入的设置服务读取，不依赖设置全局单例。
- 模块生命周期异常由 `ModuleRegistry` 与 `main()` 双层边界记录和反向清理，避免异常越过启动/关闭流程。

完整说明见 [ARCHITECTURE.md](ARCHITECTURE.md)，当前审计问题与实施顺序见 [todo.md](todo.md)。

内存检查可使用 `cmake --preset asan`、`cmake --build --preset asan`，再以 `scripts/collect_runtime_baseline.ps1` 对 ASan 产物采集资源基线；Application Verifier 仅通过 `scripts/application_verifier.ps1 -Enable` 显式配置。

架构门禁运行 `ctest --test-dir build/debug --output-on-failure`；它检查分层依赖、Process OCC 边界、淘汰 API 与孤儿源文件。

设备 SDK 构建使用 `debug`（all-off）、`acs` 和 `gtn` preset。GTN 和 ACS adapter 均由各自开关控制，并通过构造注入的 Process 设置服务读取配置；all-off 使用不依赖供应商 SDK 的本地 `Simulator` 与 `PureSimulationSink`。控制器状态以 150 ms 在专用单线程池采集，安全 IO 以 500 ms 采集，串口外设以 2 s 低频采集且串口对象不归属 GUI 线程。

## 主要目录

| 路径 | 职责 |
| --- | --- |
| `src/core/` | Kernel、工程/文档、CAM 核心数据、设置、任务、运动学和纯算法。 |
| `src/view/` | OCC 场景、每工作区 GuiDocument、视图控件与渲染器。 |
| `src/modules/cad/` | 工件导入/导出、CAD 建模、草图、选择与命令。 |
| `src/modules/cam/` | 机台、轮廓、离散/求解、图层、引线、排序和 CAM UI。 |
| `src/modules/process/` | 加工流程、前置检查、刀路执行、控制器/激光/IO 与监控。 |
| `src/app/` | MainWindow、工作区视图、命令注册、ProjectExplorer 和应用对话框。 |
| `resources/` | Qt 资源和 SVG 图标。 |
| `3rd/` | vendored 依赖、硬件 SDK 与运行时快照。 |

## 构建

要求 CMake 3.20+、MSVC x64、Qt 6.9.1、OpenCASCADE 7.9.0 与 SARibbon。构建目录若使用 Ninja，不能附加 MSBuild 的 `/m /nologo` 参数。

```powershell
cmd /c "call \"C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --build build --config Debug"
```

本地 SDK 路径通过 `LCNC_QT6_ROOT`、`LCNC_OCCT_ROOT`、`LCNC_SARIBBON_ROOT`、`LCNC_QUAZIP_ROOT` 等 CMake cache 变量配置。硬件开关包括 `LCNC_WITH_ACS`、`LCNC_WITH_GTN`、`LCNC_WITH_BDAQ`、`LCNC_WITH_REAL_LASER`。

## `.lcnc` 工程包

当前 format v4 使用 QuaZip，包含 `project.toml`、`workpiece.xbf`、`cam_toolpath.toml`、`cam_toolpath_points.bin` 和项目工具快照 `tools.toml`。机台模型不属于工程包；manifest 记录软件、机台和算法可追溯信息。桌面端仅打开 v4；旧 v1/v2/v3 工程必须先运行 `lcnc_project_upgrade <input.lcnc> <output.lcnc> [--tools <tools.toml>]`，工具不会覆盖输入文件。升级器会保留源包中的 `tools.toml`；没有快照的旧包必须通过 `--tools` 提供完整快照，不能把全局工具名当作项目参数。
若工程记录的机台构型与当前机台不一致，软件会提示该差异：允许查看和仿真，但会禁止真实加工，直至确认配置后重新保存工程。
归档保存使用 staging 文件后原子替换，保存失败会保留旧工程包。
工程包回归测试为 `ctest --test-dir build --output-on-failure`，其中包含 v4 工具快照 round-trip、缺快照拒绝、失败保存不改写既有包，以及实际离线工具的 v1/v2/v3 结构 fixture 升级。
同一 CTest 套件还覆盖 TaskManager 的协作取消、超时与异常失败边界，以及 Process 运行时配置的轴归一化、伪轴过滤和权限状态。
内存检查可使用 `cmake --preset asan && cmake --build --preset asan && ctest --test-dir build/asan --output-on-failure`；ASan preset 会自动部署其运行时和 OCCT TBB DLL。
Process 配置在启动时完成校验后才创建设备服务；默认工具、控制器和激光器均从同一份已注入设置读取。
资源采集使用 `scripts/collect_runtime_baseline.ps1`；它输出 CSV 并可用 `-MaxPrivateBytesGrowth`、`-MaxHandleGrowth` 设置长期门禁。启动初始化阶段应单独观察，不应与稳定段混为泄漏结论。

## 提交前检查

```powershell
git diff --check
rg -n '#include\s*[<"](view|modules|app)/' src/core
rg -n '#include\s*[<"](modules|app)/' src/view
rg -n 'TopoDS_|AIS_|gp_|Geom_|BRep|XCAF' src/modules/process
```

随后执行 Debug 构建，并按改动域完成启动、打开/关闭工程、CAM 生成/重算、Process 仿真或硬件安全检查。

## 维护文档

- [ARCHITECTURE.md](ARCHITECTURE.md)：唯一架构事实源。
- [todo.md](todo.md)：审计结果与改进计划。
- [代码规范.md](代码规范.md)：编码、分层和安全约束。
- `AGENTS.md` / `CLAUDE.md`：对应开发工具的仓库操作说明。
