# LaserCNC

LaserCNC 是面向五轴激光加工的 CAD + CAM + Process 一体化 Windows 桌面软件，使用 C++17、Qt 6、OpenCASCADE/XCAF、SARibbon、toml11、spdlog 与 QuaZip。

## 当前架构

- `Kernel` 统一编排核心服务与 `cad -> cam -> process` 模块生命周期。
- `LcncProjectManager` 支持多工作区；每个工作区拥有一个统一的 Workpiece+CAM XCAF 文档、`CamDataManager` 和项目会话。
- 机台模型由 Kernel 的 `MachineWorkspace` 独立持有，跨工程复用且不写入 `.lcnc`。
- `GuiApplication` 为每个工作区拥有一个 `GuiDocument`，显示对象按文档、XCAF entry 和实体类型注册。
- Process 只消费 CAM 输出的 OCC-free `ToolpathExportSnapshot`，不依赖 OCC 类型。

完整说明见 [ARCHITECTURE.md](ARCHITECTURE.md)，当前审计问题与实施顺序见 [todo.md](todo.md)。

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

当前 format v3 使用 QuaZip，包含 `project.toml`、`workpiece.xbf`、`cam_toolpath.toml` 和 `cam_toolpath_points.bin`。机台模型不属于工程包。

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
