# LaserCNC

LaserCNC 是面向五轴激光加工的 CAD + CAM + Process 一体化桌面软件。当前代码基线采用单项目微内核框架：一个进程内只有一个项目会话，`LcncProjectManager` 统一管理 Workpiece、Machine、CAM 三个数据域，显示层共享唯一 workspace `GuiDocument`。

## 当前状态

- `Kernel` 是进程内唯一全局入口，负责核心服务、事件总线、模块生命周期和服务定位。
- `LcncProjectManager` 是新建、打开、保存、导入和 `.lcnc` 工程包 IO 的唯一入口。
- Workpiece、Machine、CAM 使用三份独立 `LcncDocument` 保存 OCC/XCAF 数据。
- `LcncProjectSession` 保存项目名、路径、manifest、dirty flags 和各域项目级 metadata。
- `GuiApplication` 只拥有一个 workspace `GuiDocument`，中央 `WidgetOccView` 只 attach 这个视图。
- `GuiDocument` 使用 `DocumentId + XCAF entry` 注册 AIS 对象，支持按 Workpiece/Machine/CAM domain 局部刷新。
- CAD 显式操作 Workpiece 域；CAM 显式操作 Machine/CAM 域并只读引用 Workpiece；Process 执行页保持独立 tab，不进入 ProjectExplorer 树。

## 技术栈

| 项 | 选型 |
| --- | --- |
| 语言 | C++17 |
| UI | Qt 6.9.1 + SARibbon |
| 几何内核 | OpenCASCADE 7.9.0 / XCAF |
| 配置 | toml11 |
| 日志 | spdlog |
| 构建 | CMake 3.20+ / MSVC 2022 x64 |

## 架构速览

```text
app / UI / commands
    ↓
modules facade / services
    ↓
view
    ↓
core/algorithms
    ↓
core
```

```text
LcncProjectManager
├── LcncProjectSession
├── Workpiece LcncDocument
├── Machine LcncDocument
└── CAM LcncDocument
```

```text
GuiApplication
└── workspace GuiDocument
    ├── GraphicsScene
    ├── AIS_InteractiveContext
    ├── V3d_View
    ├── RenderingManager
    └── Display registry: (DocumentId, entry) → AIS_Shape
```

## 主要目录

| 路径 | 用途 |
| --- | --- |
| `src/main.cpp` | QApplication、OpenGL、Kernel、模块和 MainWindow 启动入口。 |
| `src/core/` | Kernel、project、document、settings、task、command、算法和运动学。 |
| `src/view/` | OCC scene、workspace GuiDocument、WidgetOccView、渲染器和显示管理。 |
| `src/modules/cad/` | Workpiece/CAD 模块：导入导出、建模、选择、草图、特征和 CAD 命令。 |
| `src/modules/cam/` | Machine/CAM 模块：机台、轴标定、挂载、刀路、CAM runtime 数据和仿真。 |
| `src/modules/process/` | Process 模块：加工执行、仿真控制、连接/回零/急停/倍率。 |
| `src/app/` | MainWindow、AppContext、CommandRegistry、ProjectExplorer 和应用对话框。 |
| `resources/` | Qt resource collection 与 SVG 图标。 |
| `3rd/` | vendored spdlog 和 toml11。 |

## `.lcnc` 工程包

`.lcnc` 是特殊后缀 zip 包，当前内部基础结构为：

```text
project.toml
project.xbf
```

- `project.toml`：manifest、版本、资源索引和项目 metadata。
- `project.xbf`：OCC/XCAF binary snapshot。
- 当前 package backend 使用临时目录和 PowerShell `Compress-Archive` / `Expand-Archive`；后续规划替换为可测试的 zip backend。

## 构建

推荐使用 VS Code CMake Tools 构建 `LaserCNC` target。当前 Debug 构建已验证通过。

主要 CMake cache 变量：

| 变量 | 含义 |
| --- | --- |
| `LCNC_QT6_ROOT` | Qt 安装根目录 |
| `LCNC_OCCT_ROOT` | OpenCASCADE 安装根目录 |
| `LCNC_OCCT_3RDPARTY_DIR` | OpenCASCADE 3rdparty lib 根目录 |
| `LCNC_SARIBBON_ROOT` | SARibbon 安装根目录 |
| `OCC_BIN_DIR` | OCC runtime DLL 目录 |
| `SARIBBON_BIN_DIR` | SARibbon runtime DLL 目录 |

## 文档索引

| 文档 | 用途 |
| --- | --- |
| [文件结构.md](文件结构.md) | 当前目录结构和源码文件职责。 |
| [微内核框架结构.md](微内核框架结构.md) | 当前单项目微内核框架结构。 |
| [微内核架构框架.md](微内核架构框架.md) | 微内核约束和设计规则。 |
| [todo.md](todo.md) | 本次审阅发现的问题和下一阶段开发规划。 |
| [代码规范.md](代码规范.md) | 代码风格和工程约束。 |
| [ModuleReadme.md](ModuleReadme.md) | 模块化接入说明。 |
