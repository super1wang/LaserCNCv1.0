# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Build

```powershell
cmd /c "call \"C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --build build --config Debug"
```

- `build/` 当前是 Ninja 生成器，不要追加 MSBuild 专用的 `/m /nologo`。

- Single CMake target: `LaserCNC` (WIN32 executable).
- Requires CMake 3.20+, MSVC 2022 x64, C++17.
- Qt 6.9.1, OpenCASCADE 7.9.0, SARibbon — paths configured via CMake cache variables (`LCNC_QT6_ROOT`, `LCNC_OCCT_ROOT`, `LCNC_SARIBBON_ROOT`).
- Vendored 3rd-party libs in `3rd/`: spdlog (logging), toml11 (config).
- OCC and SARibbon DLLs are copied to the output directory via POST_BUILD commands.
- Conditional Process device SDKs (ACS, GTN, BDAQ, real laser) are OFF by default; enable with CMake `-D` options (`LCNC_WITH_ACS`, etc.).
- If build fails with `LNK1168`, the previous `LaserCNC.exe` is still running — kill it and retry.

## Architecture

### Micro-Kernel + Unified Project Document + Workspace-Bound Views

The application is a single-process, multi-workspace desktop app for 5-axis laser machining (CAD + CAM + Process). See `ARCHITECTURE.md` for the authoritative architecture.

**Kernel** (`src/core/kernel/kernel.h`) is the one global entry point (`Kernel::current()`). It owns `AppSettings`, `LcncProjectManager`, `TaskManager`, `MachineConfigurationService`, `ServiceRegistry`, `EventBus`, and `ModuleRegistry`. `GuiApplication` and `CommandContainer` are injected as raw pointers (owned by `main()` and `MainWindow` respectively).

Each `ProjectWorkspace` owns one project `LcncDocument` containing both Workpiece and CAM-kind XCAF entities, plus a dense `CamDataManager` and `LcncProjectSession`. `workpieceDocument()` and `camDocument()` currently alias that same physical document.

The machine document is an independent reference asset owned by Kernel's `MachineWorkspace`; it is not persisted in `.lcnc` and must not dirty a project.

`GuiApplication` owns one `GuiDocument` per open workspace. AIS objects are registered by document id, XCAF entry, domain, and entity kind.

### Layer Dependency (strict, top-down only)

```
app / modules/*/ui / commands
    → modules facade / services
    → view
    → core/algorithms
    → core
```

Hard rules:
- `core/**` must **never** include `view/`, `modules/`, or `app/`.
- `view/**` must **never** include `modules/` or `app/`.
- `core/algorithms/**` must not depend on `QWidget`, `QAction`, `QDialog`, `LcncDocument`, `GuiDocument`, or `Kernel`.
- Cross-module calls use facade/service contracts; cross-module notifications use EventBus or Qt signals.

### Module Lifecycle

Modules implement `IModule` (`src/core/kernel/i_module.h`): `info()`, `init(kernel)`, `start()`, `stop()`.
Dependency order is `cad → cam → process`, enforced by topological sort in `ModuleRegistry`.
In `main.cpp`: construct Kernel → registerCoreServices → load settings → inject GuiApplication → add modules → kernel.bootstrap() → MainWindow → app.exec().

### Startup Order (in `main.cpp`)

1. Set Qt/OpenGL surface format (must precede `QApplication`).
2. Create `QApplication`.
3. Initialize `Logger`.
4. Construct `Kernel`, call `registerCoreServices()`, `appSettings()->loadDefault()`.
5. Create `GuiApplication`, inject into Kernel via `setGuiApp()`.
6. Add modules (`CadModule` → `CamModule` → `ProcessModule`); skip any disabled by `[modules].disabled` in `mainwindow.toml`.
7. `kernel.bootstrap()` (topological sort → init → start).
8. Create `MainWindow`, call `show()`, enter event loop.
9. On exit: save settings, `kernel.shutdown()` (reverse stop), shutdown logger.

## Key Conventions

### Naming & Style (from `代码规范.md`)
- Files: `snake_case.{h,cpp}`; classes/structs: `PascalCase`; functions: `camelCase`; members: `m_camelCase`; constants: `kCamelCase` or `UPPER_SNAKE_CASE`.
- Headers use `#pragma once`; prefer forward declarations, keep heavy Qt/OCC includes in `.cpp`.
- `.cpp` include order: own header → `core` → `view` → `modules` → `app` → third-party → standard library.
- No `using namespace` in headers.

### Logging
```cpp
LCNC_INFO(lcnc::LogCode::Xxx, "fmt {}", arg);
LCNC_DEBUG / LCNC_INFO / LCNC_WARN / LCNC_ERR / LCNC_CRIT
```
Every catch block must log with `LCNC_ERR`. `LCNC_CRIT` only writes a critical log;
the caller must still return, throw, or terminate explicitly.

### Banned Legacy APIs
New code must not use: `projectDocument()`, `workspaceGuiDocument()`, `ensureProjectDocument()`, or the `sourceDocument()` path on `GuiDocument`. Use `workpieceDocument()`, `machineDocument()`, `camDocument()` and document/domain-aware display APIs.

### Algorithms (`core/algorithms/`)
Pure OCC/math, no UI or document ownership. Free functions preferred. Namespaces: `lcnc::cad_algo`, `lcnc::cam_algo`, `lcnc::kinematics`. OCC `Standard_Failure` caught by caller and logged.

## Process Module (`src/modules/process/`)

The Process module handles execution/simulation, not geometry. Critical boundary: **Process must never include OCC types** (`TopoDS_*`, `AIS_*`, `gp_*`, `BRep*`, `XCAF*`). It consumes only the OCC-free `ToolpathExportSnapshot` DTO from CAM via `ICamToolpathProvider`.

Current runtime structure:
- `ProcessModule` — facade and coordinator for state, connection, preflight, monitoring and workflow.
- `ProcessCuttingPlanService` + `ProcessToolpathService` + `NormalCuttingManager` — prepare and execute the CAM snapshot.
- `MotionSinkFactory` — selects PureSimulation, ACS text, or GTN buffered execution.
- `ProcessWorkflowExecutor` + step registry — executes the editable process flow.

`ProcessDeviceCoordinator` is still planned work; until it exists, do not add new direct SDK access paths. Track the remaining safety work in `todo.md`.

Controller adapters (`PureSimulation`, `SimulatorCMHP`, ACS, GTN) are behind `IMotionController`. Vendor SDK headers (ACSC.h, gts.h) must NEVER appear in public interfaces — they stay in option-gated private `.cpp` files.

## `.lcnc` Project Package

QuaZip archive (format v3) containing:
- `project.toml` — manifest, version, metadata
- `workpiece.xbf` — Workpiece/Auxiliary and CAM-kind XCAF entities
- `cam_toolpath.toml` + `cam_toolpath_points.bin` — CAM metadata and dense points

Save/load goes through `LcncProjectManager` → `LcncProjectPackage`.

## Pre-Commit Verification

1. Build passes using the command in the Build section.
2. Layer check: no `core/**` includes `view/modules/app`; no `view/**` includes `modules/app`
3. No legacy API usage: grep for `projectDocument\|workspaceGuiDocument\|ensureProjectDocument`
4. Process module: no OCC includes (grep for `TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF` in `src/modules/process/`)
