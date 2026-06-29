# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```powershell
cmake --build build --config Debug -- /m /nologo
```

- Single CMake target: `LaserCNC` (WIN32 executable).
- Requires CMake 3.20+, MSVC 2022 x64, C++17.
- Qt 6.9.1, OpenCASCADE 7.9.0, SARibbon — paths configured via CMake cache variables (`LCNC_QT6_ROOT`, `LCNC_OCCT_ROOT`, `LCNC_SARIBBON_ROOT`).
- Vendored 3rd-party libs in `3rd/`: spdlog (logging), toml11 (config).
- OCC and SARibbon DLLs are copied to the output directory via POST_BUILD commands.
- Conditional Process device SDKs (ACS, GTN, BDAQ, real laser) are OFF by default; enable with CMake `-D` options (`LCNC_WITH_ACS`, etc.).
- If build fails with `LNK1168`, the previous `LaserCNC.exe` is still running — kill it and retry.

## Architecture

### Micro-Kernel + Three-Domain Documents + Single Workspace View

The application is a single-process, single-project desktop app for 5-axis laser machining (CAD + CAM + Process).

**Kernel** (`src/core/kernel/kernel.h`) is the one global entry point (`Kernel::current()`). It owns `AppSettings`, `LcncProjectManager`, `TaskManager`, `MachineConfigurationService`, `ServiceRegistry`, `EventBus`, and `ModuleRegistry`. `GuiApplication` and `CommandContainer` are injected as raw pointers (owned by `main()` and `MainWindow` respectively).

**Project core = workpiece geometry + CAM data, both owned by `core/` and saved/loaded as one unit.** `LcncProjectManager` owns:
- **Workpiece `LcncDocument`** (OCC/XCAF) — source geometry, CAD modeling results.
- **CAM data** — `lcnc::cam::CamDataManager` (in `core/project/cam/`) owns the dense, project-core CAM runtime: contours, toolpaths, layers, lead-ins, and **process parameters** (`LaserToolpath` + `LayerContainer`/`LayerManager`). Sparse OCC contour mirrors live in a CAM `LcncDocument` for tree/selection.

**The machine model is an independent reference asset, NOT project data.** `CamModule`'s `MachineWorkspace` *owns* the machine `LcncDocument` + kinematics; it loads from a global config path (`CamConfig::machineModelPath`) and is **never written into `.lcnc`, never marks the project dirty, and is not cleared on new/open project**. `LcncProjectManager` holds only a non-owning *reference* to the machine doc (`attachMachineDocument`) purely so the view layer (`GuiDocument::domainForDocument`) can route machine rendering/selection by domain.

**Persistence is a single transactional writer in `core`.** `LcncProjectManager::saveProject`/`openProject` write/read manifest + `workpiece.xbf` + CAM data (`lcnc::cam::saveCamToolpath`/`loadCamToolpath` in `core/project/cam/cam_toolpath_io.*`) together. Modules do **no** project-file IO — there are no `projectSaved`/`projectOpened` persistence hooks. CAM only refreshes its view after core loads (`CamModule::onCamDataLoaded`); the legacy `process_cutting_plan.toml` v1 migration also runs in core (`migrateLegacyProcessCuttingPlan`).

**CAD / CAM / Process own no project data — they are business logic** over the core data:
- **CAD** edits workpiece geometry through core document APIs.
- **CAM** runs algorithms (extract / discretise / IK), writes results into the core-owned `CamDataManager` (borrowed via `projectManager()->camData()`), and keeps only the machine reference asset + renderers + transient UI/preview state.
- **Process** consumes CAM data read-only via the OCC-free `ToolpathExportSnapshot` DTO.

**Single workspace `GuiDocument`** — workpiece + machine + CAM display into one shared OCC view. AIS objects are registered by `{DocumentId, XCAF entry}` to avoid cross-document conflicts.

> Remaining cleanup (not yet done): `MachineKinematics` still physically lives in the (workspace-owned) machine `LcncDocument` rather than in `MachineWorkspace` itself; and `LcncProjectManager` still exposes `machineDocument()`/`machineDocumentId()` as the view-routing reference accessor (fully excising them needs a `GuiDocument` domain-routing rework). Both are spirit-compliant (machine is independently owned) but not literal-final.

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
Every catch block must log with `LCNC_ERR`. `LCNC_CRIT` throws `std::logic_error`.

### Banned Legacy APIs
New code must not use: `projectDocument()`, `workspaceGuiDocument()`, `ensureProjectDocument()`, or the `sourceDocument()` path on `GuiDocument`. Use `workpieceDocument()`, `machineDocument()`, `camDocument()` and document/domain-aware display APIs.

### Algorithms (`core/algorithms/`)
Pure OCC/math, no UI or document ownership. Free functions preferred. Namespaces: `lcnc::cad_algo`, `lcnc::cam_algo`, `lcnc::kinematics`. OCC `Standard_Failure` caught by caller and logged.

## Process Module (`src/modules/process/`)

The Process module handles execution/simulation, not geometry. Critical boundary: **Process must never include OCC types** (`TopoDS_*`, `AIS_*`, `gp_*`, `BRep*`, `XCAF*`). It consumes only the OCC-free `ToolpathExportSnapshot` DTO from CAM via `ICamToolpathProvider`.

Internal structure — thin `ProcessModule` facade delegates to:
- `ProcessRuntime` — internal coordinator (NOT a global singleton)
- `ProcessStateMachine` — states: Idle → Preparing → Ready → Processing → Paused/Stopping/Error/EmergencyStop
- `ProcessDeviceCoordinator` — unified motion/laser/IO entry point
- `ProcessToolpathService` → `ProcessToolMatcher` → `ProcessJobPlan` (contour ordering/sorting now lives in the core CAM layer model — `LayerContainer` sort strategy + `core/algorithms/cam/contour_order_planner` — not a Process-side sorter)
- `ProcessInstructionPlanner` → controller-neutral `ProcessCommandBuffer` → `IControllerTranslator` → ACS/GTN/PureSimulation adapter
- `ProcessWorkflowService` + `ProcessExecutionService` + `ProcessNodeExecutorRegistry`

Controller adapters (`PureSimulation`, `SimulatorCMHP`, ACS, GTN) are behind `IMotionController`. Vendor SDK headers (ACSC.h, gts.h) must NEVER appear in public interfaces — they stay in option-gated private `.cpp` files.

## `.lcnc` Project Package

Zip archive (currently PowerShell-based, planned replacement with QuaZip) containing:
- `project.toml` — manifest, version, metadata (`formatVersion` 2)
- `workpiece.xbf` — OCC/XCAF binary snapshot of the workpiece geometry (v1 used `project.xbf`)
- `cam_toolpath.toml` + `cam_toolpath_points.bin` — project-core CAM data (layers, contours, signature tables, dense sampled points)

The machine model is **not** in the package (independent reference asset). Save/load is one transaction in `core`: `LcncProjectManager::saveProject`/`openProject` → `LcncProjectPackage` (geometry) + `lcnc::cam::saveCamToolpath`/`loadCamToolpath` (CAM data). Modules never read/write the package.

## Build (this environment)

The `build/` dir uses the **Ninja** generator and needs the MSVC env. The `-- /m /nologo` MSBuild flags in older docs do **not** work here. Build via a shell that has run `vcvars64.bat` (VS 18 Insiders), e.g. a `.bat` that `call`s vcvars then `cmake --build build --config Debug -j 16`. `LNK1168` on link = `LaserCNC.exe` is still running; kill it and relink.

## Pre-Commit Verification

1. Build passes (see Build section above — Ninja + vcvars, not `/m /nologo`)
2. Layer check: no `core/**` includes `view/modules/app`; no `view/**` includes `modules/app`
3. No legacy API usage: grep for `projectDocument\|workspaceGuiDocument\|ensureProjectDocument`
4. Process module: no OCC includes (grep for `TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF` in `src/modules/process/`)
