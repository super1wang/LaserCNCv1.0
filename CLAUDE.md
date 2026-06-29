# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```powershell
cmake --build cmake-build-sdkcheck --config Debug -j 16
```

- **Use the `cmake-build-sdkcheck/` dir (Visual Studio 2022 generator, MSVC 14.4x)** — it matches the toolchain Qt 6.9.1 `msvc2022_64` and OpenCASCADE were built with. No vcvars needed (the VS/MSBuild generator brings its own env). Output: `cmake-build-sdkcheck/Debug/LaserCNC.exe`.
- ⚠️ **Do NOT build/run the `build/` dir.** It is configured with the **VS18 Insiders preview compiler (MSVC 14.51)**, which is **ABI-incompatible** with the VS2022-built Qt/OCC libraries — a binary built there **crashes at startup** with a garbage stack (fake `~TaskManager` → `Qt6Cored.dll`). That is a toolchain mismatch, not a code bug.
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

**One unified project document (workpiece + CAM), owned by `core/`, saved/loaded as one unit.** `LcncProjectManager` owns a single project `LcncDocument` (`workpieceDocument()`; `camDocument()` is an alias of it) that holds **both** the workpiece source geometry (`EntityKind::Workpiece`/`Auxiliary`) **and** the CAM contour wires (`EntityKind::Cam`) in one XCAF tree. A standalone STEP import adds `Workpiece` entities; `generateToolpath` writes each contour wire as a `Cam` entity and records its label in `LaserContour.xcafEntry`. The dense CAM runtime — contours, toolpaths, layers, lead-ins, **process parameters**, and **project-level generation params** (`CamDataManager::GenerationParams`) — lives in `lcnc::cam::CamDataManager` (`core/project/cam/`), also owned by `LcncProjectManager`. There is **no** separate empty CAM document anymore.

**The machine model is an independent reference asset owned by `core/`, NOT project data.** `lcnc::cam::MachineWorkspace` (now in `core/machine/`) is **owned by the `Kernel`**, holds the machine `LcncDocument` (+ kinematics), is **preloaded at startup** from `CamConfig::machineModelPath` (via `MainWindow`), stays **resident in the view across project switches** (project reset erases only Workpiece + CAM display, never Machine), and is **never written into `.lcnc` / never dirties the project**. `CamModule` borrows it (`Kernel::current().machineWorkspace()`) for load/axis/calibration business logic. `LcncProjectManager` keeps only a non-owning machine-doc *reference* (`attachMachineDocument`) so `GuiDocument::domainForDocument` can route by domain.

**Persistence is a single transactional writer in `core` (`formatVersion` 3).** `LcncProjectManager::saveProject`/`openProject` write/read manifest + `workpiece.xbf` (now containing Workpiece **and** Cam-kind entities) + CAM data (`lcnc::cam::saveCamToolpath`/`loadCamToolpath` in `core/project/cam/cam_toolpath_io.*`) together. After load, `CamModule::onCamDataLoaded` relinks each contour's wire from the XCAF Cam entity by `xcafEntry` (no runtime re-extraction). Modules do **no** project-file IO. The legacy `process_cutting_plan.toml` v1 migration runs in core (`migrateLegacyProcessCuttingPlan`).

**`GuiDocument` domain routing is `EntityKind`-aware.** Because one document carries multiple domains, `rebuildDomain(domain, doc)` registers only the labels whose `EntityKind` maps to that domain (Workpiece→Workpiece+Auxiliary, Cam→Cam, Machine→Machine). CAM contour bodies are displayed by stable `ContourId` (`displayContourBody`), not re-shown via `rebuildDomain(Cam)`.

**CAD / CAM / Process own no project data — they are business logic** over the core data:
- **CAD** edits workpiece geometry through core document APIs.
- **CAM** runs algorithms (extract / discretise / IK), writes results into the core-owned `CamDataManager` (borrowed via `projectManager()->camData()`), manages layers (`addToolpathLayer`/`removeToolpathLayer` → `CamDataManager::addLayer`/`removeLayer`), and keeps only renderers + transient UI/preview state.
- **Process** consumes CAM data read-only via the OCC-free `ToolpathExportSnapshot` DTO.

> Remaining cleanup (not yet done): `MachineKinematics` still physically lives in the (Kernel-owned `MachineWorkspace`'s) machine `LcncDocument` rather than in the workspace object itself; `LcncProjectManager` still exposes `machineDocument()`/`machineDocumentId()` as the view-routing reference accessor; and **per-project tool snapshots are not yet embedded** in `.lcnc` (layers still reference tools by name only, resolved against the global `ToolFactory`/`config.toml` — projects are not yet self-contained for tool params). `Tool` has only `SetFromTable` (no serializer), so embedding tools needs a safe `Tool`↔toml round-trip in the process layer + project-scoped `ToolFactory` registration — deferred because these drive laser/motion on real hardware.

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

Zip archive packed/unpacked via **QuaZip** (`JlCompress::compressDir`/`extractDir` in `lcnc_project_package.cpp`; replaces the old PowerShell `Compress-Archive`). QuaZip is built **static** from `LCNC_QUAZIP_ROOT` (default `E:/GitHub/quazip`) via `add_subdirectory`, using the OCCT-bundled static **zlib 1.2.8** (`LCNC_ZLIB_ROOT`) — no extra runtime DLL, no PowerShell dependency. Contains:
- `project.toml` — manifest, version, metadata (`formatVersion` **3**; v2 = workpiece-only xbf, v1 = pre-split)
- `workpiece.xbf` — OCC/XCAF binary snapshot holding **both** workpiece geometry (`EntityKind::Workpiece`/`Auxiliary`) and CAM contour wires (`EntityKind::Cam`); load routes by stored kind
- `cam_toolpath.toml` + `cam_toolpath_points.bin` — project-core CAM data: layers, contours (incl. `xcafEntry` linking each to its Cam wire label), signature tables, dense sampled points, container sort state, and the project-level `[generation]` params

v2→v3 is forward-compatible: a v2 project (no Cam wires) loads fine; re-saving writes v3. Tool definitions are **not** embedded yet (referenced by name; see Architecture note). The machine model is **not** in the package. Save/load is one transaction in `core`: `LcncProjectManager::saveProject`/`openProject` → `LcncProjectPackage` (geometry) + `lcnc::cam::saveCamToolpath`/`loadCamToolpath` (CAM data). Modules never read/write the package.

## Pre-Commit Verification

1. Build passes (see Build section above — use `cmake-build-sdkcheck`, NOT `build/`)
2. Layer check: no `core/**` includes `view/modules/app`; no `view/**` includes `modules/app`
3. No legacy API usage: grep for `projectDocument\|workspaceGuiDocument\|ensureProjectDocument`
4. Process module: no OCC includes (grep for `TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF` in `src/modules/process/`)
