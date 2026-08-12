# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Build

以 `BUILD.md` 为唯一构建约定。必须记住并保持两条路线完全隔离：

- CMake/Ninja：`build-cmake/`，使用 `acs-gtn` 与 `acs-gtn-debug` 等 preset。
- Visual Studio/MSBuild：`build-vs/`，生成并打开
  `build-vs/LaserCNC.sln`，使用 `vs-acs-gtn` preset 或直接 MSBuild。
- 禁止使用旧 `build/`，禁止让两种生成器共享任何生成树。
- 两条路线的 `LaserCNC.exe` 和运行依赖按生成器、功能开关和配置输出：日常 Ninja 为
  `x64/ninja/Debug` 或 `x64/ninja/Release`，VS/MSBuild 为 `x64/vs/Debug` 或
  `x64/vs/Release`，ASan 与 all-off 分别为 `x64/ninja-asan/<Config>` 和
  `x64/ninja-all-off/<Config>`；中间产物留在各自生成树。其它 Ninja 开关也使用各自的
  `ninja-<variant>/<Config>` 目录。
- `/m`、`/nologo` 只传给 MSBuild，绝不传给 Ninja。两条路线不得并发构建。

日常 CMake/Ninja：

```powershell
cmd /c "call \"E:\vs2022IDE\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --preset acs-gtn && cmake --build --preset acs-gtn-debug --parallel 16"
```

Visual Studio/MSBuild：

```powershell
cmake --preset vs-acs-gtn
cmake --build --preset vs-acs-gtn-debug --parallel 16
```

- Single CMake target: `LaserCNC` (WIN32 executable).
- Requires CMake 3.20+, MSVC 2022 x64, C++17.
- Qt 6.9.1, OpenCASCADE 7.9.0, SARibbon — paths configured via CMake cache variables (`LCNC_QT6_ROOT`, `LCNC_OCCT_ROOT`, `LCNC_SARIBBON_ROOT`).
- Vendored 3rd-party libs in `3rd/`: spdlog (logging), toml11 (config).
- OCC and SARibbon DLLs are copied to the output directory via POST_BUILD commands.
- ACS and GTN are ON in the daily `acs-gtn` preset. `all-off`, `acs`, `gtn`, and `asan` remain explicit verification presets. A disabled SDK must not leak headers or link libraries into Process.
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
Modules that start `TaskManager` jobs must retain their task ids, request cancellation in `stop()`, and wait before destroying borrowed runtime state. A timeout must safe-stop and retain SDK-owned objects rather than freeing them under an active call.
`init()`, `start()` and `stop()` must not leak either standard or unknown exceptions. `stop()` must be idempotent because ModuleRegistry uses it for init/start rollback as well as normal shutdown.
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
- `ProcessDeviceCoordinator` — a shared recursive device lease owned by `Service`; every vendor SDK read/write/connect/disconnect path must acquire it.

The coordinator is currently a serial lease, not yet a dedicated device-thread command queue. Do not add direct SDK access paths or retain a `MotionControl*` across a worker boundary. Track the remaining safety work in `todo.md`.

Controller adapters (`PureSimulation`, `SimulatorCMHP`, ACS, GTN) are behind `IMotionController`. `SimulatorCMHP` belongs to ACS: the ACS-gated controller header/implementation opens ACS Simulator and loads the deployed `Simulator.prg`; all-off uses the SDK-free `Simulator` identity. Vendor SDK types must never appear in public facades or DTOs. PureSimulation is explicit-only, defaults OFF when ACS or GTN is compiled, and must never be used as a failed-controller fallback.
Controller status polling runs at 150 ms on its own single-thread pool, safety IO monitoring at 500 ms on a separate pool, and serial peripherals at 2 s on a low-frequency pool. `QSerialPort` owns a dedicated IO thread; no hardware wait may run on the GUI thread.
Device public headers must not include `MessageModule` or legacy logging headers; implementation files own any temporary compatibility include.
Message notifications and compatibility logging must write through `lcnc::Logger`; `LogModule` must not be reintroduced.
ProcessModule code must use its injected settings service, not `ProcessSettingsService::current()`.
`ToolFactory` lookup must be side-effect free: never synthesize an empty fallback tool, and replace existing entries when loading the same index.
`ProcessRuntimeConfiguration` is owned by ProcessModule and borrowed by Service/controllers; ACS/GTN axis selection, extension-axis checks and simulation selection must use it. Normalize and deduplicate axis names at this boundary, and never pass the `BASE` pseudo-axis to a device adapter; do not reintroduce `DT` static runtime state and keep `lcnc_process_runtime_configuration_test` green.
Do not reintroduce synchronous single-controller connection methods or `ProcessLayerJob::order`; UI commands must use asynchronous all-device operations.
Keep pure axis helpers in `runtime/process_axis_utilities`; they must not access UI or device SDKs.
Use `Service::shutdownDevices()` for final device teardown; do not duplicate or reorder laser/motion shutdown in callers.

## `.lcnc` Project Package

QuaZip archive (format v3) containing:
- `project.toml` — manifest, version, metadata
- `workpiece.xbf` — Workpiece/Auxiliary and CAM-kind XCAF entities
- `cam_toolpath.toml` + `cam_toolpath_points.bin` — CAM metadata and dense points

Save/load goes through `LcncProjectManager` → `LcncProjectPackage`.
Desktop loading accepts only v4 and requires `tools.toml`. `lcnc_project_upgrade` alone may call `loadForMigration()` for v1/v2/v3 and legacy cutting-plan input; every migration save writes v4 and must use a distinct output path. It preserves an embedded snapshot, while a historical package without one must be supplied as `--tools <tools.toml>`. Project tools are restored from `tools.toml`, not resolved only by global names.
Machine-fingerprint mismatch is a project-session safety gate: core emits the mismatch event, UI warns the operator, and Process must refuse real machining while still allowing PureSimulation.
`Service`, `MotionControl`, `LaserDevice`, `LDFactory` and `ProcessParameterRegistry` receive `ProcessSettingsService` by constructor injection only. Initialize settings before constructing Service; SimulatorCMHP, ACS, GTN and laser adapters must forward that dependency. The settings `current()` singleton is removed.
Archive saves must retain the existing package until the staging archive is complete and atomically replaced; do not reintroduce delete-then-write behavior.
`lcnc_project_package_test` must remain green: it covers v4 snapshot round-trip, required-resource rejection, staging archive replacement, preservation of the existing package after a failed save, and v1/v2/v3 structural fixtures through the actual `lcnc_project_upgrade.exe`. Destroy the staging `QTemporaryFile` before QuaZip writes the archive, otherwise Windows can reject `MoveFileExW` with sharing violation 32.
`lcnc_task_manager_test` must remain green for task-lifecycle changes: it covers cooperative abort, finite wait timeout, and exception-to-failure conversion.
The `asan` preset must deploy `clang_rt.asan_dynamic-x86_64.dll` and the selected OCCT TBB DLL to every executable/test target; do not rely on developer-machine PATH or manual deployment for CTest.
`collect_runtime_baseline.ps1` writes its CSV even after an early process exit and supports optional private-byte/handle-growth gates. Separate startup initialization from stable-window trends; Application Verifier is a separate interactive-environment gate, not a substitute for ASan.

## Pre-Commit Verification

1. Build passes using the command in the Build section.
2. Layer check: no `core/**` includes `view/modules/app`; no `view/**` includes `modules/app`
3. No legacy API usage: grep for `projectDocument\|workspaceGuiDocument\|ensureProjectDocument\|sourceDocument`
4. Process module: no OCC includes (grep for `TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF` in `src/modules/process/`)
5. CTest architecture gate: `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
5. Memory-sensitive changes additionally build `cmake --preset asan && cmake --build --preset asan`; do not enable Application Verifier without explicit user/test-run authority.
