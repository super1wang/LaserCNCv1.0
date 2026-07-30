# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

`BUILD.md` is authoritative. Preserve both supported routes and never share a
generated tree between them:

- CMake/Ninja uses `build-cmake/` and the `acs-gtn` family of presets.
- Visual Studio/MSBuild uses `build-vs/`, `build-vs/LaserCNC.sln`, and the
  `vs-acs-gtn` family of presets.
- The legacy `build/` tree is forbidden.
- Both routes deploy `LaserCNC.exe` and runtime files only to root
  `x64/Debug` or `x64/Release`; intermediates stay in their generator tree.
- MSBuild flags such as `/m` and `/nologo` must never be passed to Ninja.
  Do not build the two routes concurrently because they share the deployment
  directory.

```powershell
# CMake/Ninja
cmd /c "call \"C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\Tools\VsDevCmd.bat\" -arch=x64 -host_arch=x64 && cmake --preset acs-gtn && cmake --build --preset acs-gtn-debug --parallel 16"

# Visual Studio/MSBuild
cmake --preset vs-acs-gtn
cmake --build --preset vs-acs-gtn-debug --parallel 16
```

- Single CMake target: `LaserCNC` (WIN32 executable).
- Requires CMake 3.20+, MSVC 2022 x64 (19.44+), C++17.
- Qt 6.9.1, OpenCASCADE 7.9.0, SARibbon — paths configured via CMake cache variables (`LCNC_QT6_ROOT`, `LCNC_OCCT_ROOT`, `LCNC_SARIBBON_ROOT`).
- Vendored 3rd-party libs in `3rd/`: spdlog (logging), toml11 (config).
- OCC and SARibbon DLLs are copied to the output directory via POST_BUILD commands.
- Conditional Process device SDKs (ACS, GTN, BDAQ, real laser) are OFF by default; enable with CMake `-D` options (`LCNC_WITH_ACS`, etc.).
- Use `CMakePresets.json` for the verified `debug` (all-off), `acs`, `gtn`, and `asan` variants. A disabled SDK must not leak headers or link libraries into Process.
- If build fails with `LNK1168`, the previous `LaserCNC.exe` is still running — kill it and retry.

## Architecture

### Micro-Kernel + Unified Project Document + Workspace-Bound Views

The application is a single-process, multi-workspace desktop app for 5-axis laser machining (CAD + CAM + Process).

**Kernel** (`src/core/kernel/kernel.h`) is the one global entry point (`Kernel::current()`). It owns `AppSettings`, `LcncProjectManager`, `TaskManager`, `MachineConfigurationService`, `ServiceRegistry`, `EventBus`, and `ModuleRegistry`. `GuiApplication` and `CommandContainer` are injected as raw pointers (owned by `main()` and `MainWindow` respectively).

**Each project workspace owns one unified project document (workpiece + CAM), saved/loaded as one unit.** `LcncProjectManager` manages multiple `ProjectWorkspace` instances. Each workspace owns one `LcncDocument` (`workpieceDocument()` and `camDocument()` alias it) holding both workpiece source geometry (`EntityKind::Workpiece`/`Auxiliary`) and CAM contour wires (`EntityKind::Cam`) in one XCAF tree. Dense CAM runtime data lives in the workspace's `lcnc::cam::CamDataManager`. There is no separate CAM document.

**The machine model is an independent reference asset owned by `core/`, NOT project data.** `lcnc::cam::MachineWorkspace` (now in `core/machine/`) is **owned by the `Kernel`**, holds the machine `LcncDocument` (+ kinematics), is **preloaded at startup** from `CamConfig::machineModelPath` (via `MainWindow`), stays **resident in the view across project switches** (project reset erases only Workpiece + CAM display, never Machine), and is **never written into `.lcnc` / never dirties the project**. `CamModule` borrows it (`Kernel::current().machineWorkspace()`) for load/axis/calibration business logic. `LcncProjectManager` keeps only a non-owning machine-doc *reference* (`attachMachineDocument`) so `GuiDocument::domainForDocument` can route by domain.

**Persistence is a single transactional writer in `core` (`formatVersion` 4).** `LcncProjectManager::saveProject`/`openProject` write/read manifest + `workpiece.xbf` (now containing Workpiece **and** Cam-kind entities) + CAM data (`lcnc::cam::saveCamToolpath`/`loadCamToolpath` in `core/project/cam/cam_toolpath_io.*`) together. After load, `CamModule::onCamDataLoaded` relinks each contour's wire from the XCAF Cam entity by `xcafEntry` (no runtime re-extraction). Modules do **no** project-file IO. Desktop loading accepts only v4; only `lcnc_project_upgrade` may invoke `loadForMigration()` for v1/v2/v3 and the legacy cutting-plan input.

**`GuiDocument` domain routing is `EntityKind`-aware.** Because one document carries multiple domains, `rebuildDomain(domain, doc)` registers only the labels whose `EntityKind` maps to that domain (Workpiece→Workpiece+Auxiliary, Cam→Cam, Machine→Machine). CAM contour bodies are displayed by stable `ContourId` (`displayContourBody`), not re-shown via `rebuildDomain(Cam)`.

**CAD / CAM / Process own no project data — they are business logic** over the core data:
- **CAD** edits workpiece geometry through core document APIs.
- **CAM** runs algorithms (extract / discretise / IK), writes results into the core-owned `CamDataManager` (borrowed via `projectManager()->camData()`), manages layers (`addToolpathLayer`/`removeToolpathLayer` → `CamDataManager::addLayer`/`removeLayer`), and keeps only renderers + transient UI/preview state.
- **Process** consumes CAM data read-only via the OCC-free `ToolpathExportSnapshot` DTO.

### CAM Contour Extraction Strategy

The contour extraction algorithm (`LaserToolpathBuilder::extractContours`) dispatches on `ExtractionStrategy`:

| Strategy | Description |
| -------- | ----------- |
| `Auto` (0) | Machine+posture-driven: derives beam direction in WPC from kinematic `wpcHome` + nominal beam axis (-Z), picks the machining face via `selectMachiningFace`, then extracts all face wires. Falls back to tube classification if no planar face faces the beam. |
| `PlanarFaceWires` (1) | Same beam-aligned face selection as Auto, then extracts all wires of that face (outer boundary + every inner hole). Holes-first ordering (contourType `InnerHole=2` before `OuterBoundary=0`). |
| `TubeClassification` (2) | Existing outer-surface ∩ cross-section pipeline (LegacyOuterWire per face, then FaceClassifier group intersection). Produces `TubeCrossSection=1` contours. |
| `ManualFaceSelection` (3) | User picks faces in 3D view via `CmdSelectMachiningFace`. All wires of the selected faces are extracted. |
| `LegacyOuterWire` (4) | Pre-strategy fallback: `BRepTools::OuterWire` of every face. |

**Key data flow:**

1. `CamModule::generateToolpath` reads `m_extractionStrategy` (persisted in `cam.toml`).
2. Beam direction: `beamDirectionWpc(wpcEntry)` = `kinematics->nominalBeamDirectionMachine().Transformed(wpcHome.Inverted())`.
3. `ContourExtractionParams` carries `strategy` + `machiningBeamDirection` + `selectedMachiningFaces` (manual picks) to the pure-algorithm layer.
4. Each resulting `LaserContour` has `contourType` set (`OuterBoundary=0`, `TubeCrossSection=1`, `InnerHole=2`, `Unknown=3`) and `sourceInfo` for debugging.
5. Lead-in is validated per-contour during generate (`setContourStart`/`setAutomaticContourStart` → `computeLeadInSolution`); invalid lead-ins are rejected with an error dialog.

**Machining faces in the project tree** (`CamModule::MachiningFaceEntry`):

- Auto-captured after Auto/PlanarFaceWires generate (cyan AIS highlight).
- Manually added via `CmdSelectMachiningFace` (yellow AIS highlight).
- Displayed under a "加工面" root node in the project explorer.
- Persisted via deterministic face signature (`computeFaceSignature`: area + centroid + surface type + outer-wire vertex count). On project reload, `rebindMachiningFacesFromRecords` matches stored signatures to the loaded workpiece geometry. Non-matching faces are dropped (logged as WARN).
- Records saved in `cam_toolpath.toml` `[machiningFaces]` alongside the toolpath data.

> Remaining cleanup (not yet done): `MachineKinematics` still physically lives in the Kernel-owned `MachineWorkspace` machine `LcncDocument` rather than in the workspace object itself, and `LcncProjectManager` still exposes `machineDocument()`/`machineDocumentId()` as view-routing reference accessors. Project tool snapshots are already embedded as required `tools.toml` in v4 packages.

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

`SimulatorCMHP` belongs to ACS. In ACS-enabled builds its option-gated controller header/implementation restores the ACS Simulator connect/load/stop/close lifecycle and loads the CMake-deployed `Simulator.prg`; all-off exposes only the SDK-free `Simulator` identity. ACS/GTN types must not leak into public facades or DTOs. PureSimulation is explicit-only, defaults off when ACS or GTN is enabled, and is never a connection-failure fallback.
Public device interfaces must not include `MessageModule` or legacy logging headers.
`MessageModule` notification logging must route directly to `lcnc::Logger`; `LogModule` is removed and must not be reintroduced.
ProcessModule must use its injected settings service rather than `ProcessSettingsService::current()`.

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
Polling is split by transport: controller state uses a 150 ms dedicated single-thread pool, safety IO uses a separate 500 ms pool, and serial peripherals use a 2 s low-frequency pool. `QSerialPort` itself lives on a dedicated IO thread so blocking protocol waits cannot execute on the GUI thread.

Controller adapters (`PureSimulation`, `SimulatorCMHP`, ACS, GTN) are behind `IMotionController`. Vendor SDK headers (ACSC.h, gts.h) must NEVER appear in public interfaces — they stay in option-gated private `.cpp` files.

## `.lcnc` Project Package

Zip archive packed/unpacked via **QuaZip** (`JlCompress::compressDir`/`extractDir` in `lcnc_project_package.cpp`; replaces the old PowerShell `Compress-Archive`). QuaZip is built **static** from `LCNC_QUAZIP_ROOT` (default `E:/GitHub/quazip`) via `add_subdirectory`, using the OCCT-bundled static **zlib 1.2.8** (`LCNC_ZLIB_ROOT`) — no extra runtime DLL, no PowerShell dependency. Contains:
- `project.toml` — manifest, version, metadata (`formatVersion` **3**; v2 = workpiece-only xbf, v1 = pre-split)
- `workpiece.xbf` — OCC/XCAF binary snapshot holding **both** workpiece geometry (`EntityKind::Workpiece`/`Auxiliary`) and CAM contour wires (`EntityKind::Cam`); load routes by stored kind
- `cam_toolpath.toml` + `cam_toolpath_points.bin` — project-core CAM data: layers, contours (incl. `xcafEntry` linking each to its Cam wire label), signature tables, dense sampled points, container sort state, and the project-level `[generation]` params

Desktop only accepts v4 and core rejects a package without `tools.toml`. v1/v2/v3 projects must be copied through `lcnc_project_upgrade <input.lcnc> <output.lcnc> [--tools <tools.toml>]`; the tool is the only caller of `loadForMigration()` and always saves v4. It preserves an embedded snapshot, and requires `--tools` when an old package has none. `tools.toml` embeds project tool parameters and is restored ahead of global names. The machine model is **not** in the package. Save/load is one transaction in `core`: `LcncProjectManager::saveProject`/`openProject` → `LcncProjectPackage` (geometry, CAM and staged extension resources). Modules provide extension data but never own package IO.
Machine fingerprint mismatch is stored in the project session and signalled by core. UI shows a warning; `ProcessModule::validateProcessingEnvironment()` must reject real machining but not PureSimulation until the project is re-saved against the verified machine.
Process `Service` is constructed only after the typed settings repository initializes and receives it by reference. `MotionControl`, `LaserDevice`, `LDFactory`, `ProcessParameterRegistry` and the SimulatorCMHP/ACS/GTN/laser adapters receive and forward the same reference. The `ProcessSettingsService::current()` singleton is removed; do not reintroduce fallback global reads.
`ToolFactory` lookup must not insert a missing entry; configuration reloads replace an existing tool at the same index.
`ProcessRuntimeConfiguration` is ProcessModule-owned and is borrowed by Service/controllers. ACS/GTN standard-axis, extension-axis and simulation decisions must read it; normalize and deduplicate configured axes and filter the `BASE` pseudo-axis before device access. Do not reintroduce the deleted `DT` static runtime state.
`lcnc_project_package_test` verifies v4 package round-trip, required `tools.toml`, preservation of an existing package after a failed save, and v1/v2/v3 structural migration through `lcnc_project_upgrade.exe`; the staging `QTemporaryFile` must be destroyed before QuaZip writes, or Windows may reject the final replacement with Win32 error 32.
`lcnc_task_manager_test` verifies cooperative abort, finite timeout and exception failure conversion; task lifecycle changes must keep it passing.
`lcnc_process_runtime_configuration_test` verifies axis normalization, `BASE` filtering, extension-axis matching, simulation state and permission state; changes to ACS/GTN runtime configuration must keep it passing.
Do not reintroduce synchronous single-controller connection methods or the obsolete `ProcessLayerJob::order` field.
Keep pure Process axis helpers in `runtime/process_axis_utilities`, without UI or device SDK dependencies.
Final device teardown must use `Service::shutdownDevices()` so laser shutdown precedes motion/controller disconnection.
ASan CTest must be self-contained: CMake deploys `clang_rt.asan_dynamic-x86_64.dll` plus the selected OCCT TBB runtime beside every executable target.
Runtime-baseline collection must retain CSV evidence on early exit and may enforce private-memory/handle-growth thresholds. Evaluate startup separately from stable-state drift; Application Verifier remains an independent gate.
Archive writes use a sibling staging zip followed by a Windows atomic replacement; save code must never delete the existing `.lcnc` before the new archive is complete.

## Pre-Commit Verification

1. Build passes using the command in the Build section.
2. Layer check: no `core/**` includes `view/modules/app`; no `view/**` includes `modules/app`
3. No legacy API usage: grep for `projectDocument\|workspaceGuiDocument\|ensureProjectDocument\|sourceDocument`
4. Run the CTest architecture gate: `ctest --test-dir build-cmake --build-config Debug --output-on-failure`
4. Process module: no OCC includes (grep for `TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF` in `src/modules/process/`)
5. Memory-sensitive changes additionally build `cmake --preset asan && cmake --build --preset asan`; do not enable Application Verifier without explicit user/test-run authority.
