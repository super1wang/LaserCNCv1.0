[CmdletBinding()]
param([Parameter(Mandatory = $true)][string]$Root)

$ErrorActionPreference = 'Stop'
$rootPath = (Resolve-Path -LiteralPath $Root).Path
$srcPath = Join-Path $rootPath 'src'
$violations = [System.Collections.Generic.List[string]]::new()

function Find-ForbiddenInclude([string]$Path, [string]$Pattern, [string]$Rule) {
    Get-ChildItem -LiteralPath $Path -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
        foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern $Pattern)) {
            $violations.Add("${Rule}: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
        }
    }
}

Find-ForbiddenInclude (Join-Path $srcPath 'core') '#\s*include\s*[<"](?:view|modules|app)/' 'core may not depend on view/modules/app'
Find-ForbiddenInclude (Join-Path $srcPath 'view') '#\s*include\s*[<"](?:modules|app)/' 'view may not depend on modules/app'
Find-ForbiddenInclude (Join-Path $srcPath 'modules/process') '#\s*include\s*[<"](?:TopoDS|AIS_|gp_|Geom_|BRep|XCAF)' 'Process may not include OCC types'
Find-ForbiddenInclude (Join-Path $srcPath 'core/algorithms') '#\s*include\s*[<"](?:QWidget|QAction|QDialog|core/document/lcnc_document|view/gui_document|core/kernel/kernel)' 'core algorithms must remain independent of UI, documents, and Kernel'

# CAM is the sole toolpath producer.  Cross-module contracts intentionally
# expose detached catalog/execution snapshots only: neither Process nor the
# offline sandbox may ask CAM to solve/reorder/slice a path or write layer data.
$toolpathProviderContract = Join-Path $srcPath 'modules/cam/i_cam_toolpath_provider.h'
foreach ($match in (Select-String -LiteralPath $toolpathProviderContract -Pattern '\bsolveToolpathForOrder\s*\(|\bexportToolpathSnapshotForOrder\s*\(')) {
    $violations.Add("CAM toolpath consumer contract must be read-only: ${toolpathProviderContract}:$($match.LineNumber): $($match.Line.Trim())")
}
$layerProviderContract = Join-Path $srcPath 'modules/cam/i_cam_layer_provider.h'
foreach ($match in (Select-String -LiteralPath $layerProviderContract -Pattern '\bsetLayer|\bsetManual|\bappendToManual|\bremoveFromManual|\bclearManual|\bsetSortStrategy|\bsetLastAuto')) {
    $violations.Add("CAM layer consumer contract must be read-only: ${layerProviderContract}:$($match.LineNumber): $($match.Line.Trim())")
}
foreach ($consumerPath in @(
        (Join-Path $srcPath 'modules/process'),
        (Join-Path $srcPath 'modules/simulation'))) {
    Get-ChildItem -LiteralPath $consumerPath -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
        foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\b(?:solveToolpathForOrder|exportToolpathSnapshotForOrder|toolpathRef)\s*\(')) {
            $violations.Add("Process/simulation may only consume committed CAM toolpath data: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
        }
    }
}
Get-ChildItem -LiteralPath (Join-Path $srcPath 'core/algorithms/cad') -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern 'QString\s*\*|catch\s*\(\s*const\s+Standard_Failure')) {
        $violations.Add("CAD algorithms must propagate typed/OCC failures: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}
Get-ChildItem -LiteralPath (Join-Path $srcPath 'modules/process/device') -Recurse -File |
    Where-Object { $_.Extension -in '.h', '.hpp' } |
    ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '#\s*include\s*[<"](?:modules/process/system/)?(?:MessageModule|process_log_compat)\.h')) {
        $violations.Add("Process device public headers may not expose compatibility logging: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

Get-ChildItem -LiteralPath $srcPath -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\b(projectDocument|workspaceGuiDocument|ensureProjectDocument|sourceDocument)\s*\(')) {
        $violations.Add("retired document API: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

Get-ChildItem -LiteralPath (Join-Path $srcPath 'modules/process') -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\bProcessSettingsService::current\s*\(')) {
        $violations.Add("Process must use injected settings: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\b(?:MessageModule|process_log_compat|LegacyProcessMotionService|LegacyProcessIoService)\b')) {
        $violations.Add("retired Process compatibility surface: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

# Vendor pointers and the defensive SDK lease are runtime-private.  Process
# services, workflows, cutting code and UI must use typed executor operations.
Get-ChildItem -LiteralPath (Join-Path $srcPath 'modules/process') -Recurse -File -Include *.h,*.hpp,*.cpp |
    Where-Object { $_.FullName -notlike '*\runtime\process_device_runtime.cpp' -and
                   $_.FullName -notlike '*\runtime\process_device_runtime.h' -and
                   $_.FullName -notlike '*\runtime\process_device_session.cpp' -and
                   $_.FullName -notlike '*\runtime\process_device_configuration.cpp' -and
                   $_.FullName -notlike '*\device\laser\ld_factory.cpp' -and
                   $_.FullName -notlike '*\device\laser\ld_factory.h' } |
    ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -CaseSensitive -Pattern '\b(?:motionControl|laserDevice|lockDeviceAccess)\s*\(')) {
        $violations.Add("Process raw device access escaped typed runtime: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

# Runtime/sink/cutting contracts publish only narrow callbacks; they must not
# retain the Process facade implementation to reach UI state.
$processModuleContractFiles = @(
    'modules/process/runtime/i_motion_command_sink.h',
    'modules/process/runtime/pure_simulation_sink.h',
    'modules/process/cutting/pure_simulation_toolpath_ticker.h',
    'modules/process/cutting/normal_cutting_manager.h'
)
foreach ($relativePath in $processModuleContractFiles) {
    $contractPath = Join-Path $srcPath $relativePath
    foreach ($match in (Select-String -LiteralPath $contractPath -Pattern '\bProcessModule\s*\*')) {
        $violations.Add("Process runtime/sink contract may not expose ProcessModule*: ${contractPath}:$($match.LineNumber): $($match.Line.Trim())")
    }
}

$processModule = Join-Path $srcPath 'modules/process/process_module.cpp'
foreach ($match in (Select-String -LiteralPath $processModule -Pattern '\b(?:connectDevices|disconnectDevices|pollStatus|pollPeripheralStatus)\s*\(')) {
    $violations.Add("ProcessModule must delegate connection/status device operations: $($processModule):$($match.LineNumber): $($match.Line.Trim())")
}
foreach ($match in (Select-String -LiteralPath $processModule -Pattern '\bProcessFlowStore::')) {
    $violations.Add("ProcessModule must delegate workflow persistence to ProcessWorkflowService: $($processModule):$($match.LineNumber): $($match.Line.Trim())")
}

$processPath = Join-Path $srcPath 'modules/process'
Get-ChildItem -LiteralPath $processPath -Recurse -Directory | ForEach-Object {
    if ($_.Name -cmatch '[A-Z]') {
        $violations.Add("Process directory must use snake_case: $($_.FullName)")
    }
}
Get-ChildItem -LiteralPath $processPath -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    if ($_.BaseName -cmatch '[A-Z]') {
        $violations.Add("Process source file must use snake_case: $($_.FullName)")
    }
}

# Application-owned persistence is intentionally v4-only.  Keep historical
# readers/writers out of the normal product tree so an old file can never be
# silently repaired or interpreted as current data.
Get-ChildItem -LiteralPath $srcPath -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\b(loadForMigration|migrateLegacyProcessCuttingPlan|LegacyOuterSurface)\b|CamConfig\.json|projectXcafPath')) {
        $violations.Add("retired application format compatibility: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

Get-ChildItem -LiteralPath (Join-Path $srcPath 'modules/process/workflow') -Recurse -File -Include *.h,*.hpp,*.cpp | ForEach-Object {
    foreach ($match in (Select-String -LiteralPath $_.FullName -Pattern '\bProcessNodeType::(?:IO|Monitor|Axis|AxesMove|Cutting)\b|\bwriteLegacyNode\b')) {
        $violations.Add("retired workflow compatibility API: $($_.FullName):$($match.LineNumber): $($match.Line.Trim())")
    }
}

$cmake = Get-Content -LiteralPath (Join-Path $rootPath 'CMakeLists.txt') -Raw
Get-ChildItem -LiteralPath $srcPath -Recurse -File -Filter *.cpp | ForEach-Object {
    $relative = $_.FullName.Substring($rootPath.Length + 1).Replace('\', '/')
    if (-not $cmake.Contains($relative)) { $violations.Add("orphan source not listed in CMakeLists.txt: $relative") }
}

if ($violations.Count -gt 0) { $violations | ForEach-Object { Write-Error $_ }; exit 1 }
Write-Host 'Architecture checks passed.'
