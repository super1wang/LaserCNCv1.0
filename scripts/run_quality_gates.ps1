[CmdletBinding()]
param(
    [string]$Preset = 'acs-gtn',
    [string]$BuildPreset = 'acs-gtn-debug',
    [string]$BuildDirectory = 'build-cmake',
    [int]$Parallel = 16,
    [switch]$SkipConfigure,
    [switch]$SkipBuild,
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Push-Location $root
try {
    git diff --check
    if ($LASTEXITCODE -ne 0) { throw 'git diff --check failed.' }

    & (Join-Path $root 'scripts/check_architecture.ps1') -Root $root
    if ($LASTEXITCODE -ne 0) { throw 'Architecture checks failed.' }

    if (-not $SkipConfigure) {
        cmake --preset $Preset
        if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for preset '$Preset'." }
    }
    if (-not $SkipBuild) {
        cmake --build --preset $BuildPreset --parallel $Parallel
        if ($LASTEXITCODE -ne 0) { throw "Build failed for preset '$BuildPreset'." }
    }
    if (-not $SkipTests) {
        ctest --test-dir $BuildDirectory --build-config Debug --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw 'CTest failed.' }
    }
}
finally {
    Pop-Location
}
