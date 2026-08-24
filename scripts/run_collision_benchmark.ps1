param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot),
    [string]$BuildDir = "",
    [string]$CoalRoot = "",
    [string]$IndexPath = ""
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path -LiteralPath $Root).Path
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    # Keep every Ninja generator tree under the BUILD.md-authorized
    # build-cmake root while isolating benchmark cache options from daily builds.
    # 中文翻译：碰撞基准生成树位于 build-cmake 下，并与日常缓存隔离。
    $BuildDir = Join-Path $Root "build-cmake/collision-benchmark"
}
if ([string]::IsNullOrWhiteSpace($CoalRoot)) {
    $CoalRoot = Join-Path $Root "x64/deps/install/coal-release"
}
if (-not (Test-Path -LiteralPath $CoalRoot -PathType Container)) {
    throw "Coal install prefix does not exist: $CoalRoot"
}

$configure = @(
    "-S", $Root,
    "-B", $BuildDir,
    "-G", "Ninja Multi-Config",
    "-DLCNC_RUNTIME_VARIANT=ninja-collision",
    "-DLCNC_WITH_ACS=OFF",
    "-DLCNC_WITH_GTN=OFF",
    "-DLCNC_ENABLE_COAL_COLLISION_BENCHMARK=ON",
    "-DLCNC_ENABLE_FCL_COLLISION_BENCHMARK=OFF",
    "-DLCNC_COAL_ROOT=$CoalRoot"
)
if (-not [string]::IsNullOrWhiteSpace($IndexPath)) {
    $IndexPath = (Resolve-Path -LiteralPath $IndexPath).Path
    $configure += "-DLCNC_COLLISION_BENCHMARK_INDEX=$IndexPath"
} else {
    # Clear a value retained by a previous configure so the default benchmark
    # is reproducible and cannot silently consume a stale local index.
    $configure += "-DLCNC_COLLISION_BENCHMARK_INDEX="
}

& cmake @configure
if ($LASTEXITCODE -ne 0) { throw "Collision benchmark configure failed" }
& cmake --build $BuildDir --config Release --target lcnc_coal_machine_collision_benchmark --parallel 16
if ($LASTEXITCODE -ne 0) { throw "Collision benchmark build failed" }
& ctest --test-dir $BuildDir -C Release -R "^lcnc_coal_machine_collision_benchmark$" --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Collision benchmark production gates failed" }
