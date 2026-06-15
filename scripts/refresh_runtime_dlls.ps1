# 重新生成 3rd/runtime/bin_<config>/ 下的运行时 DLL 快照（Debug 或 Release）。
#
# 何时运行：升级 Qt / OCC / SARibbon / boost / ACS SDK 后，或者新增了一个
# 需要随 exe 一起部署的 DLL 后。运行后必须把 3rd/runtime/ 的差异提交到 git。
#
# 工作原理：
#   1. 把 build_<config>/ 下当前的 LaserCNC.exe 重新链接（确保是最新的）。
#   2. 用 windeployqt 让 Qt 自己分析 LaserCNC.exe 的 import，把它需要的 Qt
#      DLL + 插件子目录 (platforms/, imageformats/, ...) 拷贝到 build_<config>/。
#   3. 把 OCC 的 TK*.dll、OCC 的 3rd-party (FreeImage / freetype / ffmpeg /
#      openvr / tbb / jemalloc) 拷过来。OCC 在这台机上只有 release 版，
#      debug 配置仍然链 release OCC（OCC 一向如此分发）。
#   4. SARibbon、boost、ACSCL_x64.dll。
#   5. 把 build_<config>/ 里所有 .dll + 插件子目录原样镜像到
#      3rd/runtime/bin_<config>/，覆盖旧快照。

param(
    [ValidateSet("debug","release")]
    [string]$Config = "debug",
    [string]$AcsDllDebug   = "F:\wangchao\Axis4-3D\trunk\bin\Debug\ACSCL_x64.dll",
    [string]$AcsDllRelease = "F:\wangchao\Axis4-3D\trunk\bin\Release\ACSCL_x64.dll"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path "$PSScriptRoot\.."
$BuildDir = Join-Path $RepoRoot "build_$Config"
$Target   = Join-Path $RepoRoot "3rd\runtime\bin_$Config"
$IsDebug  = ($Config -eq "debug")
$AcsDll   = if ($IsDebug) { $AcsDllDebug } else { $AcsDllRelease }

if (-not (Test-Path $BuildDir)) {
    throw "Build directory not found: $BuildDir. Configure and build that variant first (e.g. cmake -G Ninja -B $BuildDir -DCMAKE_BUILD_TYPE=$($Config.Substring(0,1).ToUpper() + $Config.Substring(1)))."
}

Write-Host "== Step 1: build LaserCNC ($Config) =="
& cmake --build $BuildDir
if ($LASTEXITCODE -ne 0) { throw "cmake --build failed" }

Write-Host "== Step 2: run windeployqt =="
$cache = Get-Content (Join-Path $BuildDir "CMakeCache.txt")
$qt6Dir = ($cache | Select-String '^Qt6_DIR:.*=(.+)$').Matches.Groups[1].Value
$qtBin = Join-Path $qt6Dir "..\..\..\bin"
$wd = Join-Path $qtBin "windeployqt.exe"
if (-not (Test-Path $wd)) { throw "windeployqt.exe not found at $wd" }
$wdArgs = @("--no-translations","--no-system-d3d-compiler","--no-opengl-sw")
if (-not $IsDebug) { $wdArgs += "--release" }
& $wd @wdArgs (Join-Path $BuildDir "LaserCNC.exe")
if ($LASTEXITCODE -ne 0) { throw "windeployqt failed" }

Write-Host "== Step 3: ensure ACSCL_x64.dll present in $BuildDir =="
if (Test-Path $AcsDll) {
    Copy-Item -Force $AcsDll $BuildDir
} else {
    Write-Warning "ACSCL_x64.dll not found at $AcsDll — skipping"
}

Write-Host "== Step 4: mirror DLLs and Qt plugin subdirs into 3rd/runtime/bin_$Config =="
if (Test-Path $Target) { Remove-Item -Recurse -Force $Target }
New-Item -ItemType Directory -Force -Path $Target | Out-Null

Get-ChildItem -Path $BuildDir -Filter *.dll -File | ForEach-Object {
    Copy-Item $_.FullName $Target
}
foreach ($sub in @("generic","iconengines","imageformats","networkinformation","platforms","styles","tls")) {
    $src = Join-Path $BuildDir $sub
    if (Test-Path $src) { Copy-Item -Recurse -Force $src $Target }
}

$count = (Get-ChildItem -Recurse -File $Target).Count
$size  = "{0:N1} MB" -f ((Get-ChildItem -Recurse -File $Target | Measure-Object Length -Sum).Sum / 1MB)
Write-Host "Done: $count files, $size, in $Target"
