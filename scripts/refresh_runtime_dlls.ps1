# 重新生成 3rd/runtime/bin_<config>/ 下的运行时 DLL 快照（Debug 或 Release）。
#
# 何时运行：升级 Qt / OCC / SARibbon / boost / ACS SDK 后，或者新增了一个
# 需要随 exe 一起部署的 DLL 后。运行后必须把 3rd/runtime/ 的差异提交到 git。
#
# 工作原理：
#   1. 以 build-cmake/ 的当前配置重新链接 LaserCNC.exe。
#   2. 用 windeployqt 让 Qt 自己分析 x64/<variant>/<Config>/LaserCNC.exe 的 import，把它需要的 Qt
#      DLL + 插件子目录 (platforms/, imageformats/, ...) 拷贝到该运行目录。
#   3. 从 CMake 当前选中的 OCC 根拷贝配置匹配的 TK*.dll，并补齐 TBB 和 zlib 运行时。
#   4. SARibbon、boost、ACSCL_x64.dll。
#   5. 把 x64/<variant>/<Config>/ 里所有 .dll + 插件子目录原样镜像到
#      3rd/runtime/bin_<config>/，覆盖旧快照。

param(
    [ValidateSet("debug","release")]
    [string]$Config = "debug",
    [string]$BuildDir = "",
    [string]$OcctRoot = "",
    [string]$AcsDllDebug   = "F:\wangchao\Axis4-3D\trunk\bin\Debug\ACSCL_x64.dll",
    [string]$AcsDllRelease = "F:\wangchao\Axis4-3D\trunk\bin\Release\ACSCL_x64.dll",
    [string]$TbbBinDir     = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path "$PSScriptRoot\.."
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot "build-cmake"
}
$CmakeConfig = (Get-Culture).TextInfo.ToTitleCase($Config)
$Target   = Join-Path $RepoRoot "3rd\runtime\bin_$Config"
$ZlibDll = Join-Path $RepoRoot "3rd\runtime\common\zlib1.dll"
$IsDebug  = ($Config -eq "debug")
$AcsDll   = if ($IsDebug) { $AcsDllDebug } else { $AcsDllRelease }

if (-not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))) {
    throw "Build directory not configured: $BuildDir. Configure the ACS+GTN preset first (cmake --preset acs-gtn)."
}

$cache = Get-Content (Join-Path $BuildDir "CMakeCache.txt")
$runtimeVariant = ($cache | Select-String '^LCNC_RUNTIME_VARIANT:.*=(.+)$').Matches.Groups[1].Value
if ([string]::IsNullOrWhiteSpace($runtimeVariant)) {
    throw "LCNC_RUNTIME_VARIANT is missing from $BuildDir\CMakeCache.txt"
}
$RuntimeDir = Join-Path $RepoRoot ("x64\" + $runtimeVariant + "\" + $CmakeConfig)
if ([string]::IsNullOrWhiteSpace($OcctRoot)) {
    $OcctRoot = ($cache | Select-String '^LCNC_OCCT_ROOT:.*=(.+)$').Matches.Groups[1].Value
}
if ([string]::IsNullOrWhiteSpace($OcctRoot) -or -not (Test-Path -LiteralPath $OcctRoot)) {
    throw "LCNC_OCCT_ROOT is unavailable: $OcctRoot"
}
if ([string]::IsNullOrWhiteSpace($TbbBinDir)) {
    $TbbBinDir = ($cache | Select-String '^LCNC_TBB_BIN_DIR:.*=(.+)$').Matches.Groups[1].Value
}

Write-Host "== Step 1: build LaserCNC ($Config) =="
& cmake --build $BuildDir --config $CmakeConfig --target LaserCNC
if ($LASTEXITCODE -ne 0) { throw "cmake --build failed" }
if (-not (Test-Path (Join-Path $RuntimeDir "LaserCNC.exe"))) {
    throw "Runtime executable not found: $RuntimeDir\LaserCNC.exe"
}

Write-Host "== Step 2: run windeployqt =="
$qt6Dir = ($cache | Select-String '^Qt6_DIR:.*=(.+)$').Matches.Groups[1].Value
$qtBin = Join-Path $qt6Dir "..\..\..\bin"
$wd = Join-Path $qtBin "windeployqt.exe"
if (-not (Test-Path $wd)) { throw "windeployqt.exe not found at $wd" }
$wdArgs = @("--no-translations","--no-system-d3d-compiler","--no-opengl-sw")
if (-not $IsDebug) { $wdArgs += "--release" }
& $wd @wdArgs (Join-Path $RuntimeDir "LaserCNC.exe")
if ($LASTEXITCODE -ne 0) { throw "windeployqt failed" }

Write-Host "== Step 3: deploy OpenCASCADE $CmakeConfig runtime into $RuntimeDir =="
$runtimeManifest = Join-Path $BuildDir "lcnc_runtime_dlls_$CmakeConfig.txt"
if (-not (Test-Path -LiteralPath $runtimeManifest)) {
    throw "CMake runtime manifest not found: $runtimeManifest"
}
$occtRootResolved = (Resolve-Path -LiteralPath $OcctRoot).Path
$occtDlls = Get-Content -LiteralPath $runtimeManifest |
    ForEach-Object {
        if ($_) {
            $candidate = [System.IO.Path]::GetFullPath($_)
            if ([System.IO.Path]::GetFileName($candidate) -like "TK*.dll" -and
                $candidate.StartsWith($occtRootResolved, [System.StringComparison]::OrdinalIgnoreCase)) {
                $candidate
            }
        }
    }
if ($occtDlls.Count -eq 0) {
    throw "OpenCASCADE runtime closure is missing from $runtimeManifest"
}
Get-ChildItem -LiteralPath $RuntimeDir -Filter "TK*.dll" -File | ForEach-Object {
    [System.IO.File]::Delete($_.FullName)
}
foreach ($occtDll in $occtDlls) {
    Copy-Item -LiteralPath $occtDll -Destination $RuntimeDir -Force
}

Write-Host "== Step 3a: ensure ACSCL_x64.dll present in $RuntimeDir =="
if (Test-Path $AcsDll) {
    Copy-Item -Force $AcsDll $RuntimeDir
} else {
    Write-Warning "ACSCL_x64.dll not found at $AcsDll — skipping"
}

Write-Host "== Step 3b: ensure OpenCASCADE TBB runtime present in $RuntimeDir =="
$tbbNames = if ($IsDebug) { @("tbb12_debug.dll", "tbbmalloc_debug.dll") } else { @("tbb12.dll", "tbbmalloc.dll") }
foreach ($tbbName in $tbbNames) {
    $tbbSource = Join-Path $TbbBinDir $tbbName
    if (-not (Test-Path $tbbSource)) {
        throw "OpenCASCADE runtime dependency not found: $tbbSource. Set -TbbBinDir to the matching TBB bin directory."
    }
    Copy-Item -Force $tbbSource $RuntimeDir
}

Write-Host "== Step 3c: ensure zlib runtime present in $RuntimeDir =="
if (-not (Test-Path $ZlibDll)) {
    throw "zlib runtime backup not found: $ZlibDll"
}
Copy-Item -Force $ZlibDll $RuntimeDir

Write-Host "== Step 4: mirror DLLs and Qt plugin subdirs into 3rd/runtime/bin_$Config =="
if (Test-Path $Target) { Remove-Item -Recurse -Force $Target }
New-Item -ItemType Directory -Force -Path $Target | Out-Null

Get-ChildItem -Path $RuntimeDir -Filter *.dll -File | ForEach-Object {
    Copy-Item $_.FullName $Target
}
foreach ($sub in @("generic","iconengines","imageformats","networkinformation","platforms","styles","tls")) {
    $src = Join-Path $RuntimeDir $sub
    if (Test-Path $src) { Copy-Item -Recurse -Force $src $Target }
}

$count = (Get-ChildItem -Recurse -File $Target).Count
$size  = "{0:N1} MB" -f ((Get-ChildItem -Recurse -File $Target | Measure-Object Length -Sum).Sum / 1MB)
Write-Host "Done: $count files, $size, in $Target"
