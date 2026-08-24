param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot),
    [string]$VcpkgInstalled = "G:/environment/vcpkg-master/vcpkg/installed/x64-windows",
    [string]$VsDevCmd = "E:/vs2022IDE/Common7/Tools/VsDevCmd.bat",
    [string]$Version = "3.0.4",
    [switch]$ForceReconfigure
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path -LiteralPath $Root).Path
$dependencyRoot = Join-Path $Root "x64/deps"
$sourceRoot = Join-Path $dependencyRoot "src"
$sourceDir = Join-Path $sourceRoot "coal-$Version"
$archivePath = Join-Path $dependencyRoot "coal-$Version.zip"

if (-not (Test-Path -LiteralPath $VcpkgInstalled -PathType Container)) {
    throw "vcpkg installed tree does not exist: $VcpkgInstalled"
}
if (-not (Test-Path -LiteralPath $VsDevCmd -PathType Leaf)) {
    throw "Visual Studio developer environment script does not exist: $VsDevCmd"
}
New-Item -ItemType Directory -Force -Path $sourceRoot | Out-Null

if (-not (Test-Path -LiteralPath $sourceDir -PathType Container)) {
    $downloadUrl = "https://github.com/coal-library/coal/archive/refs/tags/v$Version.zip"
    Invoke-WebRequest -Uri $downloadUrl -OutFile $archivePath
    Expand-Archive -LiteralPath $archivePath -DestinationPath $sourceRoot -Force
    if (-not (Test-Path -LiteralPath $sourceDir -PathType Container)) {
        throw "Coal source archive did not contain the expected directory: $sourceDir"
    }
}

foreach ($configuration in @("Release", "Debug")) {
    $suffix = $configuration.ToLowerInvariant()
    $buildDir = Join-Path $dependencyRoot "build/coal-$suffix"
    $installDir = Join-Path $dependencyRoot "install/coal-$suffix"
    if ($ForceReconfigure -and (Test-Path -LiteralPath $buildDir)) {
        $resolvedBuild = (Resolve-Path -LiteralPath $buildDir).Path
        $resolvedDependencies = (Resolve-Path -LiteralPath $dependencyRoot).Path
        if (-not $resolvedBuild.StartsWith($resolvedDependencies,
                                           [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove a build directory outside x64/deps: $resolvedBuild"
        }
        Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
    }
    $command = 'call "{0}" -arch=x64 -host_arch=x64 && cmake -S "{1}" -B "{2}" -G Ninja -DCMAKE_BUILD_TYPE={3} -DCMAKE_INSTALL_PREFIX="{4}" -DCMAKE_PREFIX_PATH="{5}" -DBUILD_TESTING=OFF -DCOAL_HAS_QHULL=OFF -DCOAL_ENABLE_LOGGING=OFF -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=OFF && cmake --build "{2}" --target install --parallel 16' -f $VsDevCmd,$sourceDir,$buildDir,$configuration,$installDir,$VcpkgInstalled
    & cmd.exe /d /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "Coal $configuration build or install failed"
    }
}

Write-Output "Coal $Version installed under $dependencyRoot/install"
