[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [switch]$Execute
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

# The new Ninja Multi-Config layout owns build/. These are previous generated
# trees and preset-specific subdirectories; source, x64 runtime files, SDK
# snapshots, artifacts, and user configuration are intentionally untouched.
$legacyPaths = @(
    'cmake-build-sdkcheck',
    'build_asan',
    'build_debug',
    'build_release',
    'build/acs',
    'build/acs-gtn',
    'build/asan',
    'build/generic',
    'build/gtn'
) | ForEach-Object { Join-Path $repoRoot $_ } | Where-Object { Test-Path -LiteralPath $_ }

if ($legacyPaths.Count -eq 0) {
    Write-Host 'No legacy build artifacts found.'
    return
}

Write-Host 'Legacy build artifacts:'
$legacyPaths | ForEach-Object { Write-Host "  $_" }

if (-not $Execute) {
    Write-Host 'Dry run only. Re-run with -Execute to remove the listed paths.'
    return
}

foreach ($path in $legacyPaths) {
    if ($PSCmdlet.ShouldProcess($path, 'Remove legacy build artifact')) {
        Remove-Item -LiteralPath $path -Recurse -Force
    }
}

Write-Host 'Legacy build artifact cleanup completed.'