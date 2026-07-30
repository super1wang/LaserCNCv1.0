[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [switch]$Execute
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

# build-cmake/ is reserved for CMake/Ninja and build-vs/ for the generated
# Visual Studio solution. The old shared build/ tree and historical trees are
# safe to remove; x64 runtime files and both supported trees stay untouched.
$legacyPaths = @(
    'build',
    'cmake-build-sdkcheck',
    'build_asan',
    'build_debug',
    'build_release'
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
        $blocked = @()
        Get-ChildItem -LiteralPath $path -Force | ForEach-Object {
            $childPath = $_.FullName
            try {
                Remove-Item -LiteralPath $childPath -Recurse -Force
            } catch {
                $blocked += $childPath
                Write-Warning "Could not remove '$childPath': $($_.Exception.Message)"
            }
        }

        if ($blocked.Count -eq 0) {
            Remove-Item -LiteralPath $path -Force
        } else {
            Write-Warning "Legacy tree remains because files are in use: $path"
        }
    }
}

Write-Host 'Legacy build artifact cleanup pass completed.'
