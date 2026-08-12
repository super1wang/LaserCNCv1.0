param(
    [string]$Executable = "",
    [ValidateRange(1, 86400)]
    [int]$DurationSeconds = 300,
    [ValidateRange(1, 3600)]
    [int]$SampleIntervalSeconds = 5,
    [string]$OutputPath = "",
    [ValidateRange(-1, [long]::MaxValue)]
    [long]$MaxPrivateBytesGrowth = -1,
    [ValidateRange(-1, [int]::MaxValue)]
    [int]$MaxHandleGrowth = -1
)

$ErrorActionPreference = "Stop"
$repoRoot = Resolve-Path "$PSScriptRoot\.."
if ([string]::IsNullOrWhiteSpace($Executable)) {
    # VS Code defaults to the Ninja route; pass -Executable for the isolated
    # Visual Studio build at x64\vs\Debug\LaserCNC.exe.
    $Executable = Join-Path $repoRoot "x64\ninja\Debug\LaserCNC.exe"
}
$Executable = (Resolve-Path $Executable).Path
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $stamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $OutputPath = Join-Path $repoRoot "artifacts\runtime-baseline-$stamp.csv"
}

$outputDirectory = Split-Path -Parent $OutputPath
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

# The script owns only the process it starts.  It never attaches to or stops an
# existing LaserCNC instance, which keeps workstation debugging sessions safe.
$process = Start-Process -FilePath $Executable -WorkingDirectory (Split-Path -Parent $Executable) -PassThru
$samples = [System.Collections.Generic.List[object]]::new()
$deadline = (Get-Date).AddSeconds($DurationSeconds)
$failure = $null

try {
    while ((Get-Date) -lt $deadline) {
        $process.Refresh()
        if ($process.HasExited) {
            throw "LaserCNC exited early with code $($process.ExitCode)"
        }
        $samples.Add([pscustomobject]@{
            Timestamp       = Get-Date -Format o
            ProcessId       = $process.Id
            WorkingSetBytes = $process.WorkingSet64
            PrivateBytes    = $process.PrivateMemorySize64
            HandleCount     = $process.HandleCount
            Threads         = $process.Threads.Count
        })
        Start-Sleep -Seconds $SampleIntervalSeconds
    }
} catch {
    $failure = $_
} finally {
    $process.Refresh()
    if (-not $process.HasExited) {
        Stop-Process -Id $process.Id
        $process.WaitForExit()
    }
}

$samples | Export-Csv -NoTypeInformation -Encoding utf8 -LiteralPath $OutputPath
if ($samples.Count -eq 0) {
    throw "LaserCNC produced no runtime samples."
}
$first = $samples[0]
$last = $samples[$samples.Count - 1]
$privateGrowth = [long]$last.PrivateBytes - [long]$first.PrivateBytes
$handleGrowth = [int]$last.HandleCount - [int]$first.HandleCount
@($first, $last) | Format-Table -AutoSize
Write-Host "PrivateBytes growth: $privateGrowth bytes; Handle growth: $handleGrowth"
Write-Host "Wrote $($samples.Count) samples to $OutputPath"
if ($MaxPrivateBytesGrowth -ge 0 -and $privateGrowth -gt $MaxPrivateBytesGrowth) {
    throw "PrivateBytes growth $privateGrowth exceeds gate $MaxPrivateBytesGrowth."
}
if ($MaxHandleGrowth -ge 0 -and $handleGrowth -gt $MaxHandleGrowth) {
    throw "Handle growth $handleGrowth exceeds gate $MaxHandleGrowth."
}
if ($failure) {
    throw $failure
}
