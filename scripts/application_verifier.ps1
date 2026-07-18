param(
    [Parameter(Mandatory = $true)]
    [string]$Executable,
    [switch]$Enable,
    [switch]$Disable,
    [ValidateRange(1, 120)]
    [int]$TimeoutSeconds = 15
)

$ErrorActionPreference = "Stop"
if ($Enable -eq $Disable) {
    throw "Specify exactly one of -Enable or -Disable."
}

$verifier = Join-Path ${env:WINDIR} "System32\appverif.exe"
if (-not (Test-Path $verifier)) {
    throw "Application Verifier is not installed: $verifier"
}

$executablePath = (Resolve-Path $Executable).Path
$name = [IO.Path]::GetFileName($executablePath)
function Invoke-AppVerifier([string[]]$Arguments) {
    # appverif.exe is a GUI subsystem program on some Windows installations.
    # Start it explicitly and bound the wait so a non-interactive runner can
    # never hang forever.  Only this child process is stopped on timeout.
    $process = Start-Process -FilePath $verifier -ArgumentList $Arguments -PassThru
    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
        Stop-Process -Id $process.Id -Force
        throw "appverif did not finish within $TimeoutSeconds seconds."
    }
    if ($process.ExitCode -ne 0) {
        throw "appverif failed: $($process.ExitCode)"
    }
}
if ($Enable) {
    # Heap + handles are the minimum crash/leak gate.  No process is launched
    # here; callers can inspect the configuration before running their test.
    Invoke-AppVerifier @('/verify', $name, '/tests', 'Heaps', 'Handles', '/quiet')
    Write-Host "Application Verifier enabled for $name (Heaps, Handles)."
} else {
    Invoke-AppVerifier @('/unverify', $name, '/quiet')
    Write-Host "Application Verifier disabled for $name."
}
