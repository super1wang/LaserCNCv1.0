param(
    [Parameter(Mandatory = $true)]
    [string]$Root,

    [Parameter(Mandatory = $true)]
    [string]$BuildDir
)

$ErrorActionPreference = 'Stop'

$tsPath = Join-Path $Root 'translations/lasercnc_zh_CN.ts'
if (-not (Test-Path -LiteralPath $tsPath)) {
    throw "Translation source was not found: $tsPath"
}
if (-not (Test-Path -LiteralPath $BuildDir)) {
    throw "Build directory was not found: $BuildDir"
}

[xml]$translationSource = Get-Content -LiteralPath $tsPath -Raw -Encoding UTF8
$contextNames = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
$duplicateContexts = [System.Collections.Generic.List[string]]::new()
foreach ($context in $translationSource.TS.context) {
    $name = [string]$context.name
    if ([string]::IsNullOrWhiteSpace($name) -or $name.StartsWith('obsolete::', [System.StringComparison]::Ordinal)) {
        continue
    }
    if (-not $contextNames.Add($name)) {
        $duplicateContexts.Add($name)
    }
}

$mocFiles = Get-ChildItem -LiteralPath $BuildDir -Recurse -Filter 'moc_*.cpp' -File
if ($mocFiles.Count -eq 0) {
    throw "No generated MOC files were found under: $BuildDir"
}

$qualifiedNames = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::Ordinal)
foreach ($mocFile in $mocFiles) {
    $content = Get-Content -LiteralPath $mocFile.FullName -Raw
    foreach ($match in [regex]::Matches($content, '"(lcnc::[A-Za-z_][A-Za-z0-9_:]*)"')) {
        [void]$qualifiedNames.Add($match.Groups[1].Value)
    }
}

$missingQualifiedContexts = [System.Collections.Generic.List[string]]::new()
foreach ($qualifiedName in $qualifiedNames) {
    $separator = $qualifiedName.LastIndexOf('::', [System.StringComparison]::Ordinal)
    if ($separator -lt 0) {
        continue
    }
    $bareName = $qualifiedName.Substring($separator + 2)
    if ($contextNames.Contains($bareName) -and -not $contextNames.Contains($qualifiedName)) {
        $missingQualifiedContexts.Add("$bareName -> $qualifiedName")
    }
}

if ($duplicateContexts.Count -gt 0 -or $missingQualifiedContexts.Count -gt 0) {
    $details = [System.Collections.Generic.List[string]]::new()
    if ($duplicateContexts.Count -gt 0) {
        $details.Add("Duplicate active translation contexts: $(($duplicateContexts | Sort-Object -Unique) -join ', ')")
    }
    if ($missingQualifiedContexts.Count -gt 0) {
        $details.Add("Bare context shadows a MOC-qualified class context: $(($missingQualifiedContexts | Sort-Object) -join ', ')")
    }
    throw ($details -join [Environment]::NewLine)
}

Write-Host "Translation context contract passed: $($qualifiedNames.Count) qualified MOC names checked."
