# =============================================================================
#  run_and_visualize.ps1
#  ---------------------
#  One-shot driver for the Catoms3D Flow Bridge analysis:
#    1. Run the simulator in text mode on a chosen XML config.
#    2. Dump JSON per flow-graph phase (handled by the C++ side).
#    3. Render 2D PNGs per phase via visualize_flowgraph.py.
#
#  All outputs land in ./viz_flowgraph/<config_stem>/ next to this script
#  (i.e. inside applicationsBin/flowBridge/).
#
#  Usage:
#      .\run_and_visualize.ps1 short_pinch_4x4.xml
#      .\run_and_visualize.ps1 -Config island_config.xml
#      .\run_and_visualize.ps1 -All        # render every *.xml in this dir
# =============================================================================

param(
    [string]$Config = "",
    [switch]$All,
    [string]$Exe       = (Join-Path $PSScriptRoot "..\..\build-mingw64\flowBridgeTestMinCut.exe"),
    [string]$Visualizer = (Join-Path $PSScriptRoot "..\..\applicationsSrc\flowBridgeTestMinCut\visualize_flowgraph.py"),
    [string]$Python    = ""
)

# Native commands here (the simulator) write to stderr — don't treat that as a
# script failure.
$ErrorActionPreference = "Continue"

# Make MSYS2's mingw64 DLLs available to the child process for DLL loading
# (the simulator was built with MinGW and needs libstdc++-6.dll etc.).
# Prepend so DLL resolution always picks the right copy.
if (Test-Path "C:\msys64\mingw64\bin") {
    $env:Path = "C:\msys64\mingw64\bin;C:\msys64\usr\bin;" + $env:Path
}

# Pick a Python interpreter that ACTUALLY has matplotlib.  After the PATH
# prepend above MSYS2's `python` (which usually lacks matplotlib in this
# install) wins lookups, so probe a list of well-known interpreters and use
# the first one that imports matplotlib successfully.
function Test-Python([string]$candidate) {
    try {
        & $candidate -c "import matplotlib" 2>$null
        return ($LASTEXITCODE -eq 0)
    } catch { return $false }
}

if (-not $Python) {
    $candidates = @(
        "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe",
        "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe",
        "$env:LOCALAPPDATA\Programs\Python\Python310\python.exe",
        "C:\Python312\python.exe",
        "C:\Python311\python.exe",
        "C:\Python310\python.exe",
        "py",
        "python"
    )
    foreach ($cand in $candidates) {
        if ((Test-Path $cand) -or (Get-Command $cand -ErrorAction SilentlyContinue)) {
            if (Test-Python $cand) { $Python = $cand; break }
        }
    }
    if (-not $Python) {
        Write-Warning "No Python with matplotlib found; PNG rendering will be skipped."
    }
}
if ($Python) { Write-Host "Python: $Python" -ForegroundColor DarkGray }

Set-Location $PSScriptRoot

function Invoke-Single([string]$xml) {
    $xmlFull = (Resolve-Path $xml).Path
    Write-Host ""
    Write-Host "===== $xmlFull =====" -ForegroundColor Cyan

    # Run the simulator headless.  *> captures every stream (stdout, stderr,
    # warnings) into a single log file, sidestepping PowerShell's habit of
    # turning native stderr into ErrorRecords.
    $tmpLog = Join-Path $PSScriptRoot ".viz_run.log"
    & $Exe -t -c $xmlFull *> $tmpLog
    Select-String -Path $tmpLog -Pattern '\[viz\]' |
        ForEach-Object { $_.Line }
    Remove-Item $tmpLog -ErrorAction SilentlyContinue

    # Find the per-config viz dir and render only it.
    $stem = [System.IO.Path]::GetFileNameWithoutExtension($xmlFull)
    if (-not $stem) {
        Write-Warning "Could not derive stem from $xmlFull"
        return
    }
    $vizDir = Join-Path $PSScriptRoot "viz_flowgraph\$stem"
    if (-not (Test-Path $vizDir)) {
        Write-Warning "No JSON output for $stem (no viz_flowgraph\$stem)"
        return
    }
    if (-not $Python) { return }
    & $Python $Visualizer $vizDir $vizDir
}

if ($All) {
    Get-ChildItem -Path $PSScriptRoot -Filter *.xml |
        ForEach-Object { Invoke-Single $_.Name }
}
elseif ($Config) {
    Invoke-Single $Config
}
else {
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\run_and_visualize.ps1 <config.xml>"
    Write-Host "  .\run_and_visualize.ps1 -All"
    Write-Host ""
    Write-Host "Available configs:" -ForegroundColor Yellow
    Get-ChildItem -Path $PSScriptRoot -Filter *.xml |
        Select-Object -ExpandProperty Name
}
