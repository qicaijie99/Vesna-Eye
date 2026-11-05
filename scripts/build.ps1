# Build script for the Vesna-Eye (Windows / MSVC / prebuilt OpenCV).
#
#   powershell -ExecutionPolicy Bypass -File scripts/build.ps1
#   powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -Targets glowtrack_tests
#   powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -Targets glowtrack,glowtrack_tests
#
# Every teammate/validation run can use its own build directory to avoid clashing
# with parallel builds:  -BuildDir build_detector
param(
    [string]$BuildDir = "",
    [string]$OpenCVDir = "",
    [string]$Config = "Release",
    [string[]]$Targets = @(),
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildDir)) { $BuildDir = Join-Path $root "build" }

# Robust parameter handling for `powershell -File`:
#   - `-Targets a,b`      -> array (works)
#   - `-Targets a b`      -> the 2nd token leaks into the next positional
#                            parameter ($Config or $OpenCVDir). Detect a value
#                            that cannot be that parameter and treat it as a
#                            target instead.
$validConfigs = @("Debug", "Release", "RelWithDebInfo", "MinSizeRel")
$extraTargets = @()
if ($validConfigs -notcontains $Config) {
    $extraTargets += $Config
    $Config = "Release"
}
if (-not [string]::IsNullOrWhiteSpace($OpenCVDir) -and
    -not (Test-Path (Join-Path $OpenCVDir "OpenCVConfig.cmake"))) {
    $extraTargets += $OpenCVDir
    $OpenCVDir = ""
}
$Targets = @(@($Targets) + $extraTargets |
    ForEach-Object { if ($_ -is [string]) { $_ -split ',' } else { $_ } } |
    Where-Object { -not [string]::IsNullOrWhiteSpace($_) })

# Locate the prebuilt OpenCV (VC16 folder with OpenCVConfig.cmake).
if ([string]::IsNullOrWhiteSpace($OpenCVDir)) {
    $candidates = @(
        "D:\opencv410\opencv\build\x64\vc16\lib",
        "D:\opencv\build\x64\vc16\lib",
        "C:\opencv\build\x64\vc16\lib"
    )
    foreach ($candidate in $candidates) {
        if (Test-Path (Join-Path $candidate "OpenCVConfig.cmake")) { $OpenCVDir = $candidate; break }
    }
}
if ([string]::IsNullOrWhiteSpace($OpenCVDir) -or -not (Test-Path (Join-Path $OpenCVDir "OpenCVConfig.cmake"))) {
    throw "OpenCVConfig.cmake not found. Pass -OpenCVDir <...\build\x64\vc16\lib>."
}

$vcvars = $null
$vsRoots = @(
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise",
    "C:\Program Files\Microsoft Visual Studio\2022\Community"
)
foreach ($vsRoot in $vsRoots) {
    $candidate = Join-Path $vsRoot "VC\Auxiliary\Build\vcvars64.bat"
    if (Test-Path $candidate) { $vcvars = $candidate; break }
}
if (-not $vcvars) { throw "vcvars64.bat not found (Visual Studio C++ workload required)." }

if ($Clean -and (Test-Path $BuildDir)) { Remove-Item -Recurse -Force $BuildDir }

$commands = @()
if (-not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))) {
    $commands += "cmake -S `"$root`" -B `"$BuildDir`" -G `"Visual Studio 16 2019`" -A x64 -DOpenCV_DIR=`"$OpenCVDir`""
}
$build = "cmake --build `"$BuildDir`" --config $Config"
if ($Targets.Count -gt 0) { $build += " --target " + ($Targets -join " ") }
$commands += $build

Write-Host "Build root : $root"
Write-Host "Build dir  : $BuildDir"
Write-Host "OpenCV dir : $OpenCVDir"
$script = $commands -join " && "
cmd /c "call `"$vcvars`" >nul 2>&1 && $script"
if ($LASTEXITCODE -ne 0) { throw "build failed (exit code $LASTEXITCODE)" }
Write-Host "build OK"
