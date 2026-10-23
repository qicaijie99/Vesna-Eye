# Builds the unit-test executable and runs it.
param(
    [string]$BuildDir = ""
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$buildArgs = @{ Targets = @("glowtrack_tests") }
if (-not [string]::IsNullOrWhiteSpace($BuildDir)) { $buildArgs.BuildDir = $BuildDir }
& (Join-Path $PSScriptRoot "build.ps1") @buildArgs
if ([string]::IsNullOrWhiteSpace($BuildDir)) { $BuildDir = Join-Path $root "build" }
$exe = Join-Path $BuildDir "Release\glowtrack_tests.exe"
if (-not (Test-Path $exe)) { throw "test executable not found: $exe" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "unit tests failed" }
