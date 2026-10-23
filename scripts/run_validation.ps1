# End-to-end validation on the two supplied real videos (no ground truth).
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_validation.ps1
#
# Produces under results/real_validation/:
#   v1_track.csv / v2_track.csv           per-frame tracker output
#   v1_overlay.mp4 / v2_overlay.mp4       annotated videos
#   eval_v1/ eval_v2/                     metrics.md + plots from glowtrack_eval
# and copies a few overlay sample frames into docs/validation/assets/.
param(
    [string]$Config = "config\video_720p_phone.yaml",
    [switch]$SkipBuild,
    [switch]$SkipOverlay
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$out = Join-Path $root "results\real_validation"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$assets = Join-Path $root "docs\validation\assets"
New-Item -ItemType Directory -Force -Path $assets | Out-Null

if (-not $SkipBuild) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") -Targets @("glowtrack", "glowtrack_eval")
}

$videos = @(
    @{ Name = "v1"; Path = "data\videos\v1_green_target.mp4"; Times = @(1, 6, 12, 16) },
    @{ Name = "v2"; Path = "data\videos\v2_green_target.mp4"; Times = @(1, 4, 8, 12) }
)

foreach ($video in $videos) {
    $name = $video.Name
    $trackCsv = Join-Path $out "${name}_track.csv"
    $overlay = Join-Path $out "${name}_overlay.mp4"
    Write-Host ""
    Write-Host "=== [$name] tracking $(Split-Path $video.Path -Leaf) ==="
    $glowArgs = @(
        "--config", $Config,
        "--video", $video.Path,
        "--csv", $trackCsv,
        "--print-every", "0",
        "--quiet"
    )
    if (-not $SkipOverlay) { $glowArgs += @("--overlay", $overlay) }
    & (Join-Path $root "build\Release\glowtrack.exe") @glowArgs
    if ($LASTEXITCODE -ne 0) { throw "tracking failed for $name" }

    Write-Host "--- eval $name ---"
    & (Join-Path $root "build\Release\glowtrack_eval.exe") `
        --track $trackCsv --config $Config --out (Join-Path $out "eval_$name") --label $name
    if ($LASTEXITCODE -ne 0) { throw "evaluation failed for $name" }

    # overlay sample frames for the report (ffmpeg is available on the dev machine)
    if (-not $SkipOverlay -and (Test-Path $overlay)) {
        $index = 0
        foreach ($t in $video.Times) {
            $framePath = Join-Path $assets ("${name}_overlay_t${t}s.png")
            ffmpeg -v error -y -ss $t -i $overlay -frames:v 1 $framePath 2>$null
            if (Test-Path $framePath) { Write-Host "asset: $framePath" }
            $index++
        }
    }
}

Write-Host ""
Write-Host "Validation artifacts: $out"
Write-Host "Report assets      : $assets"
