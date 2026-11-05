# One-command synthetic closed-loop validation for Vesna-Eye.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1
#
# The script
#   1) builds glowtrack + the validation tools (glowtrack_synth / glowtrack_eval),
#   2) renders the three synthetic scenarios under data\synthetic\<scenario>\,
#   3) runs glowtrack.exe on every scenario with config\synthetic_640x480.yaml,
#   4) evaluates the tracker CSV against the synthetic ground truth (glowtrack_eval),
#   5) prints a compact summary table and writes results\synthetic\summary.csv.
#
# Useful switches:
#   -BuildDir build_validation     isolated build directory (default: build)
#   -Scenarios static_sweep,dynamic,stress
#   -SkipBuild / -SkipGenerate / -SkipTrack
#   -RealVideos                    also run the two real phone videos (no GT)
#   -StaticFrames / -DynamicFrames / -StressFrames / -Seed
#
# All generated data/plots/metrics are reproducible; the Lead copies the numbers
# into docs/validation_report.md.
param(
    [string]$BuildDir = "build",
    [string[]]$Scenarios = @("static_sweep", "dynamic", "stress"),
    [switch]$SkipBuild,
    [switch]$SkipGenerate,
    [switch]$SkipTrack,
    [switch]$RealVideos,
    [int]$Seed = 42,
    [int]$StaticFrames = 180,
    [int]$DynamicFrames = 300,
    [int]$StressFrames = 150,
    [string]$Config = "config/synthetic_640x480.yaml",
    [string]$DataRoot = "data/synthetic",
    [string]$OutRoot = "results/synthetic"
)

$ErrorActionPreference = "Stop"
$Invariant = [System.Globalization.CultureInfo]::InvariantCulture
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Write-Step([string]$text) {
    Write-Host ""
    Write-Host "==== $text ====" -ForegroundColor Cyan
}

function Format-Num($value, [int]$decimals = 3) {
    if ($null -eq $value) { return "n/a" }
    return ([double]$value).ToString("F$decimals", $Invariant)
}

function Get-Metric([hashtable]$metrics, [string]$key) {
    if (-not $metrics.ContainsKey($key)) { return $null }
    $raw = $metrics[$key]
    if ($null -eq $raw -or "$raw" -eq "" -or "$raw" -eq "nan") { return $null }
    try { return [double]::Parse("$raw", $Invariant) } catch { return $null }
}

function Read-MetricsCsv([string]$path) {
    $metrics = @{}
    if (-not (Test-Path $path)) { return $metrics }
    Import-Csv -Path $path -Encoding UTF8 | ForEach-Object {
        $metrics[$_.metric] = $_.value
    }
    return $metrics
}

function Resolve-FrameCount([string]$scenario) {
    switch ($scenario) {
        "static_sweep" { return $StaticFrames }
        "dynamic" { return $DynamicFrames }
        "stress" { return $StressFrames }
        default { throw "unknown scenario '$scenario' (expected static_sweep|dynamic|stress)" }
    }
}

# ---------------------------------------------------------------------------
# 1) Build
# ---------------------------------------------------------------------------
$buildScript = Join-Path $root "scripts\build.ps1"
if (-not (Test-Path $buildScript)) { throw "missing $buildScript" }

if (-not $SkipBuild) {
    Write-Step "Building glowtrack + validation tools (build dir: $BuildDir)"
    & $buildScript -BuildDir $BuildDir -Config Release `
        -Targets @("glowtrack", "glowtrack_synth", "glowtrack_eval")
    if ($LASTEXITCODE -ne 0) { throw "build failed (exit code $LASTEXITCODE)" }
}

$binDir = Join-Path $root (Join-Path $BuildDir "Release")
$trackerExe = Join-Path $binDir "glowtrack.exe"
$synthExe = Join-Path $binDir "glowtrack_synth.exe"
$evalExe = Join-Path $binDir "glowtrack_eval.exe"
foreach ($exe in @($trackerExe, $synthExe, $evalExe)) {
    if (-not (Test-Path $exe)) {
        throw "missing executable $exe (run without -SkipBuild first)"
    }
}

$configPath = Join-Path $root $Config
if (-not (Test-Path $configPath)) { throw "missing config $configPath" }

$dataRootPath = Join-Path $root $DataRoot
$outRootPath = Join-Path $root $OutRoot
New-Item -ItemType Directory -Force -Path $outRootPath | Out-Null

# ---------------------------------------------------------------------------
# 2) Generate scenarios
# ---------------------------------------------------------------------------
$scenarioData = @{}
foreach ($scenario in $Scenarios) {
    $frames = Resolve-FrameCount $scenario
    $scenarioDir = Join-Path $dataRootPath $scenario
    $frameDir = Join-Path $scenarioDir "frames"
    $gtPath = Join-Path $scenarioDir "gt.csv"
    $scenarioData[$scenario] = @{
        dir    = $scenarioDir
        frames = $frames
        gt     = $gtPath
    }
    if ($SkipGenerate -and (Test-Path $gtPath) -and (Test-Path $frameDir)) {
        Write-Host "[generate] $scenario : reusing existing frames ($frames expected)"
        continue
    }
    Write-Step "Generating scenario '$scenario' ($frames frames)"
    & $synthExe --scenario $scenario --out $scenarioDir --frames $frames --seed $Seed
    if ($LASTEXITCODE -ne 0) { throw "glowtrack_synth failed for '$scenario'" }
    if (-not (Test-Path $gtPath)) { throw "ground truth not written: $gtPath" }
}

# ---------------------------------------------------------------------------
# 3) Track + 4) evaluate
# ---------------------------------------------------------------------------
if ($SkipTrack) { Write-Step "Skipping tracking (results not refreshed)" }

$summaryRows = @()
$plotRoot = $outRootPath

foreach ($scenario in $Scenarios) {
    $info = $scenarioData[$scenario]
    $trackCsv = Join-Path $outRootPath ("{0}_track.csv" -f $scenario)
    $evalDir = Join-Path $outRootPath ("eval_{0}" -f $scenario)

    if (-not $SkipTrack) {
        Write-Step "Tracking '$scenario' ($($info.frames) frames)"
        # The documented CLI passes the scenario directory; frame_source has a
        # recursive fallback that finds the PNGs in its frames/ subdirectory.
        & $trackerExe --config $configPath --images $info.dir --csv $trackCsv `
            --print-every 0 --quiet
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "glowtrack.exe returned exit code $LASTEXITCODE for '$scenario'"
        }
    }
    if (-not (Test-Path $trackCsv)) {
        Write-Warning "no tracker CSV for '$scenario' ($trackCsv); skipping evaluation"
        continue
    }

    Write-Step "Evaluating '$scenario'"
    & $evalExe --track $trackCsv --gt $info.gt --config $configPath `
        --out $evalDir --label $scenario
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "glowtrack_eval returned exit code $LASTEXITCODE for '$scenario'"
    }

    $metrics = Read-MetricsCsv (Join-Path $evalDir "metrics.csv")
    $summaryRows += [pscustomobject]@{
        scenario       = $scenario
        frames         = $info.frames
        detected_pct   = Get-Metric $metrics "detection_rate_visible"
        valid_pct      = Get-Metric $metrics "valid_rate_visible"
        measured_pct   = Get-Metric $metrics "measured_rate_visible"
        yaw_rmse_deg   = Get-Metric $metrics "yaw_rmse_measured_deg"
        pitch_rmse_deg = Get-Metric $metrics "pitch_rmse_measured_deg"
        dist_rel_rmse  = Get-Metric $metrics "distance_rel_rmse_pct"
        jitter_ratio_y = Get-Metric $metrics "jitter_ratio_yaw"
        jitter_ratio_p = Get-Metric $metrics "jitter_ratio_pitch"
        dropout_yaw_rm = Get-Metric $metrics "dropout_pred_yaw_rmse_deg"
        drop_pred_n    = Get-Metric $metrics "dropout_visible_predicted_frames"
        reproj_mean_px = Get-Metric $metrics "reproj_rmse_mean_px"
        eval_dir       = $evalDir
    }
}

# ---------------------------------------------------------------------------
# Optional: real videos (no ground truth, validation for the report)
# ---------------------------------------------------------------------------
$realRows = @()
if ($RealVideos) {
    $videoConfig = Join-Path $root "config/video_720p_phone.yaml"
    $videos = @("v1_green_target", "v2_green_target")
    foreach ($video in $videos) {
        $videoPath = Join-Path $root ("data/videos/{0}.mp4" -f $video)
        if (-not (Test-Path $videoPath)) { Write-Warning "missing $videoPath"; continue }
        $realOut = Join-Path $root ("results/real")
        New-Item -ItemType Directory -Force -Path $realOut | Out-Null
        $label = $video.Split('_')[0]
        $trackCsv = Join-Path $realOut ("{0}_track.csv" -f $label)
        $evalDir = Join-Path $realOut ("eval_{0}" -f $label)
        Write-Step "Tracking real video '$video'"
        & $trackerExe --config $videoConfig --video $videoPath --csv $trackCsv `
            --print-every 0 --quiet
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "glowtrack.exe returned exit code $LASTEXITCODE for '$video'"
        }
        if (-not (Test-Path $trackCsv)) { continue }
        & $evalExe --track $trackCsv --config $videoConfig --out $evalDir --label $label
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "glowtrack_eval returned exit code $LASTEXITCODE for '$video'"
        }
        $metrics = Read-MetricsCsv (Join-Path $evalDir "metrics.csv")
        $realRows += [pscustomobject]@{
            video          = $label
            detected_pct   = Get-Metric $metrics "detection_rate_all"
            valid_pct      = Get-Metric $metrics "valid_rate_all"
            measured_pct   = Get-Metric $metrics "measured_rate_all"
            jitter_ratio_y = Get-Metric $metrics "jitter_ratio_yaw"
            max_yaw_rate   = Get-Metric $metrics "max_abs_yaw_rate_dps"
            reproj_mean_px = Get-Metric $metrics "reproj_rmse_mean_px"
            los_yaw_std    = Get-Metric $metrics "los_yaw_dev_std_deg"
            los_pitch_std  = Get-Metric $metrics "los_pitch_dev_std_deg"
            eval_dir       = $evalDir
        }
    }
}

# ---------------------------------------------------------------------------
# 5) Summary
# ---------------------------------------------------------------------------
Write-Step "Synthetic validation summary"
if ($summaryRows.Count -eq 0) {
    Write-Host "no scenarios evaluated"
} else {
    $table = @()
    foreach ($row in $summaryRows) {
        $table += [pscustomobject]@{
            scenario        = $row.scenario
            frames          = $row.frames
            detected        = Format-Num $row.detected_pct 1 + " %"
            valid           = Format-Num $row.valid_pct 1 + " %"
            measured        = Format-Num $row.measured_pct 1 + " %"
            yaw_rmse_deg    = Format-Num $row.yaw_rmse_deg 3
            pitch_rmse_deg  = Format-Num $row.pitch_rmse_deg 3
            dist_rel_rmse   = Format-Num $row.dist_rel_rmse 2 + " %"
            jitter_yaw      = Format-Num $row.jitter_ratio_y 2
            jitter_pitch    = Format-Num $row.jitter_ratio_p 2
            dropout_yaw_rmse = Format-Num $row.dropout_yaw_rm 2
            reproj_mean_px  = Format-Num $row.reproj_mean_px 2
        }
    }
    $table | Format-Table -AutoSize | Out-String -Width 220 | Write-Host

    $summaryCsv = Join-Path $outRootPath "summary.csv"
    $summaryRows | Select-Object scenario, frames, detected_pct, valid_pct,
        measured_pct, yaw_rmse_deg, pitch_rmse_deg, dist_rel_rmse,
        jitter_ratio_y, jitter_ratio_p, dropout_yaw_rm, drop_pred_n,
        reproj_mean_px | Export-Csv -Path $summaryCsv -NoTypeInformation -Encoding UTF8
    Write-Host "summary csv : $summaryCsv"
}

if ($realRows.Count -gt 0) {
    Write-Step "Real-video validation summary (no ground truth)"
    $realTable = @()
    foreach ($row in $realRows) {
        $realTable += [pscustomobject]@{
            video        = $row.video
            detected     = Format-Num $row.detected_pct 1 + " %"
            valid        = Format-Num $row.valid_pct 1 + " %"
            measured     = Format-Num $row.measured_pct 1 + " %"
            jitter_yaw   = Format-Num $row.jitter_ratio_y 2
            max_yaw_rate = Format-Num $row.max_yaw_rate 1 + " deg/s"
            reproj_mean  = Format-Num $row.reproj_mean_px 2 + " px"
            los_yaw_std  = Format-Num $row.los_yaw_std 3 + " deg"
            los_pitch_std = Format-Num $row.los_pitch_std 3 + " deg"
        }
    }
    $realTable | Format-Table -AutoSize | Out-String -Width 220 | Write-Host
    Write-Host "real-video outputs: results\real"
}

Write-Host ""
Write-Host "Outputs:"
Write-Host "  synthetic data  : $dataRootPath"
Write-Host "  tracker CSVs    : $outRootPath"
Write-Host "  metrics + plots : $outRootPath\eval_<scenario>"
Write-Host "Done."
