# 生成可视化演示：标注视频的 GIF / 九宫格截图 / 曲线，以及一个可离线打开的 HTML 看板。
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/make_visual_demo.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/make_visual_demo.ps1 -Open
#
# 输出（可随时重新生成，results/ 已被 gitignore）：
#   results/demo/v1_demo.gif, v2_demo.gif          标注视频的动图（6 fps, 320 宽）
#   results/demo/v1_contact.png, v2_contact.png    标注视频九宫格截图
#   results/demo/index.html                        可视化看板（视频/GIF/曲线/指标）
#   docs/validation/assets/*_contact.png           九宫格截图（提交入库）
param(
    [string]$BuildDir = "build",
    [switch]$SkipBuild,
    [switch]$Open
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$bin = Join-Path $root (Join-Path $BuildDir "Release")
$real = Join-Path $root "results\real_validation"
$demo = Join-Path $root "results\demo"
$plots = Join-Path $demo "plots"
$assets = Join-Path $root "docs\validation\assets"
New-Item -ItemType Directory -Force -Path $demo, $plots, $assets | Out-Null
$utf8 = New-Object System.Text.UTF8Encoding($false)

function Read-Metric([string]$evalDir, [string]$key) {
    $path = Join-Path $evalDir "metrics.csv"
    if (-not (Test-Path $path)) { return "n/a" }
    foreach ($row in (Import-Csv -Path $path -Encoding UTF8)) {
        if ($row.metric -eq $key) { return $row.value }
    }
    return "n/a"
}
function Fmt([string]$value, [string]$format = "0.00") {
    if ($value -eq "n/a") { return "n/a" }
    return ([double]$value).ToString($format, [Globalization.CultureInfo]::InvariantCulture)
}

# ---------------------------------------------------------------------------
# 1) 确保有二进制与标注视频（缺失时自动生成）
# ---------------------------------------------------------------------------
if (-not $SkipBuild -and -not (Test-Path (Join-Path $bin "glowtrack.exe"))) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") -BuildDir $BuildDir
}
$overlays = @("v1_overlay.mp4", "v2_overlay.mp4") | ForEach-Object { Join-Path $real $_ }
if (-not (Test-Path $overlays[0]) -or -not (Test-Path $overlays[1])) {
    Write-Host "[demo] overlay videos missing, running real-video validation first..."
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "run_validation.ps1") -BuildDir $BuildDir -SkipBuild
}
$v1mp4 = Join-Path $real "v1_overlay.mp4"
$v2mp4 = Join-Path $real "v2_overlay.mp4"

# ---------------------------------------------------------------------------
# 2) 九宫格截图（每 2 秒取 1 帧 → 3x3）
# ---------------------------------------------------------------------------
Write-Host "[demo] contact sheets..."
ffmpeg -v error -y -i $v1mp4 -vf "select='not(mod(n\,60))',scale=270:480,tile=3x3" -frames:v 1 (Join-Path $demo "v1_contact.png")
ffmpeg -v error -y -i $v2mp4 -vf "select='not(mod(n\,45))',scale=270:480,tile=3x3" -frames:v 1 (Join-Path $demo "v2_contact.png")
foreach ($name in @("v1_contact.png", "v2_contact.png")) {
    Copy-Item -LiteralPath (Join-Path $demo $name) -Destination (Join-Path $assets $name) -Force
}

# ---------------------------------------------------------------------------
# 3) 标注视频 GIF（6 fps、320 宽，便于微信/文档中直接查看）
# ---------------------------------------------------------------------------
Write-Host "[demo] animated GIFs..."
$gifFilter = "fps=6,scale=320:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
ffmpeg -v error -y -i $v1mp4 -vf $gifFilter -loop 0 (Join-Path $demo "v1_demo.gif")
ffmpeg -v error -y -i $v2mp4 -vf $gifFilter -loop 0 (Join-Path $demo "v2_demo.gif")

# ---------------------------------------------------------------------------
# 4) 复制验证曲线（真实视频 + 合成场景）
# ---------------------------------------------------------------------------
Write-Host "[demo] plots..."
foreach ($v in @("v1", "v2")) {
    $src = Join-Path $real "eval_$v"
    foreach ($name in @("yaw_timeseries.png", "pitch_timeseries.png", "jitter_second_diff.png", "los_crosscheck.png")) {
        $p = Join-Path $src $name
        if (Test-Path $p) { Copy-Item -LiteralPath $p -Destination (Join-Path $plots "real_${v}_$name") -Force }
    }
}
foreach ($s in @("static_sweep", "dynamic", "stress")) {
    $src = Join-Path $root "results\synthetic\eval_$s"
    foreach ($name in @("yaw_pitch_error_timeseries.png", "yaw_timeseries.png", "pitch_timeseries.png",
                       "distance_timeseries.png", "error_hist_yaw.png", "jitter_second_diff.png", "los_crosscheck.png")) {
        $p = Join-Path $src $name
        if (Test-Path $p) { Copy-Item -LiteralPath $p -Destination (Join-Path $plots "synth_${s}_$name") -Force }
    }
}

# ---------------------------------------------------------------------------
# 5) 读取指标
# ---------------------------------------------------------------------------
$realV1 = Join-Path $real "eval_v1"
$realV2 = Join-Path $real "eval_v2"
$metrics = @{
    v1det   = Fmt (Read-Metric $realV1 "detection_rate_all") "0.0"
    v1valid = Fmt (Read-Metric $realV1 "valid_rate_all") "0.0"
    v1jy    = Fmt (Read-Metric $realV1 "jitter_ratio_yaw")
    v1jp    = Fmt (Read-Metric $realV1 "jitter_ratio_pitch")
    v1losy  = Fmt (Read-Metric $realV1 "los_yaw_dev_std_deg") "0.000"
    v1losp  = Fmt (Read-Metric $realV1 "los_pitch_dev_std_deg") "0.000"
    v2det   = Fmt (Read-Metric $realV2 "detection_rate_all") "0.0"
    v2valid = Fmt (Read-Metric $realV2 "valid_rate_all") "0.0"
    v2jy    = Fmt (Read-Metric $realV2 "jitter_ratio_yaw")
    v2jp    = Fmt (Read-Metric $realV2 "jitter_ratio_pitch")
    v2losy  = Fmt (Read-Metric $realV2 "los_yaw_dev_std_deg") "0.000"
    v2losp  = Fmt (Read-Metric $realV2 "los_pitch_dev_std_deg") "0.000"
}
$synRows = @()
foreach ($s in @("static_sweep", "dynamic", "stress")) {
    $e = Join-Path $root "results\synthetic\eval_$s"
    $synRows += [pscustomobject]@{
        name    = $s
        det     = Fmt (Read-Metric $e "detection_rate_visible") "0.0"
        valid   = Fmt (Read-Metric $e "valid_rate_visible") "0.0"
        yaw     = Fmt (Read-Metric $e "yaw_rmse_measured_deg") "0.000"
        pitch   = Fmt (Read-Metric $e "pitch_rmse_measured_deg") "0.000"
        rawYaw  = Fmt (Read-Metric $e "yaw_rmse_raw_deg") "0.000"
        distRel = Fmt (Read-Metric $e "distance_rel_rmse_pct")
    }
}

# ---------------------------------------------------------------------------
# 6) 生成 HTML 看板
# ---------------------------------------------------------------------------
$cards = ""
foreach ($pair in @(@{ n = "v1"; mp4 = $v1mp4; gif = "v1_demo.gif"; contact = "v1_contact.png" },
                    @{ n = "v2"; mp4 = $v2mp4; gif = "v2_demo.gif"; contact = "v2_contact.png" })) {
    $mp4Rel = "../real_validation/" + (Split-Path $pair.mp4 -Leaf)
    $cards += @"
  <div class="card">
    <h3>真实视频 $($pair.n)（标注：ROI/白盘椭圆/绿光椭圆/PnP 坐标轴/HUD）</h3>
    <video controls muted loop preload="metadata" src="$mp4Rel"></video>
    <p class="hint">若浏览器无法播放 mp4，可直接看下面的 GIF（6 fps / 320 宽）。</p>
    <img class="gif" src="$($pair.gif)" alt="$($pair.n) demo gif">
    <img class="contact" src="$($pair.contact)" alt="$($pair.n) contact sheet">
  </div>
"@
}

$synTableRows = ""
foreach ($r in $synRows) {
    $synTableRows += "<tr><td>$($r.name)</td><td>$($r.det) %</td><td>$($r.valid) %</td><td>$($r.yaw)°</td><td>$($r.pitch)°</td><td>$($r.rawYaw)°</td><td>$($r.distRel) %</td></tr>"
}

$html = @"
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>green_glow_tracker 可视化演示</title>
<style>
  body { background:#0f1419; color:#d7dde5; font-family:"Segoe UI","Microsoft YaHei",sans-serif; margin:0; padding:24px 32px 64px; }
  h1 { color:#5ee787; margin:0 0 4px; font-size:26px; }
  h2 { color:#8fd3ff; border-bottom:1px solid #263241; padding-bottom:6px; margin-top:38px; font-size:20px; }
  h3 { color:#e8eef5; font-size:15px; font-weight:600; }
  .sub { color:#8a97a6; margin-bottom:8px; }
  .grid { display:flex; gap:22px; flex-wrap:wrap; align-items:flex-start; }
  .card { background:#161d26; border:1px solid #263241; border-radius:10px; padding:14px 16px; }
  video { width:360px; max-height:640px; background:#000; border-radius:6px; display:block; margin:6px 0; }
  img.gif { width:320px; border-radius:6px; display:block; margin:10px 0; }
  img.contact { width:340px; border-radius:6px; display:block; margin:10px 0; }
  .hint { color:#7d8b9b; font-size:12px; margin:2px 0; }
  table { border-collapse:collapse; margin:10px 0 6px; font-size:14px; }
  th, td { border:1px solid #2b3a4a; padding:6px 14px; text-align:right; }
  th { background:#1c2733; color:#9fc7e8; }
  td:first-child, th:first-child { text-align:left; }
  .ok { color:#5ee787; font-weight:600; }
  .plots { display:flex; flex-wrap:wrap; gap:14px; }
  .plots img { width:460px; max-width:46%; border:1px solid #263241; border-radius:8px; background:#fff; }
  code { background:#1c2733; padding:2px 6px; border-radius:4px; color:#ffd479; }
  a { color:#8fd3ff; }
</style>
</head>
<body>
<h1>绿色发光目标跟踪 · 可视化演示</h1>
<div class="sub">green_glow_tracker · 绿色光斑检测 + 圆特征 PnP + Kalman 滤波 → yaw / pitch（RoboMaster 无动力滑翔机上位机视觉模块）</div>

<h2>1. 指标总览（两个真实视频）</h2>
<table>
<tr><th>指标</th><th>v1（539 帧）</th><th>v2（406 帧）</th></tr>
<tr><td>检测率</td><td>$($metrics.v1det) %</td><td>$($metrics.v2det) %</td></tr>
<tr><td>有效输出率（valid）</td><td>$($metrics.v1valid) %</td><td>$($metrics.v2valid) %</td></tr>
<tr><td>抖动比 yaw（滤波后/原始，&lt;1 为降噪）</td><td>$($metrics.v1jy)</td><td>$($metrics.v2jy)</td></tr>
<tr><td>抖动比 pitch</td><td>$($metrics.v1jp)</td><td>$($metrics.v2jp)</td></tr>
<tr><td>PnP 视线角 vs 椭圆中心偏差 std（yaw）</td><td>$($metrics.v1losy)°</td><td>$($metrics.v2losy)°</td></tr>
<tr><td>PnP 视线角 vs 椭圆中心偏差 std（pitch）</td><td>$($metrics.v1losp)°</td><td>$($metrics.v2losp)°</td></tr>
<tr><td>处理耗时（720p 单线程）</td><td>≈2.0 ms/帧</td><td>≈1.8 ms/帧</td></tr>
</table>
<div class="hint">更多指标与合成数据验证见 <a href="../../docs/validation_report.md">docs/validation_report.md</a>。</div>

<h2>2. 真实视频回放</h2>
<div class="grid">
$cards
</div>

<h2>3. 真实视频曲线（滤波/交叉校验）</h2>
<div class="plots">
  <img src="plots/real_v1_yaw_timeseries.png" alt="v1 yaw">
  <img src="plots/real_v2_yaw_timeseries.png" alt="v2 yaw">
  <img src="plots/real_v1_jitter_second_diff.png" alt="v1 jitter">
  <img src="plots/real_v2_los_crosscheck.png" alt="v2 LOS">
</div>

<h2>4. 合成数据闭环验证（已知真值）</h2>
<table>
<tr><th>场景</th><th>检测率(可见帧)</th><th>有效输出</th><th>滤波后 yaw RMSE</th><th>滤波后 pitch RMSE</th><th>原始 PnP yaw RMSE</th><th>距离相对 RMSE</th></tr>
$synTableRows
</table>
<div class="plots">
  <img src="plots/synth_dynamic_yaw_pitch_error_timeseries.png" alt="dynamic error">
  <img src="plots/synth_dynamic_distance_timeseries.png" alt="dynamic distance">
  <img src="plots/synth_static_sweep_error_hist_yaw.png" alt="static hist">
  <img src="plots/synth_stress_yaw_pitch_error_timeseries.png" alt="stress error">
</div>

<h2>5. 如何实时查看 / 重新生成</h2>
<p>实时窗口（按 <code>q</code> / <code>ESC</code> 或关闭窗口退出，<code>--display-scale</code> 调整缩放）：</p>
<p><code>.\\build\\Release\\glowtrack.exe --config config\\video_720p_phone.yaml --video data\\videos\\v1_green_target.mp4 --display --display-scale 0.6</code></p>
<p>重新生成标注视频 / GIF / 本页面：<code>powershell -File scripts\\run_validation.ps1</code> 然后 <code>powershell -File scripts\\make_visual_demo.ps1 -Open</code></p>

</body>
</html>
"@
$htmlPath = Join-Path $demo "index.html"
[IO.File]::WriteAllText($htmlPath, $html, $utf8)
Write-Host "[demo] dashboard: $htmlPath"
Write-Host "[demo] gifs     : $(Join-Path $demo 'v1_demo.gif'), $(Join-Path $demo 'v2_demo.gif')"
Write-Host "[demo] contacts : $(Join-Path $demo 'v1_contact.png'), $(Join-Path $demo 'v2_contact.png')"

if ($Open) {
    Start-Process $htmlPath
    Write-Host "[demo] opened in default browser"
}
