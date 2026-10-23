# 合成数据闭环验证（validation workstream 草稿）

> **最终验证数字以 `docs/validation_report.md` 为准（本文件为工具/方法细节与人工作流记录）。**

本文件由 validation workstream 维护，是最终验证报告（`docs/validation_report.md`，Lead 负责）的
数据/方法底稿。所有数字均可由 `scripts/run_synthetic_validation.ps1` 一键复现。

- 代码：`tools/synth_dataset.cpp`（合成数据生成器）、`tools/eval_metrics.cpp`（指标+绘图评估器）、
  `scripts/run_synthetic_validation.ps1`（一键构建/生成/跟踪/评估/汇总）。
- 数据：`data/synthetic/<scenario>/frames/*.png` + `gt.csv` + `meta.yaml`（630 帧，生成耗时约 9 s）。
- 结果：`results/synthetic/<scenario>_track.csv`、`results/synthetic/eval_<scenario>/`；
  真实视频：`results/real/<video>_track.csv`、`results/real/eval_<video>/`。
- 本文 §4–§6 数字为 **2026-10-23 集成调参后的最终 config** 下重跑结果（kalman:
  `process_noise_angle=0.0004, process_noise_rate=0.08, meas_sigma_deg=0.35,
  range_process_noise=0.5, range_meas_sigma_m=0.1`；单元测试 57/57 通过）。

## 1. 目的

真实视频没有真值，只能用"PnP 视线角 vs 白盘椭圆中心视线角偏差 / 重投影误差 / 平滑度"做间接验证。
合成数据在**已知 3D 位姿**下渲染目标，使整条链路（检测 → PnP → Kalman → 输出 CSV）的
yaw / pitch / 距离误差、检测覆盖率、丢帧外推精度、滤波平滑度都可以**量化闭环**。

评估器同时支持 no-GT 模式，用于两个真实手机视频的验证（检测率、状态分布、抖动、角速度、
PnP 视线角与椭圆中心方位角的交叉校验）。

## 2. 合成数据生成器（glowtrack_synth）

```
glowtrack_synth --scenario <static_sweep|dynamic|stress> --out <dir> [--frames N]
                [--width 640 --height 480 --fx 500 --fy 500 --cx 320 --cy 240]
                [--seed 42] [--noise sigma] [--blur px] [--fps 30]
                [--disc-mm 50] [--glow-mm 62] [--no-clutter] [--quiet]
```

输出：`<out>/frames/frame_00000.png ...`、`<out>/gt.csv`、`<out>/meta.yaml`。

渲染模型（物理上自洽的针孔投影，`include/glowtrack/types.hpp` 约定）：

- 目标平面中心 = 白盘圆心，`t = d * normalize(tan(yaw), -tan(pitch), 1)`，
  因此 `atan2(t.x,t.z)=yaw`、`-atan2(t.y,t.z)=pitch`、`||t||=d` 严格成立；
- 目标法向指向相机（正对相机），再叠加平面内 roll 与绕 x 轴的 tilt（0–40°）使白盘椭圆产生离面形变；
- 渲染元素：**白色中心圆盘**（PnP 已知半径圆，`disc_diameter_mm/2=25 mm`）→ 深绿色边界环 →
  亮绿色发光环（`glow_outer_diameter_mm/2=31 mm`）→ 更大的绿色扩散片/V 形翼 + 绿色高斯光晕（additive） →
  后方浅灰白色 3D 打印 V 形支架（亮度低于白盘阈值，仅作背景结构）→ 地面灰色渐变 + 绿色杂物块
  （rounded rect/ellipse，可被绿色掩膜检出但没有"白盘+绿环"结构，用于误检测试）+ 白色灯状杂物；
- 每帧变化：曝光漂移、暗角、高斯噪声、线性运动模糊（动态场景按帧间位移自动估计方向/长度）、
  可选暗色矩形局部遮挡；
- 白盘圆轮廓按 96 点投影后 `fitEllipse`，写入 GT 的 `disc_cx/disc_cy/disc_radius_px`，
  等价于"理想检测器"对渲染白盘拟合出的椭圆，可直接与 tracker CSV 的
  `ellipse_cx/ellipse_cy/semi_major/semi_minor` 对比。

GT CSV：`frame,timestamp_s,yaw_deg,pitch_deg,distance_m,roll_deg,tilt_deg,visible,disc_cx,disc_cy,disc_radius_px`
（`visible=0` 表示该帧目标未被渲染，但 yaw/pitch/distance 仍为真值，用于丢帧外推评估）。

### 三个场景（默认参数）

| 场景 | 帧数 | 位姿覆盖 | 特点 |
| --- | ---: | --- | --- |
| `static_sweep` | 180 | yaw −25…+25°、pitch −20…+20°、距离 0.4…1.6 m、roll/tilt 0…40°；白盘半径 8.8–35.4 px | 确定性“缓动”位姿路径（3 帧/路径节点，余弦扫掠 + smoothstep 过渡，速度连续，避免滤波器被瞬跳惩罚）。分段：A 偏航扫掠（d=0.8 m）→ 过渡 → B 俯仰+距离联动扫掠 → 过渡 → C roll/tilt 扫掠（视线静止） |
| `dynamic` | 300 | yaw ±20°、pitch ±12°、距离 0.50–1.20 m、roll ±12°、tilt ±8°；白盘半径 10.7–27.5 px | 正弦平滑运动（yaw 周期 6 s，pitch 5 s，距离 8 s）；**3 段丢帧**（帧 60–69、141–148、215–220，共 24 帧目标不渲染）用于滤波外推验证 |
| `stress` | 150 | yaw ±14°、pitch ±9°、距离 0.70–1.10 m；白盘半径 11.6–18.1 px（10–20 px 量级） | 强噪声 σ=12、强运动模糊（≥10 px）、曝光波动；**2 段暗矩形局部遮挡**（帧 33–63、98–125） |

`meta.yaml` 记录场景名、内参、帧率、种子、噪声、模糊、丢帧/遮挡区间。

## 3. 评估器（glowtrack_eval）

```
glowtrack_eval --track <tracker.csv> [--gt <gt.csv>] [--config <yaml>] --out <dir> [--label <name>] [--no-plots]
```

- 鲁棒解析 tracker CSV（按表头映射列，容忍缺列/空值、旧式无表头格式），按 `frame` 与 GT 对齐
  （无重叠时退化为按行号对齐并告警）。
- 输出：`metrics.csv`（metric,value,unit,count）、`metrics.md`（分节表格 + 判定 + 指标说明）、
  `matched_series.csv`（逐帧对比）、PNG 图；控制台摘要。
- 主要指标：
  - **精度（有 GT）**：`yaw/pitch_rmse_measured_deg`（滤波输出）、`yaw/pitch_rmse_raw_deg`
    （raw PnP，即检测+PnP 精度）、bias/MAE/p95/max、`yaw/pitch_rmse_predicted_deg`、
    距离 `distance_rmse_m / distance_rel_rmse_pct / distance_bias_m`、
    白盘中心像素误差 `disc_center_err_rmse_px`、椭圆半径相对误差 `disc_radius_rel_rmse_pct`；
  - **覆盖率**：GT 可见帧上的 detected / valid / measured 比例、GT 不可见帧误检率；
  - **丢帧外推**：GT 可见但未测量的预测帧误差（`dropout_pred_*`）；目标未渲染帧的
    "盲预测"误差（`blind_pred_*`）与"保持上次测量"基线（`blind_hold_*`）；
  - **滤波平滑度**：滤波输出与 raw PnP 的二阶差分标准差、一阶差分平均绝对值及比值
    （jitter ratio < 1 表示滤波降低抖动）；`filter_raw_*_diff_*` 为滤波输出相对 raw PnP 的
    平滑滞后/预测偏差；
  - **连续性与状态**：detected gap / prediction run 统计、状态分布、最大/分位角速度、quality；
  - **PnP 质量**：重投影 RMS 统计（当前 `PoseEstimate::reproj_rmse_px` 未填充，工具自动标注 n/a）；
  - **no-GT 模式**：检测/有效/测量率、状态分布、抖动、角速度、gap/prediction run、
    以及 `--config` 内参下的 **PnP 视线角 vs 椭圆中心方位角** 交叉校验
    （`los_yaw/pitch_dev_bias/std/rmse`）。
- 判定阈值（仅参考，不参与算法）：yaw/pitch RMSE(measured) ≤ 1.0°、raw PnP RMSE ≤ 0.5°、
  距离相对 RMSE ≤ 8 %、GT 可见帧有效率 ≥ 90 %、jitter ratio ≤ 0.85（合成数据 raw PnP 接近无噪声、
  `jitter_raw_*_2nd_std_deg < 0.15°` 时该判据自动标 n/a）。

### 一键脚本

```
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1
# 可选：-BuildDir build_validation -Scenarios static_sweep,dynamic,stress
#       -SkipBuild / -SkipGenerate / -SkipTrack
#       -RealVideos          # 附带两个真实视频的 no-GT 验证
#       -StaticFrames/-DynamicFrames/-StressFrames/-Seed
```

脚本流程：构建 `glowtrack` + 两个工具 → 生成 3 个场景 → 用
`config/synthetic_640x480.yaml` 对每个场景跑 `glowtrack.exe --images data/synthetic/<scenario>` →
`glowtrack_eval` → 打印汇总表并写 `results/synthetic/summary.csv`。

## 4. 闭环验证结果

环境：Windows 10 / MSVC 2019 / OpenCV 4.10 / Release；`build_validation` 构建；
数据 640×480、fx=fy=500、主点 (320,240)，目标白盘直径 50 mm、发光片外径 62 mm。
最终 config 见文首（集成调参后）。

### 4.1 合成场景汇总（检测+PnP+Kalman 全链路）

| 场景 | 帧数 | 检测率¹ | 有效率¹ | 测量率¹ | yaw RMSE² | pitch RMSE² | raw yaw RMSE³ | raw pitch RMSE³ | 距离 RMSE | 距离相对 RMSE | 预测误差⁴ | 白盘中心误差 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| static_sweep | 180 | 100.0 % | 98.9 % | 98.9 % | 0.286° | 0.523° | 0.011° | 0.014° | 0.051 m | 6.22 % | 无丢帧 | 0.07 px |
| dynamic | 300 | 100 % | 99.3 % | 99.3 % | 0.120° | 0.112° | 0.006° | 0.011° | 0.058 m | 5.36 % | 1.03°/0.11°（n=24） | 0.07 px |
| stress | 150 | 52.7 % | 71.3 % | 48.7 % | 0.310° | 0.046° | 0.250° | 0.025° | 0.057 m | 6.68 % | 2.25°/0.57°（n=34） | 2.13 px |

¹ 分母为 GT 可见帧（static 180、dynamic 276、stress 150）。static_sweep 在最终 config 下**可见帧全部检出**。
² 滤波输出相对 GT 的 RMSE（GT 可见且输出有效帧）。
³ raw PnP（未滤波 `raw_yaw_deg/raw_pitch_deg`）相对 GT 的 RMSE，即检测+PnP 精度。
⁴ dynamic 的 24 帧为 3 段丢帧（目标未渲染）内 tracker 仍输出有效预测的帧；stress 的 34 帧为遮挡/丢失后
tracker 仍以 PREDICT 输出、GT 可见的帧。

补充：

- dynamic 中 GT 不可见 24 帧的误检率为 **0 %**（背景绿色杂物未被误检成目标）。
- 全场景 raw PnP 精度为 0.006–0.25°（yaw）、0.011–0.025°（pitch），说明检测+PnP 本身精度很高；
  滤波后误差 0.05–0.52°，主要来自平滑滞后（见表 4.3 的 filter−raw 偏差）。
- 视觉检查：三个场景的帧与 GT 对齐（如 `data/synthetic/dynamic/frames/frame_00100.png`）。

### 4.2 分场景结论与分解

**dynamic（主精度场景）**：目标 276/276 可见帧全部检出，输出有效 99.3 %；raw PnP 误差 0.006°/0.011°；
滤波后 0.120°/0.112°；距离相对误差 5.36 %（偏差 −4.63 cm，系统性偏小，原因见 4.5）；
丢帧外推 10/8/6 帧的 yaw 误差 1.03°、pitch 0.11°，明显优于"保持上次测量"基线（1.87°/2.57°）；
丢帧结束后数帧内收敛，无明显瞬态。

**static_sweep**：分段统计（180 帧，3 帧/路径节点；最终 config 下无丢检、无 PREDICT/LOST）：

| 分段 | 帧区间 | 距离 bias / RMSE | yaw RMSE | pitch RMSE |
| --- | --- | ---: | ---: | ---: |
| A 偏航扫掠（d=0.8 m） | 0–71 | −0.024 / 0.028 m | 0.208° | 0.041° |
| 过渡 T1 | 72–89 | +0.011 / 0.028 m | 0.386° | 0.532° |
| B 俯仰+距离扫掠 | 90–143 | −0.068 / 0.072 m | 0.165° | 0.595° |
| 过渡 T2 | 144–161 | −0.032 / 0.057 m | 0.631° | 1.149° |
| C roll/tilt 扫掠（视线静止） | 162–179 | −0.054 / 0.056 m | 0.111° | 0.192° |

- 最终 config 下检测率 100 %（180/180），整段无丢检；tracker 状态 TRACK 178 帧 + ACQUIRE 2 帧，
  无 PREDICT/LOST。早期 config 在远端（≈1.5 m、白盘 8.8 px）出现的 14 帧漏检不再发生。
- 距离最大绝对误差 0.113 m；|误差|>0.1 m 仅 6 帧（帧 121/124/128/159–161），来自 B/C 段距离扫掠
  与 range filter 的少量暂态，不再出现早期 config 的"距离发散"（旧 config 下 static 距离相对 RMSE
  为 17.6 %，pipeline 侧甚至观测到 66.9 %；最终 config 为 6.22 %）。
- C 段（roll/tilt 0–40°、视线静止）验证了椭圆倾斜/旋转姿态下 PnP 与滤波均正常（yaw 0.111°、距离 7.0 %）。
  最大/分位角速度 34.0/33.2°（yaw）、46.9/42.3°（pitch），滤波器在速度反转处仍有约 1° 级 pitch 滞后。

**stress**：检测率 52.7 % 是刻意构造的极端条件（强噪声 σ=12 + 强运动模糊 + 暗色矩形遮挡约 40 % 帧），
属于压力测试而非正常工作点；在能检出的帧上 raw PnP yaw RMSE 0.250°（p95 0.81°）、pitch 0.025°，
白盘中心误差 2.13 px（模糊导致的椭圆拟合退化）；遮挡/丢失后 tracker 以 PREDICT 维持输出，
34 帧预测误差 yaw 2.25°/pitch 0.57°。

### 4.3 滤波平滑度 / 滞后（jitter & filter-vs-raw）

| 数据 | yaw 二阶差分 std 滤波 / raw | yaw jitter ratio | pitch 二阶差分 std 滤波 / raw | pitch jitter ratio | 滤波−raw yaw 偏差（mean±std, max） | 滤波−raw pitch 偏差（mean±std, max） | max / p95 角速度 yaw, pitch |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| dynamic（合成） | 0.234° / 0.020° | 11.53 | 0.037° / 0.022° | 1.68 | +0.045 ± 0.121°，max 0.34° | −0.005 ± 0.115°，max 0.20° | 20.9/20.8, 15.1/15.0 dps |
| static_sweep（合成） | 0.068° / 0.062° | 1.09 | 0.109° / 0.095° | 1.14 | +0.033 ± 0.279°，max 0.96° | +0.102 ± 0.511°，max 1.90° | 34.0/33.2, 46.9/42.3 dps |
| stress（合成） | 0.046° / 0.047° | 0.99 | 0.018° / 0.028° | 0.63 | +0.016 ± 0.112°，max 0.28° | +0.018 ± 0.050°，max 0.11° | 15.9/15.8, 8.1/8.0 dps |
| **v1 真实视频** | **0.051° / 0.065°** | **0.78** | **0.079° / 0.146°** | **0.54** | −0.001 ± 0.076°，max 0.32° | −0.001 ± 0.069°，max 0.37° | 29.7/15.8, 17.3/8.4 dps |
| **v2 真实视频** | **0.100° / 0.108°** | **0.92** | **0.059° / 0.071°** | **0.83** | +0.002 ± 0.161°，max 0.72° | −0.001 ± 0.094°，max 0.40° | 40.8/28.1, 35.4/24.2 dps |

结论：合成数据的 raw PnP 接近无噪声（二阶差分 std ≈0.02–0.10°），此时"滤波降低抖动"不适用
（评估器自动把该判据标 n/a；dynamic 的 11.53× 只是 raw 分母极小的结果）；static_sweep 的 jitter ratio
≈1（1.09/1.14，滤波器基本不额外引入抖动）。**最终 config 在真实视频上两项指标都 < 1：
v1（0.78/0.54）、v2（0.92/0.83）**——调参后的 Kalman 在快速运动（v2 最大 40.8°/s）下也降低抖动，
滤波−raw 偏差 RMSE 仅 0.16°/0.09°、最大滞后 0.72°（此前 config 在 v2 上 yaw jitter ratio 2.96、
最大滞后 3.17°，已解决）。

### 4.4 真实视频 no-GT 验证（config/video_720p_phone.yaml，fx=960，最终 config）

| 视频 | 帧数 | 检测率 | 有效率 | 测量率 | 滤波后状态 | LOS 交叉校验 yaw 偏差 | LOS 交叉校验 pitch 偏差 | 质量均值 | 丢检段 / 预测段 |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- | ---: | --- |
| v1_green_target | 539 | 100 % | 99.6 % | 99.6 % | TRACK 537 / ACQUIRE 2 | −0.012 ± 0.039° | −0.017 ± 0.033° | 0.968 | 0 / 0 |
| v2_green_target | 406 | 100 % | 99.5 % | 99.5 % | TRACK 404 / ACQUIRE 2 | +0.021 ± 0.019° | −0.015 ± 0.040° | 0.973 | 0 / 0 |

- 两段真实视频全程检出（100 %）且输出有效（99.5 %+），说明检测/PnP/跟踪在真实数据上稳定；
- PnP 视线角与"白盘椭圆中心方位角"偏差均值 0.03° 以内、std 0.02–0.04°，即圆特征 PnP 与图像方位
  自洽（该交叉校验无法发现共同的系统偏差，绝对精度以合成数据为准）；
- 重投影 RMS 当前不可用（`PoseEstimate::reproj_rmse_px` 未填充，见限制）；
- 最终 config 下两段视频都不再出现预测段（此前 v2 的短预测消失），滤波抖动/滞后指标见 4.3。

### 4.5 距离误差分解与说明

各场景距离相对 RMSE：dynamic 5.36 %（bias −4.63 cm）、stress 6.68 %（+3.88 cm）、
static_sweep 6.22 %（bias −3.79 cm，最大绝对误差 0.113 m）。系统性成分分析：

- 检测到的白盘拟合椭圆等效半径相对 GT 理想椭圆的误差：static RMSE 5.14 %、dynamic 5.97 %
  （均值上约 +4.7 %，即偏大），stress 因强模糊边缘侵蚀 RMSE 11.68 %（整体偏小）
  （`disc_radius_rel_rmse_pct`，见各 `metrics.md`）；圆特征 PnP 的距离与等效半径近似成反比，
  因此 normal 场景距离系统性偏小约 3–5 %（dynamic bias −4.63 cm，符合该量级）。
  这是**白盘边缘阈值/抗锯齿/发光溢出导致的 0.5–1 px 椭圆外扩**引起的系统偏差，并不是 PnP 求解器噪声；
- stress 场景因强模糊，白盘边缘被侵蚀，椭圆偏小 → 距离偏大（+3.88 cm）；
- 最终 config 的 range filter 重调参（`range_process_noise=0.5, range_meas_sigma_m=0.1`）后，
  此前 static_sweep 在距离扫掠段的距离发散已消失：静态场景距离相对 RMSE 从 17.6 %（旧 config）
  降到 6.22 %，整段无 PREDICT/LOST，最大绝对误差 0.113 m；残余小偏差来自椭圆半径系统性偏大。

## 5. 结论

1. 合成闭环链路（检测 → PnP → Kalman → CSV）可复现运行：单帧 0.6–1.9 ms（640×480 合成 / 720p 真实），
   三场景 630 帧 + 两段真实视频全部处理成功。
2. **检测 + PnP 精度很高**：合成数据 raw PnP yaw RMSE 0.006–0.011°（normal）、0.25°（stress 小目标+模糊），
   白盘中心像素误差 0.07 px（normal）；真实视频上 PnP 视线角与椭圆中心方位角偏差 std ≤0.04°。
3. **最终 config 下滤波输出精度**：dynamic 0.120°/0.112°、static 0.286°/0.523°、stress 0.310°/0.046°；
   相比调参前（dynamic/static 0.46–0.67° 量级）显著改善，static_sweep 可见帧检测率也从 92.2 %
   提升到 100 %。
4. **丢帧外推有效**：dynamic 3 段共 24 帧盲预测 yaw 1.03°/pitch 0.11°，优于"保持上次值"基线
   1.87°/2.57°；stress 遮挡段 34 帧预测误差 yaw 2.25°/pitch 0.57°。
5. **真实视频**：检测率 100 %、有效率/测量率 ≥99.5 %；滤波在 v1/v2 上都降低抖动
   （jitter ratio 0.78/0.54 与 0.92/0.83），滤波−raw 偏差 RMSE ≤0.16°、最大滞后 ≤0.72°。
6. **距离**：三场景 5.4–6.7 % 相对 RMSE（bias −4.6…+3.9 cm），主要来自白盘椭圆半径的系统性偏差；
   最终 config 已消除距离发散，静态场景 180 帧无丢检。

## 6. 已知限制与后续建议

1. `PoseEstimate::reproj_rmse_px` 目前恒为 0（pose_estimator 未填充），重投影 RMS 指标不可用；
   评估器已在 `metrics.md` 中显式标注 n/a，等待 pose-filter-dev 修复后重跑即可得到该指标。
2. 距离系统偏差：白盘边缘阈值化使拟合椭圆半径偏大 ~4.7 %（RMSE 5.1–6.0 %），距离随之偏小 3–5 %。
   可考虑椭圆/轮廓亚像素边缘细化，或在 PnP 前对白盘半径做偏置标定（用合成数据即可拟合该偏置）。
3. 合成数据的 raw PnP 接近无噪声，因此"滤波降低抖动"的量化证据应以真实视频为准
   （最终 config 下 v1 0.78/0.54、v2 0.92/0.83）；合成场景的 jitter ratio 仅作回归对照。
4. stress 场景（检测率 52.7 %、遮挡段 34 帧预测误差 yaw 2.25°）刻意为极端条件，用于考核鲁棒性下限，
   不代表正常工作点；正常距离/角度范围内三场景检测与输出均稳定。
5. 合成噪声/模糊/杂物为简化模型，不代表真实传感器特性；绝对精度以合成闭环为准，真实视频只做
   no-GT 一致性/稳定性验证。
6. 运行集成：`--images data/synthetic/<scenario>` 需要 frame_source 对 `frames/` 子目录的递归 fallback
   （pipeline-dev 已实现，`tests/test_config.cpp::frame_source_image_sequence_nested_fallback` 锁定）。

## 7. 复现命令

```powershell
# 一键：构建 + 生成 + 跟踪 + 评估 + 汇总（约 2–3 分钟；最终 config）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1

# 使用独立构建目录（不干扰其他人的 build\）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1 -BuildDir build_validation

# 代码未变、只重跑数据+评估（先单独构建一次，再 -SkipBuild）
#   powershell -NoProfile -ExecutionPolicy Bypass -Command "& '.\scripts\build.ps1' -BuildDir build_validation -Targets @('glowtrack','glowtrack_synth','glowtrack_eval')"
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1 -BuildDir build_validation -SkipBuild

# 单独运行某个场景
build\Release\glowtrack.exe --config config\synthetic_640x480.yaml `
    --images data\synthetic\dynamic --csv results\synthetic\dynamic_track.csv --print-every 0 --quiet
build\Release\glowtrack_eval.exe --track results\synthetic\dynamic_track.csv `
    --gt data\synthetic\dynamic\gt.csv --config config\synthetic_640x480.yaml `
    --out results\synthetic\eval_dynamic --label dynamic

# 真实视频 no-GT 验证
build\Release\glowtrack.exe --config config\video_720p_phone.yaml `
    --video data\videos\v1_green_target.mp4 --csv results\real\v1_track.csv --print-every 0 --quiet
build\Release\glowtrack_eval.exe --track results\real\v1_track.csv `
    --config config\video_720p_phone.yaml --out results\real\eval_v1 --label v1
```

## 8. 资产清单（Lead 写最终报告可直接引用）

| 资产 | 路径 |
| --- | --- |
| 合成数据（3 场景，630 帧 + GT + meta） | `data/synthetic/{static_sweep,dynamic,stress}/` |
| 合成 tracker CSV | `results/synthetic/{static_sweep,dynamic,stress}_track.csv` |
| 合成指标/表格/图 | `results/synthetic/eval_<scenario>/metrics.md`、`metrics.csv`、`*.png`、`matched_series.csv` |
| 一键汇总表 | `results/synthetic/summary.csv` |
| 真实视频 tracker CSV | `results/real/{v1,v2}_track.csv` |
| 真实视频指标/图 | `results/real/eval_<video>/metrics.md`、`*.png` |
| 场景预览帧 | `data/synthetic/dynamic/frames/frame_00100.png` 等 |

图（每个 `eval_*/` 目录）：`yaw_timeseries.png`、`pitch_timeseries.png`、
`yaw_pitch_error_timeseries.png`（含 raw PnP 误差曲线）、`distance_timeseries.png`、
`error_hist_yaw.png`、`error_hist_pitch.png`、`jitter_second_diff.png`、`jitter_first_diff.png`、
`reproj_rmse_timeseries.png`、`los_crosscheck.png`（带 config 时）。
