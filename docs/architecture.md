# 架构与算法设计

本文档说明 `green_glow_tracker` 上位机视觉模块的整体架构、各模块职责、核心算法与坐标约定。
验证方法与结果见 `docs/validation_report.md`，串口协议见 `docs/protocol.md`。

## 1. 系统定位

```text
      ┌─────────────────────── 滑翔飞行器（无主动动力） ───────────────────────┐
      │                                                                       │
      │   OpenMV H7 Plus 相机                                                  │
      │   (OV5640 + 2.8mm, QVGA 320x240, JPEG over UART/USB)                   │
      │            │                                                          │
      │            ▼                                                          │
      │   ┌──────────────────────────────┐        ┌─────────────────────────┐ │
      │   │ 上位机视觉模块（本仓库）      │  UART  │ 下位机（飞控/舵机）       │ │
      │   │ C++ / OpenCV                 │ ─────► │ 姿态解算 + 气动舵面控制   │ │
      │   │ 检测 → PnP → Kalman → yaw/pitch │      │ （不在本仓库范围）        │ │
      │   └──────────────────────────────┘        └─────────────────────────┘ │
      └───────────────────────────────────────────────────────────────────────┘
```

- 输入：相机图像帧（在线为 OpenMV 串口/USB 图像流；离线为视频或图像序列）。
- 输出：目标视线角 `yaw`、`pitch`（及其角速度）、距离、置信度、跟踪状态，按固定周期/帧率发给下位机。
- 本模块只做视觉：不做 IMU 融合、不做弹道/提前量、不做控制律。

## 2. 坐标与角度约定

相机坐标系（OpenCV）：`x` 右、`y` 下、`z` 前（光轴）。

目标位姿：`X_cam = R * X_target + t`（target → camera，`cv::solvePnP` 约定）。
目标坐标系原点在白色中心圆盘中心，目标平面为 `z=0`，法线 `+z` 指向目标正面外。

输出角：

```text
yaw   = atan2(t.x, t.z)     右正
pitch = -atan2(t.y, t.z)    上正   （图像 y 向下，取负号变成"抬头为正"）
distance = ||t||
yaw_rate   = d(yaw)/dt
pitch_rate = d(pitch)/dt
```

下位机若需要"图像坐标式"pitch（下正），取 `-pitch` 即可；协议里发的就是上述定义。

## 3. 数据流与模块

```text
FrameSource ──► TargetDetector ──► PoseEstimator ──► TargetTracker ──► outputs
(video/image/  (绿光分割+白盘    (圆特征 PnP:      (状态机 + ROI +      (CSV / overlay /
 camera/OpenMV) 椭圆拟合+校验)     IPPE+精化+消歧)    Kalman 滤波+外推)     UART 协议包)
```

| 文件 | 职责 |
| --- | --- |
| `src/frame_source.cpp` | 视频 / 图像序列 / USB 相机 / OpenMV 串口图像流 |
| `src/detector.cpp` | 绿色发光区域分割、白盘椭圆提取、候选校验与评分 |
| `src/pose_estimator.cpp` | 圆特征 PnP：IPPE 初值、镜像解消歧、椭圆重投影非线性精化 |
| `src/kalman_filter.cpp` | 视线角 4 状态 CV Kalman、距离 2 状态 Kalman、CRC16 |
| `src/tracker.cpp` | 跟踪状态机、ROI 预测、量测关联、丢帧外推、延时补偿 |
| `src/config.cpp` | YAML 配置加载（`cv::FileStorage`） |
| `src/output_sink.cpp` | CSV 日志、标注视频渲染、串口发送 |
| `src/main.cpp` | CLI、标定模式、主循环、统计汇总 |

## 4. 目标检测

### 4.1 目标结构（来自视频分析，详见 `docs/recon_target_analysis.md`）

- 中央**白色圆盘**：过曝亮白，直径 50–125 px（0.4–1.5 m 距离），是干净的椭圆特征。
- 紧贴白盘的**绿色圆环 + V 形绿色发光扩散片**：用于颜色/结构校验。
- 背景实验室有绿色椅子/货架/反光，直接取"最大绿色连通域"会误检。

### 4.2 检测流程

1. **绿色分割**：BGR→HSV，阈值 `[green_h_min, green_h_max] × S≥green_s_min, V≥green_v_min`，
   形态学开/闭运算去噪（`morph_open_ksize` / `morph_close_ksize`）。
2. **候选提取**：
   - 主路径：对绿色连通域做 `findContours(RETR_CCOMP)`，取连通域的**孔洞子轮廓**
     （= 白盘边界）→ `cv::fitEllipse`，得到圆心/长短轴/倾角；
   - 回退路径：高亮低饱和（`V≥white_v_min, S≤white_s_max`）白色斑点 → 拟合椭圆，
     再校验其外圈存在绿色环（运动模糊导致绿环断裂时使用）。
3. **结构校验**（拒绝背景干扰）：
   - 椭圆长短轴比 `≥ min_axis_ratio`；
   - 白盘区域亮度/饱和度符合过曝白盘特征；
   - 在椭圆外 `ring_inner_scale ~ ring_outer_scale` 倍半径的环带上采样
     `ring_samples` 个点，绿色命中比例 `≥ ring_green_min_ratio`；
   - 绿光面积、候选与 ROI 中心的距离等先验。
4. **候选评分**：`score ∈ [0,1]`，加权组合环带绿色比例、圆度、尺寸合理性、ROI 距离、
   白盘亮度；`score < score_threshold` 丢弃。
5. **ROI 搜索**：跟踪中只在预测 ROI 内做检测（大幅降低耗时），输出坐标统一为全图坐标。

## 5. PnP 位姿解算（圆特征）

白盘是已知半径 `R = disc_diameter_mm / 2` 的圆（目标平面 `z=0`）。
圆在图像上的投影是椭圆，椭圆包含圆的 3D 姿态信息，但存在**平面圆二义性**（镜像解）。

### 5.1 初值：仿射一致 4 点 + IPPE

把拟合椭圆参数化：

```text
p(θ) = c + a·cosθ·u_major + b·sinθ·v_minor ,  θ = 0°, 90°, 180°, 270°
```

配对的 3D 点为圆上同样参数角的点 `P(θ) = (R cosθ, R sinθ, 0)`。
这组 2D-3D 对应在仿射投影意义下是严格一致的（椭圆长短轴端点对应圆上倾斜方向的直径端点）。
调用：

```cpp
cv::solvePnPGeneric(obj_pts, img_pts, K, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
```

平面目标 IPPE 通常返回 2 组解（镜像）。

### 5.2 消歧

- **朝向约束**：目标正面朝向相机 ⇒ 相机坐标系下目标法线 `n_z < 0`；
- **时序连续性**：与上一帧已接受位姿比较法线夹角、平移差、重投影代价；
- 若两解代价接近且无法判定，标记 `ambiguous=true`，交给滤波器时序约束处理。

### 5.3 非线性精化

4 点对应在真实透视下存在微小偏差，因此用椭圆（而不是 4 个点）作为观测量做精化：

- 取当前位姿，在目标平面上采样 N 个圆上 3D 点，投影到图像；
- 用拟合椭圆的标准二次型 `F(u,v) = 0` 构造残差，取一阶几何近似
  `r = F(p) / ||∇F(p)||`（即点到椭圆的近似几何距离）；
- 对 6 自由度位姿做 Levenberg–Marquardt 数值迭代（`refine_iterations`），
  残差平方和最小化；
- 收敛失败时退回 IPPE 解。

最终 `reproj_rmse_px` 由投影圆点到拟合椭圆的几何距离统计得到。

> **注意（交付版本实测）**：对已知半径的圆，任意图像椭圆都恰好是某个圆位姿的投影
> （背投影锥的两个圆截面解），因此精化后的模型圆重投影残差在数学上必然 ≈ 0。
> `reproj_rmse_px` 在实现中保留为"精化失败/回退"的检测量（`refine_iterations=0` 时非零，
> 单元测试覆盖），**不能**作为检测噪声或精度指标。视角精度请用合成真值误差与
> "PnP 视线角 vs 椭圆中心视线角"交叉校验（见 `docs/validation_report.md` §7）。

### 5.4 输出

由 `t = tvec`（目标中心在相机系，单位 mm）计算 yaw / pitch / distance。
注意：目标尺寸 `R` 的整体缩放只会等比缩放 `t`，`yaw/pitch` 理论上**不受目标尺寸误差影响**，
但 `distance` 需要现场实测 `disc_diameter_mm`。

## 6. Kalman 滤波

### 6.1 视线角滤波器（4 状态 CV 模型）

状态 `X = [yaw, pitch, yaw_rate, pitch_rate]^T`，离散白噪声加速度模型：

```text
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]
Q = q_angle·[dt^3/3 ...] + q_rate·[...]   （角度/角速度过程噪声谱密度）
z = [yaw_meas, pitch_meas]
H = [1 0 0 0; 0 1 0 0]
R = diag(σ_yaw², σ_pitch²),  σ = meas_sigma_deg · clamp(meas_sigma_scale_px / r_px, 1, …)
```

- yaw 残差用 `normalizeAngle()` 卷绕到 `(-π, π]`，处理 ±180° 跨越；
- **卡方门限**：`ν^T S^{-1} ν > gate_chi2` 时判为野值，跳过量测（继续预测）；
- **质量因子**：由协方差映射到 `quality ∈ [0,1]`，丢帧时按 `quality_decay_per_frame` 衰减；
- **延时补偿**：输出前用 `predictAhead(latency_compensation_ms)` 外推到"当前真实时刻"，
  抵消相机曝光 + 传输 + 处理延时；
- **机动自适应 Q**：被接受的量测若接近卡方门限（大机动），下一步 predict 的过程噪声最多放大
  7 倍并逐帧衰减，使滤波器在目标快速反转时快速收敛、稳态时保持平滑；
- 交付默认参数（30 fps 场景，见 `config/*.yaml`）：`process_noise_angle=4e-4`、
  `process_noise_rate=0.08`、`meas_sigma_deg=0.35`（720p）/`0.7`（QVGA，按表观半径缩放）、
  `gate_chi2=13.8`。参数由真实视频 + 合成数据联合调优（滤波后误差 vs 抖动/滞后权衡）。

### 6.2 距离滤波器（2 状态）

状态 `[range, range_rate]`，量测为 PnP 距离。距离输出不参与姿态控制时精度要求较低，
主要用于安全距离/丢失判定、ROI 尺寸预测与日志。

交付默认 `range_process_noise=0.5`、`range_meas_sigma_m=0.1`。初版参数（0.01/0.05）在距离
快速变化时滤波会因卡方门限持续拒绝量测而发散（合成 static_sweep 距离误差 66.9 %）；
调大过程噪声后降为 **5.4–6.7 %**（见 `docs/validation_report.md` §8）。

## 7. 跟踪状态机

```text
                 ┌──────────── 全图搜索 ────────────┐
                 ▼                                  │
   ┌────────┐  命中   ┌──────────┐ 连续 N 帧  ┌────────┐  丢帧  ┌────────────┐  超时  ┌──────┐
   │ SEARCH │ ──────► │ ACQUIRE  │ ─────────► │ TRACK  │ ─────► │  PREDICT   │ ─────► │ LOST │
   └────────┘         └──────────┘            └────────┘        └────────────┘        └──────┘
        ▲                                                              │ 重捕
        └──────────────────────────────────────────────────────────────┘
```

- `SEARCH`/`IDLE`：每帧做一次全图检测（720p 全图检测约 2–3.6 ms，预算内，可最快捕获目标）；
  `full_search_interval` 只用于**限制 LOST 状态的重捕频率**；
- `ACQUIRE`：连续 `acquire_hits` 帧有效检测才进入 `TRACK`（抑制误检）；
- `TRACK`：ROI 检测 + 滤波更新，输出 measured=true；
- `PREDICT`：无检测时滤波外推，`predicted=true`，`quality` 衰减，输出仍 valid；
- `LOST`：连续 `max_miss_frames` 帧无检测，停止输出 valid，回到搜索。

**量测关联**：检测椭圆中心需落在预测位置附近（门限随目标像素尺寸自适应），
否则本帧检测视为其他目标/误检，不更新滤波。

**ROI 预测**：`ROI = 预测中心 ± (roi_padding_scale × 预测像素半径 + 速度裕量)`，
随目标尺寸和预测不确定性增长，并夹到图像范围内。

## 8. 输出

- **CSV**（离线/验证）：逐帧状态、yaw/pitch、角速度、距离、质量、PnP 重投影误差、
  原始 PnP 角度、检测椭圆参数。
- **标注视频**：ROI、目标椭圆、视线十字、HUD（角度/距离/状态/帧率/耗时）。
- **串口**：19 字节定长包（含 CRC-16），见 `docs/protocol.md`。

## 9. 配置

`config/*.yaml`（`cv::FileStorage` 读取）：`camera`（内参/畸变）、`target`（圆盘/绿光直径）、
`detector`、`pose`、`kalman`、`tracker`、`output`。

三套预置配置：

| 配置 | 用途 |
| --- | --- |
| `video_720p_phone.yaml` | 两个验证视频（720x1280，fx≈960） |
| `openmv_h7plus_qvga.yaml` | 部署：OpenMV H7 Plus QVGA 320x240，fx≈247 |
| `synthetic_640x480.yaml` | 合成数据闭环验证（640x480，fx=500） |

## 10. 性能目标与验证

- 实测（VS2019 Release，单线程 OpenCV，i7 级 CPU）：
  - 720p 端到端（检测+PnP+滤波）：**≈1.8–2.0 ms/帧**；检测器全图 2.4 ms（debug 关）/3.6 ms（debug 开），
    ROI 模式 0.63–0.70 ms；
  - QVGA（320x240）端到端：**≈0.40–0.47 ms/帧**；
  - 30 fps 预算 33 ms ⇒ 720p 占用约 6 %、QVGA 约 1.3 %，实时余量充足。
- 精度验证：
  - 合成数据（已知真值）：yaw/pitch 误差、距离误差、滤波抖动降低、丢帧外推误差；
  - 真实视频（无真值）：检测率/连续性、PnP 重投影误差、PnP 视线角与椭圆中心视线角交叉一致性、
    滤波平滑度；标注视频人工抽检。
- 详见 `docs/validation_report.md`。

## 11. 与 OpenMV 的配合

- OpenMV 只作为相机 + 图像传输端：`openmv/openmv_cam_stream.py` 采集 QVGA RGB565、
  JPEG 压缩后按 `docs/protocol.md` 的帧格式通过 UART/USB 发给上位机；
- 上位机 `FrameSource`（`OpenMvSerial` 类型）解帧、`cv::imdecode` 得到 BGR 帧；
- 图像流波特率与下位机输出波特率可分别设置：`--cam-baud`（图像流，默认取 `--baud`）与 `--baud`；
- 已用 720p→320x240 QVGA 模拟验证部署分辨率：检测率 97.8–100 %、抖动比 0.30–0.69、
  ≈0.4 ms/帧（见验证报告 §5）；
- 相机内参用 `--mode calibrate` 标定后写入配置（或用 OpenMV 默认内参，见 `docs/calibration.md`）。
- 历史 OpenMV MicroPython 脚本（`F:\github_hub\视觉代码\...\main_final.py`）只输出 blob 像素坐标，
  本模块是其上位机升级替代。

## 12. 限制与后续

- 目标尺寸未知时距离有比例误差（yaw/pitch 不受影响）；
- 圆特征在近乎正对时倾斜角/距离可观测性较弱，重投影精化后仍以视线角为主输出；
- 无 IMU 融合：载体角运动与目标运动在视觉上不可区分，`yaw_rate/pitch_rate` 是视线角速度；
- 后续可扩展：IMU 融合、目标运动模型（匀速/机动检测）、弹道补偿、多目标、神经网络检测。
