# 相机标定与 OpenMV 内参

## 1. 为什么要标定

PnP 依赖相机内参 `fx, fy, cx, cy`（及畸变系数）。内参误差会直接影响：

- `yaw/pitch` 的绝对角度标定（f 偏差按比例影响角度大小）；
- 距离估计（f 偏差近似成正比）；
- 椭圆/重投影误差的几何一致性。

部署前应用棋盘格标定一次，把结果写入配置。

## 2. OpenMV H7 Plus 默认内参（标定前使用）

OpenMV Cam H7 Plus：OV5640 传感器（2592x1944，像元 1.4 µm）+ 2.8 mm 焦距镜头。

```text
全分辨率：f_px = 2.8 mm / 1.4 µm = 2000 px（水平 FOV ≈ 66°）
QVGA 320x240 与全分辨率同视场：f_px = 2000 / (2592/320) ≈ 247 px
→ fx = fy ≈ 247, cx = 160, cy = 120
```

见 `config/openmv_h7plus_qvga.yaml`。若使用不同分辨率/镜头：

```text
fx = fy ≈ 2.8mm / 1.4um × (输出宽度 / 2592)  （同一视场缩放，近似）
```

## 3. 棋盘格标定步骤

1. 准备棋盘格（推荐 9x6 内角点、方格 25 mm，打印后贴在硬板上保持平整）；
2. 用部署相机（OpenMV / 手机 / USB 相机）从不同距离、不同角度拍摄 **15–30 张**棋盘格图像，
   覆盖画面中心与四角，包含倾斜姿态；
3. 图像放到目录（png/jpg），例如 `data/calibration/chess/`；
4. 运行标定：

```powershell
.\build\Release\glowtrack.exe --mode calibrate `
    --images data\calibration\chess `
    --cols 9 --rows 6 --square-mm 25 `
    --out config\camera_calib.yaml `
    --debug-dir data\calibration\debug
```

`--debug-dir` 会输出角点检测的可视化图，用于快速剔除失败图片。

5. 看输出：

```text
Calibration: 18 views, 320x240, RMS reprojection = 0.28 px
Wrote config\camera_calib.yaml
```

RMS 建议 < 0.5 px；过大说明图像模糊/棋盘不平/角点检测错误。

6. 在跟踪配置中引用（`camera.calibration_file`），或在部署时把 `fx/fy/cx/cy/dist`
   直接抄进 YAML（`camera:` 段）。

## 4. 目标尺寸标定

用卡尺实测目标中央白色圆盘直径，写入：

```yaml
target:
  disc_diameter_mm: 50.0      # 白盘直径（PnP 圆特征半径 = 该值/2）
  glow_outer_diameter_mm: 62.0  # 绿色发光片外径（检测校验/调试用）
```

> 该尺寸只影响**距离**输出；`yaw/pitch` 对目标模型整体缩放不敏感（严格不变）。
> 因此即使暂时没有实测值，也可以先用 50 mm 默认值部署角度输出，之后再校准距离。

## 4.5 标定工具回归验证（已通过）

仓库内 `data/calibration/chess_synth/` 提供 18 张合成棋盘格图像（真实内参 fx=fy=500，
畸变 k1=-0.05/k2=0.03，9x6 角点、25 mm 方格），用于回归验证标定工具：

```powershell
.\build\Release\glowtrack.exe --mode calibrate --images data\calibration\chess_synth `
    --cols 9 --rows 6 --square-mm 25 --out results\calib_synth.yaml
# Calibration: 16 views, 640x480, RMS reprojection = 0.2903 px
# 恢复内参 fx=505.08 / fy=504.68 / cx=316.47 / cy=240.11（真值 fx=500）
```

合成渲染下内参恢复误差约 1 %、RMS 0.29 px，说明角点检测/标定链路正确；
真实相机请按 §3 采图标定并以 RMS < 0.5 px 为目标。

## 5. 标定质量自检

- 用标定后的内参对一段已知棋盘格视频重投影，检查角点误差（`--debug-dir` 图）；
- 合成验证（`tools/synth_dataset.cpp` + `glowtrack_eval`）可验证 PnP 在已知内参下的角度精度；
- 实际视频验证中可用"PnP 视线角 vs 白盘椭圆中心视线角"的偏差做交叉检查
  （两者应只差 PnP 精化量，通常 < 0.5°）。
