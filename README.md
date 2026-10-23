# green_glow_tracker

RoboMaster 无主动动力滑翔飞行器 —— **绿色发光目标** 上位机视觉跟踪模块（C++17 / OpenCV / OpenMV）。

从相机图像中检测绿色发光目标（白色中心圆盘 + 绿色发光扩散片构成的目标灯），
用 **PnP** 解算目标三维位姿，用 **Kalman 滤波** 平滑/外推视线角，最终以
**yaw / pitch**（及角速度、距离、置信度）实时输出给下位机，用于解算气动舵面的控制姿态。

> 本仓库只包含 **上位机视觉模块**。下位机（飞控/舵机）的航姿解算与控制律不在本仓库范围内；
> 上位机通过 UART 按固定协议把目标视线角发送给下位机。

**当前状态：已实现并完成端到端验证** — 57/57 单元测试通过；两个提供视频 **100 % 检测、≥99.5 % 有效输出**；
720p 端到端 **~2 ms/帧**、QVGA **~0.4 ms/帧**；合成数据闭环 yaw/pitch 精度 **0.12–0.52°**。
详见 [`docs/validation_report.md`](docs/validation_report.md)。

---

## 1. 功能

| 模块 | 说明 |
| --- | --- |
| 绿色发光目标检测 | HSV 绿光分割 + 白盘孔洞轮廓椭圆拟合 + 绿色环带/白盘亮度校验 + ROI 预测跟踪 |
| PnP 位姿解算 | 白盘为已知半径圆：仿射一致 4 点 + `SOLVEPNP_IPPE` 初值 → 椭圆重投影 LM 精化 → 朝向+时序消歧 |
| 滤波 | 视线角 4 状态 CV Kalman（卷绕、卡方门限、机动自适应 Q、质量因子、延时补偿）+ 2 状态距离 Kalman |
| 跟踪状态机 | SEARCH → ACQUIRE → TRACK → PREDICT → LOST，ROI 预测、量测关联、丢帧外推、重捕 |
| 输出 | yaw/pitch（rad/deg）、角速度、距离、置信度、状态；CSV 日志 / 标注视频 / 19 字节串口协议包（CRC-16） |
| 相机标定 | 棋盘格标定（`--mode calibrate`），OpenMV H7 Plus 默认内参与标定文件覆盖 |
| OpenMV 接入 | OpenMV 端 MicroPython 推流脚本（JPEG over UART/USB）+ 上位机串口图像源 |
| 验证工具 | 合成数据生成器（已知真值）+ 指标/曲线评估器 + 一键验证脚本 |

坐标系约定（详见 [`docs/architecture.md`](docs/architecture.md)）：

- 相机坐标系（OpenCV）：x 右、y 下、z 前；
- `yaw = atan2(t.x, t.z)`，右正；`pitch = -atan2(t.y, t.z)`，上正；距离 = `||t||`；
- 目标位姿为 target → camera（`cv::solvePnP` 约定）。

## 2. 目录结构

```
green_glow_tracker/
├─ CMakeLists.txt
├─ config/                   # 相机内参 / 目标尺寸 / 检测 / PnP / 滤波 / 输出（OpenCV YAML）
│  ├─ video_720p_phone.yaml   # 两个验证视频（720x1280，fx≈960）
│  ├─ openmv_h7plus_qvga.yaml # 部署：OpenMV H7 Plus QVGA 320x240（fx≈247）
│  └─ synthetic_640x480.yaml  # 合成数据闭环验证
├─ include/glowtrack/        # 公共接口（types/config/detector/pose/filter/tracker/io/util）
├─ src/                      # 实现 + CLI（main.cpp）
├─ tools/                    # dev_detector（检测调参）、synth_dataset（合成数据）、eval_metrics（评估/绘图）
├─ tests/                    # 无第三方依赖的单元测试（57 用例）
├─ openmv/                   # OpenMV 端 MicroPython 推流脚本 + 说明
├─ docs/                     # 架构、串口协议、标定、目标侦察、验证报告与资产
├─ scripts/                  # 构建 / 单测 / 真实视频验证 / 合成数据验证
└─ data/                     # 输入视频、侦察产物、合成数据与标定图像（运行时生成）
```

## 3. 构建（Windows / MSVC）

依赖：Visual Studio 2019（MSVC v142，C++17）、CMake ≥ 3.16、OpenCV 4.10+（预编译包，
含 `x64/vc16/lib/OpenCVConfig.cmake` 与 `opencv_videoio_ffmpeg*.dll`）。

```powershell
# 默认在 D:\opencv410\opencv\build\x64\vc16\lib 查找 OpenCV，可用 -OpenCVDir 指定
powershell -ExecutionPolicy Bypass -File scripts/build.ps1

# 只构建某个目标（并行开发/验证时可用独立 build 目录，避免互相干扰）
powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -BuildDir build_dev -Targets glowtrack_tests
# 多个目标用逗号分隔
powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -BuildDir build_dev -Targets glowtrack,glowtrack_tests

# 单元测试
powershell -ExecutionPolicy Bypass -File scripts/run_tests.ps1
```

产物：`build/Release/glowtrack.exe`、`glowtrack_synth.exe`、`glowtrack_eval.exe`、
`glowtrack_devdet.exe`、`glowtrack_tests.exe`（OpenCV 运行库由 CMake 自动拷贝到输出目录）。

Linux 部署：`cmake -S . -B build -DOpenCV_DIR=<opencv>/lib/cmake/opencv4 && cmake --build build -j`。

## 4. 运行

```powershell
# 4.1 视频离线运行（CSV + 标注视频）
.\build\Release\glowtrack.exe --config config\video_720p_phone.yaml `
    --video data\videos\v1_green_target.mp4 `
    --csv results\v1_track.csv --overlay results\v1_overlay.mp4

# 4.2 部署：OpenMV 串口图像流 + 向下位机发送跟踪结果
#     OpenMV 图像流端口与下位机端口可以是不同设备/波特率
.\build\Release\glowtrack.exe --config config\openmv_h7plus_qvga.yaml `
    --openmv COM7 --cam-baud 3000000 `
    --serial COM8 --baud 115200 `
    --csv results\flight.csv

# 4.3 相机标定（棋盘格 9x6，方格 25mm）
.\build\Release\glowtrack.exe --mode calibrate --images data\calibration\chess `
    --cols 9 --rows 6 --square-mm 25 --out config\camera_calib.yaml --debug-dir data\calibration\debug
```

常用参数：`--start-frame` / `--max-frames` / `--print-every 0` / `--quiet` / `--latency-ms`，
完整列表见 `glowtrack --help`。

## 5. 输出

- 跟踪结果：`yaw`、`pitch`（rad/deg）、`yaw_rate`、`pitch_rate`、`distance_m`、`quality`、`state`。
- CSV（22 列）：状态、滤波角度/角速度、距离、质量、PnP 原始角度、检测椭圆参数等，便于离线分析与调参。
- 标注视频：ROI、白盘椭圆、绿光椭圆、PnP 目标坐标轴、视线十字、HUD（角度/距离/状态/帧率/耗时）。
- 串口协议：19 字节定长包 + CRC-16/CCITT，字段与下位机解析示例见 [`docs/protocol.md`](docs/protocol.md)。

## 6. 验证（详见 [`docs/validation_report.md`](docs/validation_report.md)）

| 验证 | 结果 |
| --- | --- |
| 单元测试 | **57/57 通过**（检测 16、PnP 9、Kalman 7、跟踪/配置/IO 等 25） |
| 真实视频 v1/v2 | 检测 **100 %**，有效输出 **99.6 % / 99.5 %**，≈2.0/≈1.8 ms/帧 |
| PnP 视线角 vs 椭圆中心交叉校验 | 偏差 std **0.019–0.044°** |
| 滤波抖动比（filtered/raw，720p） | **0.78/0.54**（v1）、**0.92/0.83**（v2） |
| QVGA 部署分辨率模拟 | 检测 97.8 %/100 %，抖动比 0.30–0.69，0.40–0.47 ms/帧 |
| 合成数据闭环（滤波后） | yaw/pitch RMSE **0.12–0.52°**；丢帧纯预测误差 yaw 1.03°、pitch 0.11° |
| 相机标定回归 | 合成棋盘格 RMS **0.29 px**（真实 fx=500 → 恢复 505） |

一键复现：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_validation.ps1 -BuildDir build
powershell -ExecutionPolicy Bypass -File scripts\run_synthetic_validation.ps1 -BuildDir build
```

## 7. 关键假设与限制

- 目标为**平面圆形特征**：白色中心盘为已知直径圆，`target.disc_diameter_mm` 需按实物卡尺测量
  （默认 50 mm）。该尺寸只影响**距离**输出；`yaw/pitch` 对目标模型整体缩放不敏感（理论上严格不变）。
- 验证视频的手机相机内参为估计值（26mm 等效，fx≈960）；真实部署请用 `--mode calibrate` 标定
  OpenMV 相机（默认内参 fx≈247 @QVGA）。
- 单圆 PnP 存在数学上的两解（镜像圆截面），依靠朝向约束 + 时序连续性在跟踪中消歧；
  长时间丢失后重捕时距离可能跳变，yaw/pitch 不受影响。
- 纯视觉、无 IMU：`yaw_rate/pitch_rate` 为视线角速度；需要目标运动预测时请由下位机融合载体 IMU。
- 本模块不出弹道/提前量、不做控制律。
- 更完整的验证数字与已知偏置见 [`docs/validation_report.md`](docs/validation_report.md) §8、§11。

## 8. 文档索引

- [`docs/architecture.md`](docs/architecture.md) — 架构、算法（检测/PnP/Kalman/状态机）、坐标约定
- [`docs/protocol.md`](docs/protocol.md) — 下位机串口协议 + OpenMV 图像帧流协议
- [`docs/calibration.md`](docs/calibration.md) — 相机标定、OpenMV 默认内参、目标尺寸标定
- [`docs/validation_report.md`](docs/validation_report.md) — 验证方法、指标、曲线、限制与复现命令
- [`docs/recon_target_analysis.md`](docs/recon_target_analysis.md) — 目标图像侦察（视频分析结论）
- [`openmv/README.md`](openmv/README.md) — OpenMV 端推流脚本与部署说明
