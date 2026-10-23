# green_glow_tracker

RoboMaster 无主动动力滑翔飞行器 —— **绿色发光目标** 上位机视觉跟踪模块（C++ / OpenCV / OpenMV）。

从相机图像中检测绿色发光目标（白色圆盘 + 绿色发光扩散片构成的目标灯），
用 **PnP** 解算目标三维位姿，用 **Kalman 滤波** 平滑并外推视线角，最终以
**yaw / pitch**（及角速度、距离、置信度）实时输出给下位机，用于解算气动舵面的控制姿态。

> 本仓库只包含 **上位机视觉模块**。下位机（飞控 / 舵机控制）的航姿解算与控制律不在本仓库范围内；
> 上位机通过串口按固定协议把目标视线角发送给下位机。

---

## 1. 功能

| 模块 | 说明 |
| --- | --- |
| 绿色发光目标检测 | HSV/LAB 颜色分割 + 连通域 + 中心白盘椭圆拟合 + 绿色环形校验 + ROI 预测跟踪 |
| PnP 位姿解算 | 白盘圆为已知半径的圆特征，IPPE 初值 + 椭圆重投影非线性精化，含二义性消除 |
| 滤波 | 视线角 (yaw, pitch, 角速度) 与距离分别用 Kalman 滤波，含卡方门限、丢帧外推、延时补偿 |
| 输出 | yaw/pitch（rad 与 deg）、角速度、距离、置信度、跟踪状态；CSV 日志 / 标注视频 / 串口协议包 |
| 相机标定 | 棋盘格标定工具（`--mode calibrate`），按相机型号提供默认内参 |
| OpenMV 接入 | OpenMV H7 Plus（OV5640 + 2.8mm 镜头）默认内参、MicroPython 推流脚本、串口帧协议 |
| 验证工具 | 合成数据闭环验证（已知真值精度评估）+ 真实视频验证（检测率、PnP 自洽性、平滑度） |

坐标系约定（详见 `docs/architecture.md`）：

- 相机坐标系：x 右、y 下、z 前（OpenCV 约定）。
- yaw = atan2(t.x, t.z)，右为正；pitch = -atan2(t.y, t.z)，上为正；距离 = ||t||。
- 目标位姿为 target → camera（OpenCV solvePnP 约定）。

## 2. 目录结构

```
green_glow_tracker/
├─ CMakeLists.txt
├─ config/                   # 运行配置（相机内参 / 目标尺寸 / 检测 / PnP / 滤波 / 输出）
├─ include/glowtrack/        # 公共头文件（接口）
├─ src/                      # 实现：detector / pose_estimator / kalman_filter / tracker / io / main
├─ tools/                    # dev_detector / synth_dataset / eval_metrics
├─ tests/                    # 无第三方依赖的单元测试
├─ openmv/                   # OpenMV 侧：推流脚本 + 说明
├─ docs/                     # 架构、串口协议、标定、验证报告
├─ scripts/                  # 构建 / 测试 / 验证脚本
└─ data/                     # 输入视频（手动放入）与合成数据
```

## 3. 构建（Windows / MSVC）

依赖：

- Visual Studio 2019（MSVC v142，C++17）
- CMake ≥ 3.16
- OpenCV 4.10+（预编译包，含 `x64/vc16/lib/OpenCVConfig.cmake` 与 `opencv_videoio_ffmpeg*.dll`）

```powershell
# 默认在 D:\opencv410\opencv\build\x64\vc16\lib 查找 OpenCV，可 -OpenCVDir 指定
powershell -ExecutionPolicy Bypass -File scripts/build.ps1

# 只构建某个目标（并行开发时用独立 build 目录，避免互相干扰）
powershell -ExecutionPolicy Bypass -File scripts/build.ps1 -BuildDir build_dev -Targets glowtrack_tests

# 单元测试
powershell -ExecutionPolicy Bypass -File scripts/run_tests.ps1
```

构建产物：`build/Release/glowtrack.exe`、`glowtrack_synth.exe`、`glowtrack_eval.exe`、`glowtrack_tests.exe`
（OpenCV 运行库会由 CMake 自动拷贝到输出目录）。

Linux（部署平台）同样可构建：`cmake -S . -B build -DOpenCV_DIR=<opencv>/lib/cmake/opencv4 && cmake --build build -j`。

## 4. 运行

```powershell
# 对视频运行（CSV + 标注视频）
.\build\Release\glowtrack.exe --config config\video_720p_phone.yaml `
    --video data\videos\v1.mp4 --csv results\v1_track.csv --overlay results\v1_overlay.mp4

# 实时：串口接收 OpenMV 图像流（JPEG over UART），并向下位机发送跟踪结果
.\build\Release\glowtrack.exe --config config\openmv_h7plus_qvga.yaml `
    --openmv COM7 --baud 921600 --serial COM8 --baud 115200

# 注意：--openmv 与 --serial 可指向同一块板的不同虚拟串口 / 或分开的设备；
# 两个 --baud 需要分别用 --baud（通用）或配置文件写出具体速率（见 docs）。

# 相机标定（棋盘格）
.\build\Release\glowtrack.exe --mode calibrate --images data\calibration\chess `
    --cols 9 --rows 6 --square-mm 25 --out config\camera_calib.yaml
```

常用参数：`--start-frame` / `--max-frames` / `--print-every 0` / `--quiet`（见 `glowtrack --help`）。

## 5. 输出

- 跟踪结果：`yaw`、`pitch`（rad/deg）、`yaw_rate`、`pitch_rate`、`distance_m`、`quality`、`state`。
- CSV 列：`frame,timestamp_s,state,detected,valid,measured,predicted,yaw_deg,pitch_deg,...`
- 串口协议包（19 字节，CRC-16/CCITT）：见 `docs/protocol.md`。

## 6. 验证

- **合成数据闭环验证**：已知目标真值位姿 → 渲染图像序列 → 跑完整流水线 → 统计 yaw/pitch/距离误差、
  滤波前后抖动、丢帧外推误差（`tools/synth_dataset.cpp` + `tools/eval_metrics.cpp`）。
- **真实视频验证**：两个提供视频全程跟踪，统计检测率、跟踪连续性、PnP 重投影误差、
  PnP 视线角与椭圆中心视线角交叉一致性、滤波平滑度；输出标注视频与曲线。
- 详细结果见 `docs/validation_report.md`。

## 7. 关键假设与限制

- 目标为**平面圆形特征**（白色中心盘为已知直径的圆）；`disc_diameter_mm` 需按实物测量后写入配置。
  该尺寸只影响距离输出，yaw/pitch 对目标尺寸缩放不敏感（理论上严格不变）。
- 视频验证所用手机相机内参为估计值（fx≈960 @720x1280）；真实部署请用 `--mode calibrate` 标定 OpenMV。
- 仅输出视线角与距离，不包含弹道/提前量、不包含下位机姿态控制；纯视觉，无 IMU 融合。
- 强逆光/完全遮挡/出画时依赖 Kalman 外推，超过 `max_miss_frames` 后进入 LOST。

## 8. 参考

- `docs/architecture.md` 架构与算法细节
- `docs/protocol.md` 下位机串口协议
- `docs/calibration.md` 相机标定与 OpenMV 内参
- `docs/validation_report.md` 验证方法与结果
