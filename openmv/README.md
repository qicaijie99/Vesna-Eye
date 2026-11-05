# OpenMV 侧说明

本目录只负责 **图像采集与传输**。目标检测、PnP、Kalman 滤波、yaw/pitch 输出全部在
上位机 C++ 模块 `Vesna-Eye` 中完成（本仓库交付物）。

## 1. 文件

- `openmv_cam_stream.py`：OpenMV Cam H7 Plus（OV5640 + 2.8mm 镜头）MicroPython 脚本。
  固定曝光/增益/白平衡，QVGA 320x240 采集，JPEG 压缩后按 `docs/protocol.md` 第 2 节
  的帧格式发送给上位机。顶部 `USE_USB` 切换 USB CDC（推荐）或 UART。

## 2. 部署步骤

1. 用 OpenMV IDE 把 `openmv_cam_stream.py` 保存到 OpenMV 并运行（或存为 `main.py` 开机自启）；
2. 连接方式二选一：
   - **USB**（推荐）：OpenMV 的 USB CDC 端口直接出现在上位机（如 `COM7`），
     上位机运行：
     ```powershell
     .\build\Release\glowtrack.exe --config config\openmv_h7plus_qvga.yaml `
         --openmv COM7 --baud 3000000 --serial COM8 --csv results\flight.csv
     ```
     （`--openmv` 是图像流端口；`--serial` 是发给下位机的端口，按实际接线填写）
   - **UART**：OpenMV UART3（H7 Plus：P4=TX、P5=RX）交叉接 USB-TTL 模块，
     `USE_USB = False`、`UART_BAUD` 与上位机 `--baud` 一致；
3. 相机标定（可选但推荐）：见 `docs/calibration.md`；
4. 目标白盘直径实测后写入 `config/openmv_h7plus_qvga.yaml` 的 `target.disc_diameter_mm`。

## 3. 性能与参数建议

| 参数 | 建议 | 说明 |
| --- | --- | --- |
| 分辨率 | QVGA 320x240 | 与全分辨率同视场；检测/解算耗时最低 |
| `JPEG_QUALITY` | 50–70 | 越小带宽越低、图像越糊；60 是 QVGA 的常用折中 |
| UART 波特率 | 3–6 Mbaud | 3 Mbaud 约 15–20 fps @ QVGA q60；USB CDC 更快 |
| 曝光 | 固定（`AUTO_EXPOSURE_US`） | 防止目标亮度突变导致颜色阈值漂移 |
| 增益/白平衡 | 固定 | 同上，保证绿色提取稳定 |

> 绿色发光目标在画面中只占很小区域，**不需要高分辨率**；QVGA 下白盘直径约 15–60 px，
> 对应 PnP 角度精度仍可满足 RM 控制需求（合成验证：0.3–0.6°，见验证报告）。

## 4. 与上位机的接口约定

- 帧格式：`A5 5A | ver | flags | seq | width u16 | height u16 | jpeg_len u32 | jpeg | crc16`
  （CRC-16/CCITT-FALSE，覆盖 ver..payload），详见 `docs/protocol.md`。
- 内参默认值（QVGA）：`fx=fy≈247, cx=160, cy=120`（2.8mm 镜头 + OV5640 1.4µm 像元），
  建议用棋盘格标定替换。
- 上位机 `FrameSource` 类型 `OpenMvSerial` 负责解帧（`src/frame_source.cpp`，可选部署路径）。
- 历史 OpenMV 脚本（`F:\github_hub\视觉代码\...\main_final.py`）只发 blob 坐标，
  本模块替代它：图像流 + 上位机 PnP/滤波。
