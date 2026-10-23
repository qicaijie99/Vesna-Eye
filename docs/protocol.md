# 串口协议

本文件定义两类串口协议：

1. **上位机 → 下位机**：跟踪结果包（19 字节定长，含 CRC）。
2. **OpenMV → 上位机**：图像帧流（JPEG over 串口，带帧头/长度/CRC）。

默认串口参数：`115200 8N1`（跟踪结果）；OpenMV 图像流建议 `3000000 8N1` 或 USB CDC。

---

## 1. 跟踪结果包（上位机 → 下位机）

固定 **19 字节**，小端序，`#pragma pack(1)`，定义见 `include/glowtrack/types.hpp::TrackPacket`。

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | u8 | `sof1` | 帧头 1 = `0xAA` |
| 1 | u8 | `sof2` | 帧头 2 = `0x55` |
| 2 | u8 | `version` | 协议版本 = 1 |
| 3 | u8 | `seq` | 递增序号（0–255 循环） |
| 4 | u8 | `flags` | bit0 valid、bit1 measured、bit2 predicted、bit3 acquiring、bit4 tracking |
| 5 | i16 | `yaw_q` | yaw，单位 `1e-4 rad`（±3.27 rad 可表示） |
| 7 | i16 | `pitch_q` | pitch，单位 `1e-4 rad` |
| 9 | i16 | `yaw_rate_q` | yaw 角速度，单位 `1e-3 rad/s` |
| 11 | i16 | `pitch_rate_q` | pitch 角速度，单位 `1e-3 rad/s` |
| 13 | u16 | `distance_mm` | 距离，mm，饱和到 65535 |
| 15 | u8 | `quality` | 置信度 0–100 |
| 16 | u8 | `reserved` | 保留，发 0 |
| 17 | u16 | `crc16` | CRC-16/CCITT-FALSE，覆盖字节 `[0, 17)` |

角度定义（与 `docs/architecture.md` 一致）：

- 相机坐标系 x 右 / y 下 / z 前；
- `yaw = atan2(t.x, t.z)`，右正；
- `pitch = -atan2(t.y, t.z)`，上正。

下位机使用示例（C）：

```c
typedef struct __attribute__((packed)) {
  uint8_t  sof1, sof2, version, seq, flags;
  int16_t  yaw_q, pitch_q, yaw_rate_q, pitch_rate_q;
  uint16_t distance_mm;
  uint8_t  quality, reserved;
  uint16_t crc16;
} track_packet_t;   // 19 bytes

static uint16_t crc16_ccitt(const uint8_t *data, int n) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < n; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* 在 UART 接收中断/任务里按 0xAA 0x55 找帧头，收满 19 字节后校验 CRC：
   crc16_ccitt(buf, 17) == buf[17] | (buf[18] << 8) */
float yaw   = (int16_t)(pkt.yaw_q)   * 1e-4f;   /* rad */
float pitch = (int16_t)(pkt.pitch_q) * 1e-4f;   /* rad */
```

**invalid 时的行为建议**：`flags.bit0 == 0` 表示本帧角度不可用（LOST/未跟踪），
下位机应保持上一次有效角度或进入搜索/保护策略；`predicted=1` 表示角度为外推值，
可用于短时维持（外推时间上限见 `max_miss_frames`）。

发送节奏：每处理一帧发一包（默认 30 Hz；若相机帧率更高则同频）。输出角已做
`latency_compensation_ms` 延时补偿（配置项，默认 0）。

---

## 2. OpenMV 图像帧流（OpenMV → 上位机）

上位机 `FrameSource` 类型 `OpenMvSerial` 使用这个格式。帧为 JPEG（RGB565 采集后 `img.compress()`）。

### 2.1 帧格式

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | u8 | `sof1` | `0xA5` |
| 1 | u8 | `sof2` | `0x5A` |
| 2 | u8 | `version` | = 1 |
| 3 | u8 | `flags` | 保留，0 |
| 4 | u8 | `seq` | 帧序号（0–255） |
| 5 | u16 | `width` | 图像宽（小端） |
| 7 | u16 | `height` | 图像高（小端） |
| 9 | u32 | `jpeg_len` | JPEG 字节数（小端） |
| 13 | u8[`jpeg_len`] | `payload` | JPEG 数据 |
| 13+len | u16 | `crc16` | CRC-16/CCITT-FALSE，覆盖 `version` 到 `payload` 末尾 |

固定头部 13 字节 + 负载 + 2 字节 CRC。

### 2.2 传输参数建议

| 场景 | 参数 |
| --- | --- |
| USB CDC（USB_VCP，推荐） | 320x240，JPEG quality 50–70，约 10–20 KB/帧，USB 全速 CDC 实测可达 20–30 fps |
| UART | 320x240，JPEG quality 50–60，UART 3 Mbaud（约 300 KB/s）→ 约 15–20 fps；6 Mbaud 更快 |
| UART 接线 | OpenMV UART3：P4=TX、P5=RX（以所用板卡引脚图为准），交叉接 USB-TTL 的 RX/TX，共地 |

上位机命令行里图像流与下位机输出的波特率可分别设置：`--cam-baud <rate>`（OpenMV 图像流，
默认取 `--baud`）与 `--baud <rate>`（下位机跟踪结果）。

**带宽估算**：每帧字节数 ≈ `jpeg_len + 15`。30 fps × 15 KB ≈ 450 KB/s，
UART 需要 ≥ 4.5 Mbaud（8N1 有效 10 bit/byte），因此高速场景优先 USB CDC。

### 2.3 OpenMV 端

见 `openmv/openmv_cam_stream.py`（MicroPython）：固定曝光/增益/白平衡，QVA 采集，JPEG 压缩，
按上述格式打包发送；同时通过 `openmv/openmv_cam_stream.py` 顶部参数选择 UART 或 USB。

### 2.4 上位机端

`FrameSource::create()` 的 `OpenMvSerial` 类型：读串口字节流 → 找 `A5 5A` 帧头 →
读 13 字节头 → 读 `jpeg_len` 负载 → 校验 CRC → `cv::imdecode` 得到 BGR 帧。
（实现见 `src/frame_source.cpp`；该源为可选的部署路径，离线验证用视频源。）

---

## 3. 标定文件格式（相机内参）

`glowtrack --mode calibrate` 输出 YAML（OpenCV FileStorage）：

```yaml
%YAML:1.0
---
image_width: 320
image_height: 240
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 2.47e+02, 0., 1.6e+02, 0., 2.47e+02, 1.2e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0., 0., 0., 0., 0. ]
rms_reprojection_error: 0.31
```

在跟踪配置中引用标定文件：

```yaml
camera:
  calibration_file: config/camera_calib.yaml
  # 其余 fx/fy/cx/cy/dist 会被标定文件覆盖
```

标定流程见 `docs/calibration.md`。
