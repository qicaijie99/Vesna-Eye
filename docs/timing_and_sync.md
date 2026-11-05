# 时间戳与同步说明（供下位机软件对接）

本文说明 `green_glow_tracker` 上位机视觉模块中**时间戳的产生、使用与串口输出时序**，
以及下位机软件可以依赖/不能依赖的假设。配套协议见 `docs/protocol.md`。

> 结论速览
> - 上位机内部用 `std::chrono::steady_clock`（进程启动为 0 点）的单调时钟；视频/图像序列用媒体时间或帧号换算。
> - **当前串口协议不含任何时间戳字段**（19 字节：`seq` + 角度/角速度/距离/质量 + CRC）。
> - `seq` 是 8 位递增帧序号，只能用于丢包检测（256 帧回绕），不能当时钟。
> - 上位机有 `latency_compensation_ms`（默认 0）用于把输出角度**前推**，它是配置常量，不是实测延迟。
> - OpenMV 图像帧协议没有相机端时间戳；OpenMV 源的时间戳是"整帧接收 + JPEG 解码完成"的主机时间。

## 1. 各帧源的时间戳语义

| 帧源 | `timestamp_s` 含义 | 单调性处理 | 说明 |
| --- | --- | --- | --- |
| 视频文件 `--video` | `cv::CAP_PROP_POS_MSEC / 1000`（容器媒体时间）；取不到时用 `index / fps` | 若 `<=` 上次时间戳则强制 `+0.5/fps` | 与视频采集时刻对应，适合离线验证 |
| 图像序列 `--images` | `index / fps`，从 0 开始 | 天然单调 | 合成数据/离线验证 |
| USB 相机 `--camera` | `steady_clock` 从打开相机起算，`read()` 返回帧后取值 | 单调 | 含驱动缓冲延迟，不是曝光瞬间 |
| OpenMV 串口 `--openmv` | `steady_clock` 从打开串口起算，**整帧收完并通过 JPEG 解码后**取值 | 单调 | 含 UART/USB 传输 + 解码时间；相机端无时间戳字段 |

所有时间戳单位均为秒（`double`），只保证**同一进程内单调**，**不是绝对时间/UTC**。

## 2. 时间戳在流水线中的使用

1. `main.cpp` 从 `FrameSource::read()` 拿到 `timestamp`，原样传给 `TargetTracker::update()`；
2. tracker 计算 `dt`：
   - 仅当 `timestamp > last_timestamp` 时 `dt = timestamp - last_timestamp`；
   - `NaN`/非正数 → `dt = 0`；`dt` 上限钳制为 **0.5 s**（防帧间隔异常导致滤波发散）；
3. `dt` 用于两个 Kalman 滤波器的 `predict(dt)`，然后做量测更新；
4. 输出 `TrackOutput.timestamp_s` = **输入帧的时间戳**（不是串口发送时刻），CSV 逐帧记录该值；
5. 延时补偿：`kalman.latency_compensation_ms`（配置，默认 `0`）→ 输出前调用
   `predictAhead(latency_s)`，把 yaw/pitch/角速度前推到"预计使用时刻"；
   该值应设为**从帧时间戳到执行机构使用时刻的端到端延迟**（不含下位机自身控制周期）。

## 3. 串口协议现状（19 字节，无时间戳）

| 字段 | 大小 | 说明 |
| --- | --- | --- |
| `sof1/sof2` | 2 B | `0xAA 0x55` |
| `version` | 1 B | 1 |
| `seq` | 1 B | 帧序号，0–255 循环 |
| `flags` | 1 B | bit0 valid、bit1 measured、bit2 predicted、bit3 acquiring、bit4 tracking |
| `yaw_q` / `pitch_q` | 2+2 B | 1e-4 rad |
| `yaw_rate_q` / `pitch_rate_q` | 2+2 B | 1e-3 rad/s |
| `distance_mm` | 2 B | mm，饱和 u16 |
| `quality` | 1 B | 0–100 |
| `reserved` | 1 B | 0 |
| `crc16` | 2 B | CRC-16/CCITT-FALSE |

时序行为：

- **每处理完一帧发一包**（不是固定频率定时器）；包间隔 = 相机帧间隔 + 处理/传输抖动；
- `valid=false` 时也会发（`flags.bit0=0`），便于下位机保持状态/保护；
- 包内没有时间戳、处理耗时、帧率、丢包计数；
- 传输耗时参考（8N1，含起止位）：
  - 115200 bps → 19 B ≈ **1.65 ms**
  - 921600 bps → 19 B ≈ 0.21 ms
  - 3 Mbps → 19 B ≈ 0.063 ms
- 离线视频回放时**没有实时节拍**（处理多快就发多快，可能 >100 包/秒），只有 `--display` 会按视频帧率等待。

## 4. 端到端延迟构成（量级参考，需实测）

```text
目标曝光/发光
  → OpenMV 曝光（配置固定，例如 8 ms）+ 读出/JPEG 压缩
  → UART/USB 传输（QVGA JPEG 约 10–20 KB；3 Mbps UART 约 30–55 ms，USB CDC 更快）
  → 上位机接收 + JPEG 解码（此之后才取帧时间戳）
  → 检测 + PnP + Kalman（实测约 2 ms @720p / 0.44 ms @QVGA）
  → 串口发送（19 B，1.65 ms @115200）
  → 下位机接收中断/解析 + 控制周期
```

总计可能在 **10–60 ms** 量级（取决于图像传输方式），需按实际链路测量。
`latency_compensation_ms` 只能补偿其中"可预测的固定部分"，且不包含下位机自身延迟。

## 5. 下位机对接建议

### 5.1 短期（不改协议）

1. 用 UART 接收完成中断/任务打**到达时间戳**，作为该包的时间基准；
2. 用 `seq` 做丢包检测：相邻包 `seq` 差 1 为正常；差 >1 为丢包；差 255 可能是回绕或长间隔
   （8 位序号每 256 帧回绕一次，30 fps 下约 8.5 s）；
3. 控制周期内使用最新包，并按实测总延迟做固定补偿：
   - 上位机侧：`kalman.latency_compensation_ms`（建议 = 相机传输 + 主机处理延迟）；
   - 下位机侧：接收/解析/控制周期延迟；
4. **不要假设固定 30 Hz**：包间隔随帧率、处理时间、图像传输抖动变化；
5. `flags.bit2=1`（predicted）表示角度为丢帧外推值，可短时使用；
   `flags.bit0=0` 表示本帧角度不可用，应保持上次有效值或进入保护；
6. 联调注意：用视频文件回放时串口包速率远高于实时，不能用来测实时延迟。

### 5.2 中期（建议扩展协议）

推荐二选一或同时做：

- **方案 A：包内加时间戳**（19 → 25 B，协议 v2）
  - `uint32 host_time_ms`：上位机 `steady_clock` 毫秒（进程启动为 0 点）；
  - `uint16 proc_ms`：本帧检测+PnP+滤波耗时（可选，用于延迟诊断）；
  - 下位机记录到达时间，用多包做 offset/drift 估计，实现自适应延迟补偿。
- **方案 B：周期同步包**
  - 例如 1 Hz 发送 `0xAA 0x56 | host_time_ms(u32) | crc16`；
  - 下位机用到达时间对 `host_time_ms` 做线性拟合，估计时钟偏移与漂移；
  - 正常跟踪包仍保持 19 B 不变（兼容现有下位机）。
- **OpenMV 帧协议加相机时间戳**（建议一并做）
  - 在 13 B 帧头后增加 `uint32 capture_ms`（OpenMV `time.ticks_ms()`）；
  - 上位机可计算"曝光→处理完成"的真实延迟，并把该延迟写进跟踪包，显著减小抖动影响。

### 5.3 长期

- 硬件 PPS/触发同步（相机与飞控共时钟），或使用带硬件时间戳的链路；
- 上位机按固定控制频率（如 100 Hz）插值/预测输出，而不是"来一帧发一帧"。

## 6. 当前实现的明确限制

- 无绝对时间（UTC）语义，进程重启后时间戳从 0 重新开始；
- OpenMV 源的时间戳是"主机收到完整帧"的时间，**不是相机曝光时刻**（帧协议无相机时间戳）；
- 串口包内无时间戳，下位机只能依赖到达时间 + 固定补偿；
- 无固定发送节拍、无重传/冗余发送；
- 视觉线程与串口发送在同一循环内串行执行，发送失败会停发（`serial` 句柄被重置）。

## 7. 相关代码位置

| 内容 | 文件 |
| --- | --- |
| 各帧源时间戳 | `src/frame_source.cpp`（`VideoFileSource` / `ImageSequenceSource` / `CameraSource` / `OpenMvSerialSource`） |
| `dt` 计算与钳制、延时补偿 | `src/tracker.cpp`（`kMaxDtSeconds = 0.5`、`predictAhead`） |
| 主循环时间戳传递、逐帧发送 | `src/main.cpp` |
| 串口包字段与量化 | `src/serial_port.cpp`、`include/glowtrack/types.hpp`（`TrackPacket`） |
| 协议定义 | `docs/protocol.md` |
| 配置项 | `config/*.yaml` → `kalman.latency_compensation_ms`、`nominal_fps`、`output.serial_baud` |
