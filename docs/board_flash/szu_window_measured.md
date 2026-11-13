# 照搬深大"读窗口"方案：DART H7 实测帧率

日期：2026-11-13
目标：把深大 OpenMV 代码里的**OV5640 读出窗口（readout window）+ 低 VTS** 方案移植到
DART Control H7 固件，并在板上实测采集帧率与端到端帧率。

## 1. 固件改动（本板）

新增维护命令（`Task/src/tsk_usb_console.cpp`、`Task/src/tsk_video_test.cpp`）：

| 命令 | 作用 |
| --- | --- |
| `CAMERA RD <reg>` | 读任意 OV5640 寄存器（hex），返回 `Rxxxx=xx` |
| `CAMERA WR <reg> <val>` | 写任意 OV5640 寄存器（hex） |
| `CAMERA WIN <hs> <vs> <he> <ve> <hts> <vts> <hoff> <voff>` | 按 OpenMV `set_framesize()` 的方式写读窗口 + 时序 + 偏移（8 个 16-bit 寄存器对） |
| `BENCH` | 连续拍 20 帧、不发送载荷，返回 `avg_ms/min_ms/max_ms`（采集帧率） |
| `CAMERA DCMI` | 打印 `DCMI_ERR/SR/CR/NDTR/DMA_CR`，用于定位 DCMI 错误 |

另试验过把默认分辨率切到 320×240（为匹配 640×480 读窗口 ratio=1），该实验后已回滚到
**160×120**；最终有效配置是 160×120 输出 + 320×240 读窗口（2× 降采样，ratio 1）。

烧录：`BOOT` → ROM DFU → WSL2 + usbipd + `dfu-util -a 0 -s 0x08000000:leave`
（同 `board_flash_report.md`）。当前固件 83,716 B Flash。

## 2. 测量方法

- **采集帧率**：`BENCH` 命令在板上连续捕获 20 帧（`Camera::Snapshot` 模式），统计平均单帧
  周期 `avg_ms`；不含 USB 传输，但包含每帧 DCMI 启动/等待 VSYNC 的开销（约 4–9 ms）。
- **端到端帧率**：`SNAP1`（1bpp 绿掩膜 160×120 = 2400 B）+ USB 传输，主机侧计时
  `total = 命令→载荷结束`。
- 每个配置先 `CAMERA RESET`（恢复默认 1936/1088），再 `CAMERA WIN ...`，等待 1 s 再测。
- 读窗口寄存器值按 OpenMV 驱动公式计算：
  `sensor_ws/ws = ((2592-w)/4 + x/2)*2`、`sensor_hs = ((1944-h)/4 - y/2)*2 + 8`，
  `HTS = max(2*w_out+8, 1438)+160+252`、`VTS = max(h/2+24, 248)`。

## 3. 实测结果（160×120 RGB565，默认 PCLK ≈14.7 MHz）

### 3.1 640×480 读窗口（FOV ≈24.7% 线性，HTS=1936）

| VTS | 采集 avg | 采集帧率 | 1bpp 端到端 |
| ---: | ---: | ---: | ---: |
| 800 | 128 ms | 7.8 fps | — |
| 612 | 97 ms | 10.3 fps | 11.8 fps |
| 400 | 64 ms | 15.6 fps | 18.7 fps |
| 300 | 49 ms | 20.4 fps | 18.0 fps |
| 268 | 44 ms | 22.7 fps | 21.8 fps |
| **248** | **40 ms** | **25.0 fps** | **22.3 fps** |
| 200 / 160 | — | ❌ `ERR BENCH CAPTURE` | — |

### 3.2 320×240 读窗口（FOV ≈12.3% 线性，HTS=1936）

| VTS | 采集 avg | 采集帧率 | 1bpp 端到端 |
| ---: | ---: | ---: | ---: |
| 1088 / 984 / 800 | — | ❌（大 VTS 与窗口组合异常） | — |
| 612 | 97 ms | 10.3 fps | — |
| 400 | 64 ms | 15.6 fps | — |
| 300 | 48 ms | 20.8 fps | — |
| 248 | 40 ms | 25.0 fps | — |
| 200 | 32 ms | 31.3 fps | — |
| 160 | 26 ms | 38.5 fps | — |
| **144** | **23 ms** | **43.5 fps** | **26.4 fps**（total 38 ms） |

### 3.3 HTS 下限

VTS=144、读窗口 320×240 时：HTS 1936 ✅（26 ms）、1896 ✅（26 ms）、
**1600 / 1438 ❌ `ERR BENCH CAPTURE`** → 本板/本模式 HTS 不能低于约 1896。

### 3.4 与 PLL ×2 组合

`CAMERA PLL2` + 窗口、或先窗口再直接写 `0x3036=0xC0/0x3037=0x13`，
两种顺序都 **❌ `ERR BENCH CAPTURE`**：PCLK 提到 ≈29.4 MHz 后该窗口时序不再被传感器接受。
因此"窗口 + PLL×2"不可用。

### 3.5 图像验证

| 图 | 说明 |
| --- | --- |
| `szu_rgb_full.png` | 默认全幅读出（FOV 100%），曝光正常 |
| `szu_rgb_win640.png` | 640×480 读窗口（FOV ≈25%），画面为同场景中心区域放大 |
| `szu_rgb_win320.png` | 320×240 读窗口（FOV ≈12%），进一步放大、曝光变暗 |

三张图同一场景、依次放大，证明读窗口生效且图像方向/内容正确。
（本次实验场景没有绿色发光目标，所以 1bpp 掩膜为全 0、green8 均值 <1，属正常；帧率与图像有效性不受影响。）

## 4. 对比

| 配置 | FOV（线性） | 采集 | 1bpp 端到端 |
| --- | ---: | ---: | ---: |
| 旧基线：全幅读出 VTS=1088 | 100% | ≈7 fps（140–176 ms） | 7.36 fps |
| 深大 OpenMV：640×480 窗口、VTS=268、PCLK 80 MHz | 24.7% | 4.8 ms ≈ 208 fps（时序上限，未实测 loop） | 未记录 |
| **本板：640×480 窗口、VTS=248、PCLK 14.7 MHz** | **24.7%** | **25.0 fps** | **22.3 fps** |
| **本板：320×240 窗口、VTS=144、PCLK 14.7 MHz** | **12.3%** | **43.5 fps** | **26.4 fps** |

> 本板 PCLK 只有深大的 1/5.4（14.7 vs 80 MHz），所以同样窗口下采集慢 5–7 倍；
> 通过把窗口再缩小到 320×240（VTS 144），本板采集达到 43.5 fps，但仍低于深大。

## 5. 结论

1. 深大方案的**关键机制（读窗口 + 低 VTS）在本板已跑通**：全幅 1088 行 → 320×240 窗口
   144 行，采集从 ≈7 fps 提到 **43.5 fps**（实测，`BENCH` 20 帧平均），1bpp 端到端
   **26.4 fps**，分别是旧基线的 **6.2× / 3.6×**。
2. 640×480 窗口（FOV 25%）下实测 **25 fps 采集 / 22.3 fps 端到端**，视场代价更小。
3. 仍达不到深大 60 fps 量级的根本原因是 **PCLK**：本板 14.7 MHz（PLL×2 后 29.4 MHz，
   但窗口+PLL×2 会失败），深大板 80 MHz。
4. 进一步提速的可能方向：
   - 采集侧：`HTS` 不能低于 ~1896，VTS 已压到 144；剩余开销是每帧 DCMI 启动/等 VSYNC
     （≈4–6 ms），改 **DCMI 连续模式 + 双缓冲**可再省一点（预计采集 46–50 fps）；
   - 传输侧：1bpp 2400 B 目前要 ≈10 ms（5×512 B 分块 + 忙等），加大分块/去掉等待可到
     2–3 ms，端到端有望从 26.4 fps 提到约 35 fps；
   - 视觉侧：任务实际只需要绿点中心，可只传 ROI 或板上椭圆（几十字节），把端到端拉高。
5. 当前板子已恢复默认全幅时序（`CAMERA RESET` 后状态 READY）；读窗口需每次上电/复位后
   重新下发（`CAMERA WIN ...`），若要量产应固化到相机初始化里按模式选择。

## 6. 复现命令（COM14，115200）

```text
PING
CAMERA RESET
CAMERA WIN 03C8 02E0 0657 04C7 0790 00F8 0004 0002   # 640x480 窗口, HTS 1936, VTS 248
BENCH                                                # -> avg ~40 ms
SNAP1                                                # -> 1bpp 端到端 ~22 fps

CAMERA RESET
CAMERA WIN 0468 0358 05B7 044F 0790 0090 0004 0002   # 320x240 窗口, HTS 1936, VTS 144
BENCH                                                # -> avg ~23 ms
SNAP1                                                # -> 1bpp 端到端 ~26 fps

CAMERA RESET                                        # 恢复全幅
```

## 7. 相关日志与证据

- `docs/board_flash/szu_window_bisect.log`：窗口/VTS/HTS 扫描原始输出
- `docs/board_flash/szu_window_640.log`：640×480 窗口扫描
- `docs/board_flash/szu_window_final2.log`：最终配置 + PLL×2 对照
- `docs/board_flash/szu_window/*.png`：全幅/640/320 窗口对比图
- 主机脚本：`_recon/szu_window_bisect.py`、`_recon/szu_window_640.py`、
  `_recon/szu_window_final2.py`、`_recon/szu_rgb_capture.py`、`_recon/flash_fw.ps1`
