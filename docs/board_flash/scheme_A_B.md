# 采集提速方案 A / B（定义、上板结果与标注）

日期：2026-11-12
背景：目标 60 fps；当前 OV5640 320×240 采集 ≈7 fps，主要受 HTS/VTS/PCLK 限制。
真实场景曝光固定，因此可关闭自动曝光，使用固定曝光 + 固定增益。

## 方案定义

### 方案 A：降分辨率（QQVGA 160×120，全视场）
- 传感器输出改为 **160×120**（OV5640 QQVGA），视场不变；
- 每帧 RGB565 = 38,400 B（原 153,600 B 的 1/4）；
- 固定曝光（AEC/AGC 手动）+ 固定增益；
- DCMI/缓冲区/协议头同步改为 160×120；
- 优点：实现简单，带宽/算力都降 4 倍；
- 缺点：白盘像素少一半，PnP 精度下降；帧率仍受 HTS/VTS 限制。

### 方案 B：ROI 窗口（160×120 窗口，保持 320×240 视场）
- 传感器仍按 320×240 全视场工作，但通过 OV5640 **窗口寄存器**只输出目标附近的
  160×120 区域：
  - `0x3800/01` X 起点、`0x3802/03` Y 起点；
  - `0x3804/05` X 终点、`0x3806/07` Y 终点；
  - `0x3808/09` 输出宽、`0x380A/0B` 输出高；
- DCMI 只搬 160×120（38,400 B），USB 只传这一块；
- ROI 来源：板上用绿色光斑质心做简单跟踪，或上位机把预测 ROI 回传给板子；
- 优点：**目标像素密度不降**（白盘还是 320×240 下的尺寸），PnP 精度基本不变；
- 缺点：需要 ROI 跟踪/回传，实现比 A 复杂；帧率同样受 HTS/VTS/PCLK 限制。

> 两者都不改变“传感器时序”这一根本瓶颈；要 60 fps，必须同时改 PLL/HTS/VTS
> 或使用支持高帧率的传感器模式。

## 方案 A 上板结果（已实现）

- 固件：默认 `Resolution::R160x120`、`PixelWidth=160/Height=120`；
  QQVGA 表内写入固定曝光：`0x3503=0x03`（AEC+AGC 手动）、
  `0x3500/01/02=0x0001F4`（500 lines）、增益 `0x350A/B=0x00F8`。
- `PARAMS` 确认：`CAM_WIDTH=160 CAM_HEIGHT=120 FRAME_BYTES=38400`。
- 实测（默认 HTS=1936/VTS=1088/PCLK≈14.7 MHz）：

| 帧 | 采集耗时 |
| --- | ---: |
| 第 1 帧 | 0.120 s |
| 后续帧 | 0.348–0.349 s |
| 平均 | **0.303 s → 约 3.3 fps** |

- 固定曝光后不再出现“暗光 AEC 拉到 1.2 s”的问题，但帧率仍由 VTS/HTS 决定，
  且 QQVGA 下 DCMI 同步开销使实测比理论 7 fps 更慢。
- 尝试在 QQVGA 表内改 HTS/VTS（800/306、800/612）→ **FRAME TIMEOUT**；
  说明当前 PLL/PCLK 与这些 HTS/VTS 不匹配，必须按 OV5640 数据手册重配 PLL。

## 方案 B 实现标注（待上板）

代码位置（`Task/src/tsk_camera.cpp` / `ov5640.c`）建议：

```c
/* Scheme B: ROI window (160x120) inside the 320x240 field of view. */
/* Window registers: 0x3800/01 X start, 0x3802/03 Y start,
   0x3804/05 X end, 0x3806/07 Y end, 0x3808/09 out width, 0x380A/0B out height. */
static void ApplyRoiWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    /* x/y = ROI top-left in the 320x240 sensor coordinate system. */
    WriteReg16(0x3800, x);
    WriteReg16(0x3802, y);
    WriteReg16(0x3804, x + w - 1U);
    WriteReg16(0x3806, y + h - 1U);
    WriteReg16(0x3808, w);
    WriteReg16(0x380A, h);
}
```

- DCMI 长度 = `w * h * 2`（160×120 → 38,400 B）；
- USB 协议头输出 `RGB565 160 120 38400`；
- ROI 可由板上绿色质心跟踪或上位机命令 `ROI x y` 设置；
- 需要与方案 A 一样固定曝光。

## 结论与下一步

1. **方案 A 已上板**（160×120 + 固定曝光），但只有 ≈3.3 fps；固定曝光解决了暗光 1.2 s
   的问题，**没有解决时序上限**。
2. **方案 B 已定义并给出代码标注**，待实现/上板；它保持目标像素密度，是精度优先的选择。
3. **60 fps 的关键仍是 OV5640 时序/PLL**：需要按数据手册配置
   `PCLK ≈ 30–48 MHz`、`HTS/VTS` 使 `PCLK/(HTS×VTS)=60`，并做完整 sensor re-init；
   当前用任意 HTS/VTS 替换都会 FRAME TIMEOUT，说明 PLL 未匹配。
4. 下一步：
   - 获取/推导 OV5640 60 fps 寄存器表（PLL1 乘数/分频、HTS、VTS、PCLK period）；
   - 在初始化表中一次性写入并验证；
   - 之后再叠加方案 A/B（分辨率/ROI）与 1bpp/JPEG 负载，达到 60 fps。
