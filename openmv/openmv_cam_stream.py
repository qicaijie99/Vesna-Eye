# OpenMV Cam H7 Plus -- green glow target camera stream
#
# This script only captures images and streams them to the upper-computer C++
# vision module (green_glow_tracker). All target detection / PnP / Kalman
# filtering runs on the host; the OpenMV stays a "smart camera" that delivers
# JPEG frames with a byte stream protocol.
#
# Frame protocol (see docs/protocol.md section 2):
#   A5 5A | ver(1) | flags(1) | seq(1) | width(u16 LE) | height(u16 LE) |
#   jpeg_len(u32 LE) | jpeg[jpeg_len] | crc16(u16 LE, CCITT-FALSE over
#   ver..payload)
#
# Wiring (UART mode), OpenMV H7 Plus UART3:
#   OpenMV P4 (TX) -> USB-TTL RX
#   OpenMV P5 (RX) -> USB-TTL TX
#   GND            -> GND
#   The host opens the USB-TTL COM port (FrameSource type OpenMvSerial).
# USB mode: no wiring, use the OpenMV USB CDC port (recommended: faster).

import sensor
import time
import struct

try:
    from pyb import UART
except ImportError:  # non-OpenMV board / host sanity check
    UART = None

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
USE_USB = True          # True: USB CDC (fast, recommended); False: UART
UART_ID = 3
UART_BAUD = 3000000     # 3 Mbaud; use 6000000 if your board/wiring supports it
JPEG_QUALITY = 60       # 50-70 typical for QVGA over UART
FRAME_DELAY_MS = 0      # extra delay between frames (0 = maximum rate)
AUTO_EXPOSURE_US = 8000 # fixed exposure to keep the green glow reproducible
AUTO_GAIN_DB = 8        # fixed gain
LED_ON = False          # green/red LEDs of the board


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) -- matches the host."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)   # 320x240, same FOV as full resolution
    # Fixed exposure/gain/white balance: stable colour thresholds and PnP.
    sensor.set_auto_gain(False, gain_db=AUTO_GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=AUTO_EXPOSURE_US)
    sensor.set_auto_rotation(False)
    sensor.skip_frames(time=1500)
    try:
        sensor.set_framebuffers(2)
    except Exception:
        pass
    if LED_ON:
        try:
            from pyb import LED
            LED(2).on()
        except Exception:
            pass


def make_output():
    if USE_USB:
        from pyb import USB_VCP
        return USB_VCP()
    if UART is None:
        raise RuntimeError("UART not available")
    return UART(UART_ID, UART_BAUD, timeout=1000)


def main():
    init_sensor()
    out = make_output()
    seq = 0
    clock = time.clock()
    print("openmv_cam_stream: %dx%d, q=%d, %s" %
          (sensor.width(), sensor.height(), JPEG_QUALITY,
           "USB" if USE_USB else "UART%d@%d" % (UART_ID, UART_BAUD)))
    while True:
        clock.tick()
        img = sensor.snapshot()
        jpeg = img.compress(quality=JPEG_QUALITY)
        width = img.width()
        height = img.height()
        header_body = struct.pack("<BBBHHI", 1, 0, seq & 0xFF, width, height,
                                  len(jpeg))
        crc = crc16_ccitt(header_body + jpeg)
        out.write(b"\xA5\x5A")
        out.write(header_body)
        out.write(jpeg)
        out.write(struct.pack("<H", crc))
        seq = (seq + 1) & 0xFF
        if FRAME_DELAY_MS:
            time.sleep_ms(FRAME_DELAY_MS)
        print("fps=%.1f bytes=%d" % (clock.fps(), len(jpeg)))


main()
