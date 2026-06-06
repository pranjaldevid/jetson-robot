"""GC9A01 240x240 round SPI display driver — Jetson Orin Nano.

Hardware is configured via the JetsonHacks device-tree overlay
(jetson-orin-spi-overlay-guide, 'st7789/jetson-default' preset), which:
  - enables the SPI controller as /dev/spidev1.0  (CS on BOARD pin 24)
  - configures DC=BOARD 29 and RST=BOARD 31 as proper GPIOs

This replaces the old jetson-io + devmem approach. CS is the controller's
hardware chip-select and is never driven here. DC/RST use Jetson.GPIO (BOARD).
"""
import time
import numpy as np
import spidev
import Jetson.GPIO as GPIO

WIDTH = 240
HEIGHT = 240

# (command, data bytes) — standard GC9A01 init sequence.
_INIT = [
    (0xEF, []),
    (0xEB, [0x14]),
    (0xFE, []),
    (0xEF, []),
    (0xEB, [0x14]),
    (0x84, [0x40]),
    (0x85, [0xFF]),
    (0x86, [0xFF]),
    (0x87, [0xFF]),
    (0x88, [0x0A]),
    (0x89, [0x21]),
    (0x8A, [0x00]),
    (0x8B, [0x80]),
    (0x8C, [0x01]),
    (0x8D, [0x01]),
    (0x8E, [0xFF]),
    (0x8F, [0xFF]),
    (0xB6, [0x00, 0x20]),
    (0x36, [0x08]),                  # MADCTL — RGB, portrait (try 0x08 if R/B swapped)
    (0x3A, [0x05]),                  # COLMOD — 16 bits/pixel (RGB565)
    (0x90, [0x08, 0x08, 0x08, 0x08]),
    (0xBD, [0x06]),
    (0xBC, [0x00]),
    (0xFF, [0x60, 0x01, 0x04]),
    (0xC3, [0x13]),
    (0xC4, [0x13]),
    (0xC9, [0x22]),
    (0xBE, [0x11]),
    (0xE1, [0x10, 0x0E]),
    (0xDF, [0x21, 0x0C, 0x02]),
    (0xF0, [0x45, 0x09, 0x08, 0x08, 0x26, 0x2A]),
    (0xF1, [0x43, 0x70, 0x72, 0x36, 0x37, 0x6F]),
    (0xF2, [0x45, 0x09, 0x08, 0x08, 0x26, 0x2A]),
    (0xF3, [0x43, 0x70, 0x72, 0x36, 0x37, 0x6F]),
    (0xED, [0x1B, 0x0B]),
    (0xAE, [0x77]),
    (0xCD, [0x63]),
    (0x70, [0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03]),
    (0xE8, [0x34]),                  # 4 dot inversion
    (0x62, [0x18, 0x0D, 0x71, 0xED, 0x70, 0x70,
            0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70]),
    (0x63, [0x18, 0x11, 0x71, 0xF1, 0x70, 0x70,
            0x18, 0x13, 0x71, 0xF3, 0x70, 0x70]),
    (0x64, [0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07]),
    (0x66, [0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00]),
    (0x67, [0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98]),
    (0x74, [0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00]),
    (0x98, [0x3E, 0x07]),
    (0x35, []),                      # tearing effect line on
    (0x21, []),                      # display inversion ON — GC9A01 panels need this
    (0x11, []),                      # sleep out
]


class GC9A01:
    """Driver for a single GC9A01 round display on hardware-CS SPI."""

    def __init__(self, spi_bus=1, spi_dev=0, dc=29, rst=31,
                 speed_hz=32000000):
        # Defaults match the JetsonHacks 'jetson-default' overlay:
        #   spi_bus=1, spi_dev=0  -> /dev/spidev1.0  (CS = BOARD 24)
        #   dc=29, rst=31         -> DC/RST GPIOs
        self.dc = dc
        self.rst = rst

        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_dev)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.dc, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.rst, GPIO.OUT, initial=GPIO.HIGH)

        self._reset()
        self._init_display()

    def _reset(self):
        GPIO.output(self.rst, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(self.rst, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.rst, GPIO.HIGH)
        time.sleep(0.12)

    def _write_cmd(self, cmd):
        GPIO.output(self.dc, GPIO.LOW)
        self.spi.writebytes([cmd])

    def _write_data(self, data):
        GPIO.output(self.dc, GPIO.HIGH)
        if data:
            self.spi.writebytes(list(data))

    def _init_display(self):
        for cmd, data in _INIT:
            self._write_cmd(cmd)
            if data:
                self._write_data(data)
        time.sleep(0.12)             # settle after sleep-out
        self._write_cmd(0x29)        # display on
        time.sleep(0.02)

    def set_window(self, x0, y0, x1, y1):
        """Set the active drawing rectangle (inclusive coordinates)."""
        self._write_cmd(0x2A)        # column address set
        self._write_data([x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF])
        self._write_cmd(0x2B)        # row address set
        self._write_data([y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF])

    def _write_pixels(self, buf):
        """Send RAMWR, then stream the whole frame in one transfer.

        writebytes2 is the proven path (this is what the JetsonHacks ST7789
        driver uses): it streams an arbitrarily large buffer natively with
        correct chip-select handling, no manual chunking required.
        """
        self._write_cmd(0x2C)        # RAMWR
        GPIO.output(self.dc, GPIO.HIGH)
        try:
            self.spi.writebytes2(buf)
        except AttributeError:
            # Fallback for ancient spidev without writebytes2
            chunk = 4096
            for i in range(0, len(buf), chunk):
                self.spi.writebytes(buf[i:i + chunk])

    def blit(self, buf):
        """Write a full 240x240 RGB565 big-endian frame (bytes/bytearray)."""
        self.set_window(0, 0, WIDTH - 1, HEIGHT - 1)
        self._write_pixels(buf)

    def fill(self, color565):
        """Fill the whole screen with a 16-bit RGB565 color."""
        hi = (color565 >> 8) & 0xFF
        lo = color565 & 0xFF
        frame = np.empty(WIDTH * HEIGHT * 2, dtype=np.uint8)
        frame[0::2] = hi
        frame[1::2] = lo
        self.set_window(0, 0, WIDTH - 1, HEIGHT - 1)
        self._write_pixels(frame.tobytes())

    def close(self):
        self.spi.close()


def frame_from_rgb(rgb):
    """Pack an (H, W, 3) uint8 RGB array into 240x240 RGB565 big-endian bytes."""
    rgb = np.asarray(rgb, dtype=np.uint16)
    r = (rgb[..., 0] >> 3) & 0x1F
    g = (rgb[..., 1] >> 2) & 0x3F
    b = (rgb[..., 2] >> 3) & 0x1F
    rgb565 = (r << 11) | (g << 5) | b
    return rgb565.astype(">u2").tobytes()   # big-endian
