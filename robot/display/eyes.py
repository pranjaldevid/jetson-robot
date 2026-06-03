"""Robotic eye system for two GC9A01 240×240 round displays.

Usage:
    with Eyes() as eyes:
        eyes.look(0, 0)
        eyes.blink()
        eyes.idle()
"""
import random
import time

import numpy as np
from PIL import Image, ImageDraw
import Jetson.GPIO as GPIO

from robot.display.gc9a01 import GC9A01, frame_from_rgb, WIDTH, HEIGHT

_CENTER = WIDTH // 2          # 120

_SCLERA_D      = 220
_IRIS_D        = 110
_PUPIL_D       = 55
_HIGHLIGHT_D   = 18
_HIGHLIGHT_DX  = 22           # offset from pupil centre: right
_HIGHLIGHT_DY  = -18          # offset from pupil centre: up
_MAX_GAZE_PX   = 30

_IRIS_COLOR   = (0, 150, 255)  # #0096FF electric blue
_LIMBAL_COLOR = (0, 0, 80)     # dark blue
_WHITE        = (255, 255, 255)
_BLACK        = (0, 0, 0)

_BLINK_STEPS   = 5
_BLINK_FRAME_S = 0.033         # ~30 fps per blink frame


def _bbox(cx, cy, d):
    r = d // 2
    return (cx - r, cy - r, cx + r, cy + r)


def _draw_eye(gaze_x: float, gaze_y: float) -> Image.Image:
    img = Image.new("RGB", (WIDTH, HEIGHT), _BLACK)
    draw = ImageDraw.Draw(img)

    # Sclera
    draw.ellipse(_bbox(_CENTER, _CENTER, _SCLERA_D), fill=_WHITE)

    # Gaze offset (clamp to [-1, 1] before calling)
    ox = int(gaze_x * _MAX_GAZE_PX)
    oy = int(gaze_y * _MAX_GAZE_PX)
    ix, iy = _CENTER + ox, _CENTER + oy

    # Iris with limbal ring as outline
    draw.ellipse(_bbox(ix, iy, _IRIS_D),
                 fill=_IRIS_COLOR, outline=_LIMBAL_COLOR, width=3)

    # Pupil
    draw.ellipse(_bbox(ix, iy, _PUPIL_D), fill=_BLACK)

    # Specular highlight
    draw.ellipse(_bbox(ix + _HIGHLIGHT_DX, iy + _HIGHLIGHT_DY, _HIGHLIGHT_D),
                 fill=_WHITE)

    return img


def _apply_lids(base: Image.Image, step: int) -> Image.Image:
    """Overlay closing eyelids at *step* (0 = open, _BLINK_STEPS = shut)."""
    img = base.copy()
    draw = ImageDraw.Draw(img)
    lid_h = int(step * (_CENTER + 10) / _BLINK_STEPS)  # grows to 130 px
    # Top lid: ellipse centred above the top edge, visible portion grows downward
    draw.ellipse((0, -lid_h, WIDTH, lid_h), fill=_BLACK)
    # Bottom lid: ellipse centred below the bottom edge, visible portion grows upward
    draw.ellipse((0, HEIGHT - lid_h, WIDTH, HEIGHT + lid_h), fill=_BLACK)
    return img


def _to_buf(img: Image.Image) -> bytes:
    return frame_from_rgb(np.array(img))


class Eyes:
    """Manages the two GC9A01 round-display robot eyes."""

    def __init__(self):
        self._left  = GC9A01(spi_bus=0, spi_dev=0, dc=29, rst=31)
        self._right = GC9A01(spi_bus=0, spi_dev=1, dc=33, rst=32)
        self._gx = 0.0
        self._gy = 0.0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _blit(self, buf: bytes):
        self._left.blit(buf)
        self._right.blit(buf)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def look(self, x: float, y: float):
        """Render both eyes looking at normalised gaze (x, y) in [-1, 1].

        +x = right, +y = down.  Max pupil/iris offset is 30 px.
        """
        self._gx = max(-1.0, min(1.0, x))
        self._gy = max(-1.0, min(1.0, y))
        self._blit(_to_buf(_draw_eye(self._gx, self._gy)))

    def blink(self):
        """Play a blink animation: close over 5 frames, open over 5 frames."""
        base = _draw_eye(self._gx, self._gy)
        close_steps = list(range(1, _BLINK_STEPS + 1))       # [1..5]
        open_steps  = list(range(_BLINK_STEPS - 1, -1, -1))  # [4..0]
        for step in close_steps + open_steps:
            self._blit(_to_buf(_apply_lids(base, step)))
            time.sleep(_BLINK_FRAME_S)

    def idle(self, duration: float = None):
        """Slow random gaze drift with a blink every 3–8 seconds.

        Args:
            duration: seconds to run; ``None`` runs indefinitely.
        """
        cx, cy = self._gx, self._gy
        tx, ty = 0.0, 0.0
        now = time.monotonic()
        next_blink  = now + random.uniform(3, 8)
        next_target = now
        deadline    = (now + duration) if duration is not None else None

        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            # Pick a new gaze target once we arrive at the current one
            if now >= next_target:
                tx = random.uniform(-0.55, 0.55)
                ty = random.uniform(-0.55, 0.55)
                next_target = now + random.uniform(1.5, 4.0)

            # Exponential drift toward target (~20 px/s at full offset)
            cx += (tx - cx) * 0.07
            cy += (ty - cy) * 0.07
            self.look(cx, cy)

            if now >= next_blink:
                self.blink()
                next_blink = time.monotonic() + random.uniform(3, 8)

            time.sleep(0.05)  # 20 fps

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self):
        self._left.close()
        self._right.close()
        GPIO.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
