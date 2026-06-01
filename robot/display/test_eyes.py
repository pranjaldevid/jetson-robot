"""Two-screen GC9A01 smoke test.

Run with:  python -m robot.display.test_eyes

Fills display 1 (spidev0.0) red and display 2 (spidev0.1) blue, holds for
2 seconds, blanks both, and cleans up GPIO.
"""
import time
import Jetson.GPIO as GPIO

from robot.display.gc9a01 import GC9A01

RED = 0xF800     # RGB565
BLUE = 0x001F
BLACK = 0x0000


def main():
    # Pins match config/pins.yaml (display_left / display_right, BOARD numbering).
    disp1 = GC9A01(spi_bus=0, spi_dev=0, dc=29, rst=31)   # /dev/spidev0.0
    disp2 = GC9A01(spi_bus=0, spi_dev=1, dc=33, rst=32)   # /dev/spidev0.1
    try:
        disp1.fill(RED)
        disp2.fill(BLUE)
        time.sleep(2)
        disp1.fill(BLACK)
        disp2.fill(BLACK)
    finally:
        disp1.close()
        disp2.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()