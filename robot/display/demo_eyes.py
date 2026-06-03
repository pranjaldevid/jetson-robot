"""GC9A01 robot-eye demonstration.

Run with:  python -m robot.display.demo_eyes

Sequence:
  1. Both eyes open, centre gaze (2 s)
  2. Left / right / up / down (1.5 s each)
  3. Two blinks
  4. Idle random gaze for 30 s
"""
import time
from robot.display.eyes import Eyes


def main():
    with Eyes() as eyes:
        print("Centre gaze...")
        eyes.look(0, 0)
        time.sleep(2)

        for label, x, y in [
            ("left",  -1.0,  0.0),
            ("right",  1.0,  0.0),
            ("up",     0.0, -1.0),
            ("down",   0.0,  1.0),
        ]:
            print(f"Looking {label}...")
            eyes.look(x, y)
            time.sleep(1.5)

        eyes.look(0, 0)
        time.sleep(0.2)

        print("Blink 1...")
        eyes.blink()
        time.sleep(0.3)
        print("Blink 2...")
        eyes.blink()

        print("Idle for 30 seconds...")
        eyes.idle(duration=30)
        print("Demo complete.")


if __name__ == "__main__":
    main()
