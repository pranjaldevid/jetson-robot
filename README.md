# Robot

Four-wheeled robot. Jetson Orin Nano (brain) + Pico 2 W (motors).

## Hardware
- Jetson Orin Nano — vision, IMU, displays, audio
- Pi Camera Module 2 (CSI)
- MPU6050 IMU (I2C bus 7)
- 2x GC9A01 round displays (SPI0)
- INMP441 mic + MAX98357 amp + 3W speaker (I2S0)
- Pico 2 W + Kitronik 5329 — 4 motors

See `config/pins.yaml` for the wiring map.
