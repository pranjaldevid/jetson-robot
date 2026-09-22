# Jetson Robot

Four-wheeled autonomous companion robot. Jetson Orin Nano runs perception, navigation, and interaction; a Raspberry Pi Pico 2 W handles low-level motor control.

## Hardware

- Jetson Orin Nano — vision, IMU, displays, audio
- Pi Camera Module 2 (CSI)
- MPU6050 IMU (I2C bus 7)
- 2x GC9A01 round displays (SPI0)
- INMP441 mic + MAX98357 amp + 3W speaker (I2S0)
- Pico 2 W + Kitronik 5329 — 4 motors

See `config/pins.yaml` for the wiring map.

## Software architecture

- **ros2_ws/src** — ROS 2 workspace. Camera and IMU nodes feed a Visual SLAM (VSLAM) pipeline for localisation and mapping; a voice command node handles speech input for navigation and person-following behaviours.
- **pico** — firmware for the Pico 2 W, running four-wheel-drive motor control over the Kitronik 5329 driver board.
- **config** — pin mappings and hardware configuration (see `pins.yaml`).
- **systemd** — service definitions for running the stack on boot.
- **docs** — project and setup notes.
- **tests** — test scripts for individual subsystems.

## Status

Working:
- ROS 2 simulation environment integrating camera and IMU for Visual SLAM
- Voice command processing for navigation and person-following
- Motor control firmware for the four-wheel-drive platform
- Chassis and electronics built and integrated (multi-iteration CAD, FDM printed)

In progress:
- On-device LLM inference
- Behaviour state machine for autonomous decision-making

