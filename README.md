# Micro-Mouse

An autonomous maze-solving robot built on ESP32, designed to navigate and solve mazes using floodfill algorithms and sensor fusion.

## Features

- **Maze Navigation**: Floodfill algorithm for optimal pathfinding
- **Motion Control**: PID-based motor control with encoder feedback
- **Sensor Suite**: 
  - IMU (MPU6050) for orientation tracking
  - Time-of-Flight (VL53L0X) for distance sensing
  - IR sensors for wall detection
  - Encoders for odometry
- **Power Management**: Real-time voltage/current monitoring
- **Display**: OLED display for status and debugging
- **Button Control**: Start/stop and mode selection

## Hardware

- **Microcontroller**: ESP32
- **Motors**: DC motors with encoders
- **Sensors**: ToF, IMU, IR, encoders
- **Display**: SSD1306 OLED
- **Power Monitoring**: INA3221

## Project Structure

```
src/          - Implementation files
include/      - Header files
platformio.ini - PlatformIO configuration
```

## Building

```bash
platformio run -e esp32dev
```

## Uploading

```bash
platformio run -e esp32dev --target upload
```

## Configuration

Edit [include/config.h](include/config.h) to adjust:
- Motor control parameters
- Sensor calibration values
- Maze dimensions
- Movement speeds

## Calibration

1. Move robot forward exactly 18 cm using `move_forward_cm(18)`
2. Record encoder tick count
3. Calculate: `TICKS_PER_CM = (left_ticks + right_ticks) / 2.0 / 18.0`
4. Update `TICKS_PER_CM` in config.h
