# AR_Giga_AutoDrive_GPS - Refactored Student Examples

## Overview

This repository contains a series of progressively complex Arduino examples for autonomous vehicle navigation on the Arduino Giga R1 WiFi. Each example is designed to be:

- **Simple**: Focused on one concept at a time
- **Well-commented**: Educational explanations throughout
- **Working**: Tested and verified code that actually works

## Hardware Configuration (Robot #2)

| Component | Connection |
|-----------|------------|
| Arduino Giga R1 WiFi | Main controller |
| GPS Module (NEMA) | Serial3 (RX/TX) |
| Yahboom 9-axis IMU | I2C (Wire) |
| RPLidar A1 | Serial2 (optional for future) |
| Steering Servo | Pin 7 |
| ESC (Throttle) | Pin 6 |

## Project Structure

```
AR_Giga_AutoDrive_GPS/
├── Example_1_GPS_Basic_Waypoint/       # Lesson 1: GPS fundamentals
│   └── Example_1_GPS_Basic_Waypoint.ino
├── Example_2_IMU_Heading_Maintenance/  # Lesson 2: IMU dead reckoning
│   └── Example_2_IMU_Heading_Maintenance.ino
├── Example_3_LIDAR_Obstacle_Avoidance/ # Lesson 3: Reactive obstacle avoidance
│   └── Example_3_LIDAR_Obstacle_Avoidance.ino
├── Example_4_GPS_IMU_Integrated_Navigation/ # Capstone: Full navigation
│   └── Example_4_GPS_IMU_Integrated_Navigation.ino
├── class_libs/                          # Shared library headers
│   ├── imu_i2c_driver.hpp
│   ├── bsp_iic.hpp
│   ├── RPLidar.h
│   └── TinyGPS++/
├── README.md                            # This file
└── CHANGELOG.md                         # Version history
```

## Lesson Sequence

### Example 1: GPS Basic Waypoint Navigation

**Learning Objectives:**
- Understanding GPS coordinates (latitude/longitude)
- Calculating bearing and distance between points
- Simple proportional steering control

**Key Concepts:**
- NMEA GPS data parsing
- Haversine formula for distance
- P-only controller

**Time Required:** 30-45 minutes

### Example 2: IMU Heading Maintenance (Dead Reckoning)

**Learning Objectives:**
- Understanding IMU sensors (accelerometer, gyro, magnetometer)
- Euler angles and heading calculation
- Sensor calibration procedures

**Key Concepts:**
- Dead reckoning navigation
- Magnetometer calibration (figure-8 method)
- Gyro drift and zeroing

**Time Required:** 45-60 minutes

### Example 3: LIDAR Obstacle Avoidance

**Learning Objectives:**
- LIDAR sensor operation and scanning
- Sector-based environmental awareness
- Reactive control strategies

**Key Concepts:**
- RPLidar data format
- Obstacle detection thresholds
- Reactive vs. deliberative control

**Time Required:** 30-45 minutes

### Example 4: GPS + IMU Integrated Navigation (Capstone)

**Learning Objectives:**
- Sensor fusion concepts
- PID controller tuning
- Multi-sensor navigation architecture

**Key Concepts:**
- Combining slow absolute (GPS) with fast relative (IMU) measurements
- PID control theory and tuning
- Waypoint cycling navigation

**Time Required:** 60-90 minutes

## Getting Started

### Step 1: Install Arduino IDE

Download and install the Arduino IDE from https://www.arduino.cc/en/software

### Step 2: Install Board Support

1. Open Arduino IDE
2. Go to File > Preferences
3. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/earlephilhower/arduino-pico/master/package_rp2040_index.json
   ```
4. Tools > Board > Boards Manager > Search "RP2040" > Install

### Step 3: Install Required Libraries

Go to Sketch > Include Library > Manage Libraries, then install:
- TinyGPS++ by Mikal Hart
- Servo by Arduino

### Step 4: Open an Example

1. File > Open
2. Navigate to the desired example folder
3. Select the `.ino` file (e.g., `Example_1_GPS_Basic_Waypoint.ino`)

### Step 5: Upload and Test

1. Select Board: "Arduino Giga R1 WiFi"
2. Select Port (should auto-detect)
3. Click Upload
4. Open Serial Monitor (115200 baud)

## Calibration Procedures

### Magnetometer Calibration (Required for IMU examples)

1. Set `PERFORM_MAG_CALIBRATION = true` in the code
2. Upload and open Serial Monitor
3. When prompted, pick up the vehicle
4. Rotate slowly in a FIGURE-8 pattern in ALL THREE AXES
5. Continue for ~30 seconds until calibration completes
6. Copy the printed offset values into `hardcodedMagOffsets[]`
7. Set `PERFORM_MAG_CALIBRATION = false` and re-upload

### North Alignment (Optional but Recommended)

1. Point vehicle physically North (use phone compass)
2. Note the Yaw reading in Serial Monitor
3. Set `MAG_CALIBRATION_OFFSET = -yaw_reading`
4. Yaw should now read ~0° when pointing North

## Troubleshooting

### GPS Never Gets Fix
- Ensure antenna has CLEAR view of sky
- Wait 1-2 minutes for initial acquisition
- Check Serial3 wiring (TX connected to GPS RX, and vice versa)
- Verify baud rate matches your GPS module (usually 9600)

### Vehicle Circles or Drives Away from Waypoint
- Check `steeringDirection` value (try flipping sign)
- Reduce `Kp` gain significantly (try 0.3)
- Verify MIN/MAX steering values match your servo direction
- Ensure enough satellites (5+) for reliable navigation

### IMU Not Found
- Check I2C wiring (SDA to SDA, SCL to SCL)
- Verify IMU has proper power (3.3V or 5V as required)
- Use `Wire.scan()` to verify I2C address (should be 0x68)

### Erratic Steering
- Reduce control gains (Kp, Ki, Kd)
- Add derivative term (Kd) to smooth response
- Check for electrical noise (add capacitors if needed)
- Verify servo power supply is adequate

## What Changed from Original Code?

| Issue | Original | Refactored |
|-------|----------|------------|
| Structure | .ino files scattered | Proper Arduino folders |
| Comments | Minimal | Extensive educational comments |
| PID Complexity | Overly complex integral handling | Simple P-first, optional I/D |
| Heading Wrap | Inconsistent handling | Single normalized error function |
| Calibration | Hidden in code | Clear step-by-step procedures |
| Working Code | Example 4 had bugs | All examples tested and working |

## License

This project is developed for educational purposes. Feel free to use and modify for your classroom.

## Author

Refactored for educational clarity - Original work by various contributors
