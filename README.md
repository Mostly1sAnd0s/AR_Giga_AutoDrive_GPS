# AR_Giga_AutoDrive_GPS - Autonomous Vehicle Navigation

## 🚀 Overview

This project provides **progressive learning examples** for autonomous vehicle navigation using the Arduino Giga R1 WiFi. Originally developed for educational purposes, this refactored version focuses on:

- **Working code** - All examples tested and verified
- **Clear comments** - Educational explanations throughout  
- **Progressive difficulty** - Build skills step by step

## 📚 Student Examples (Start Here!)

| Example | Description | Difficulty | Time |
|---------|-------------|------------|------|
| [Example 1](Example_1_GPS_Basic_Waypoint/) | GPS waypoint navigation with proportional steering | ⭐ Easy | 30 min |
| [Example 2](Example_2_IMU_Heading_Maintenance/) | IMU-based heading maintenance (dead reckoning) | ⭐⭐ Medium | 45 min |
| [Example 3](Example_3_LIDAR_Obstacle_Avoidance/) | Reactive obstacle avoidance with RPLidar | ⭐⭐ Medium | 45 min |
| [Example 4](Example_4_GPS_IMU_Integrated_Navigation/) | Full GPS+IMU navigation with PID control | ⭐⭐⭐ Advanced | 90 min |

**👉 New students should start with Example 1 and work through sequentially.**

For complete documentation, calibration procedures, and troubleshooting, see [REFACTOR_README.md](REFACTOR_README.md).

## 🛠 Hardware Requirements (Robot #2)

| Component | Model | Connection |
|-----------|-------|------------|
| **Microcontroller** | Arduino Giga R1 WiFi | - |
| **GPS Module** | NEMA compatible | Serial3 |
| **IMU** | Yahboom 9-axis | I2C (Wire) |
| **LIDAR** | RPLidar A1 | Serial2 |
| **Steering Servo** | Standard servo | Pin 7 |
| **ESC** | Electronic Speed Controller | Pin 6 |

## 📂 Project Structure

```
AR_Giga_AutoDrive_GPS/
├── Example_1_GPS_Basic_Waypoint/          # Lesson 1: GPS navigation
├── Example_2_IMU_Heading_Maintenance/     # Lesson 2: IMU dead reckoning
├── Example_3_LIDAR_Obstacle_Avoidance/    # Lesson 3: LIDAR avoidance
├── Example_4_GPS_IMU_Integrated_Navigation/ # Capstone: Full navigation
├── class_libs/                            # Shared library headers
├── AR_Giga_AutoDrive_GPS.ino              # Original working code (reference)
├── README.md                              # This file
└── CHANGELOG.md                           # Version history
```

## ⚙️ Quick Start

### 1. Install Arduino IDE
Download from https://www.arduino.cc/en/software

### 2. Install Board Support
Add to Additional Board Manager URLs:
```
https://raw.githubusercontent.com/earlephilhower/arduino-pico/master/package_rp2040_index.json
```
Then install "RP2040" boards package.

### 3. Install Libraries
Via Arduino IDE Library Manager:
- TinyGPS++ by Mikal Hart
- Servo by Arduino

### 4. Upload Example 1
- Open `Example_1_GPS_Basic_Waypoint/Example_1_GPS_Basic_Waypoint.ino`
- Select Board: "Arduino Giga R1 WiFi"
- Upload and open Serial Monitor (115200 baud)

## 🔧 Calibration (Required for IMU Examples)

### Magnetometer Calibration
1. Set `PERFORM_MAG_CALIBRATION = true` in Example 4
2. Upload and open Serial Monitor
3. Pick up vehicle, rotate in **figure-8 pattern** in all 3 axes
4. Wait ~30 seconds for calibration to complete
5. Copy the printed offset values into your code
6. Set `PERFORM_MAG_CALIBRATION = false` and re-upload

### North Alignment
1. Point vehicle physically North (use phone compass)
2. Note Yaw reading in Serial Monitor
3. Set `MAG_CALIBRATION_OFFSET = -yaw_reading`
4. Yaw should now read 0° when pointing North

## 🎮 PID Tuning Guide (Example 4)

Start with **P-only control** (`Ki=0, Kd=0`) and adjust:

| Problem | Solution |
|---------|----------|
| Vehicle circles waypoint | Reduce `Kp` (try 0.3) |
| Oscillates left-right | Reduce `Kp`, add small `Kd` (0.05) |
| Consistently undershoots | Increase `Kp`, or add small `Ki` (0.001) |
| Overshoots and hunts | Add `Kd` term (0.1-0.3) |

**Tuning Order:** Kp first → then Kd → then Ki (if needed)

## 📝 Key Features by Example

### Example 1: GPS Basic Waypoint
- Simple P-only steering controller
- GPS coordinate parsing with TinyGPS++
- Bearing and distance calculation
- **No IMU required** - great for learning GPS basics

### Example 2: IMU Heading Maintenance
- Magnetometer-based heading control
- Gyro zeroing calibration
- Dead reckoning navigation
- **Works indoors** where GPS doesn't work

### Example 3: LIDAR Obstacle Avoidance
- RPLidar A1 sector scanning
- Reactive obstacle avoidance
- Front/Left/Right sector detection
- **Combines with any navigation system**

### Example 4: GPS + IMU Integrated (Capstone)
- Sensor fusion: GPS position + IMU heading
- Full PID steering controller
- Waypoint cycling navigation
- Complete calibration routines
- **Production-ready navigation system**

## 🐛 Common Issues

| Issue | Solution |
|-------|----------|
| GPS never gets fix | Ensure antenna has clear sky view, wait 1-2 min |
| Vehicle drives away from waypoint | Flip `steeringDirection` sign, reduce `Kp` |
| IMU not found | Check I2C wiring (SDA/SCL), verify power |
| Erratic steering | Reduce gains, add D term, check servo power |

See [REFACTOR_README.md](REFACTOR_README.md) for detailed troubleshooting.

## 📜 What Changed in This Refactor?

The original codebase had several issues that caused the integrated navigation to fail:

| Issue | Original | Fixed |
|-------|----------|-------|
| PID Controller | Overly complex integral windup | Simple P-first, optional I/D |
| Heading Wrap | Inconsistent 180° handling | Single normalized error function |
| Structure | .ino files scattered | Proper Arduino folders |
| Comments | Minimal | Extensive educational comments |
| Working Code | Example 4 had bugs | All examples tested and working |

## 📚 Additional Resources

- [REFACTOR_README.md](REFACTOR_README.md) - Complete documentation
- [CHANGELOG.md](CHANGELOG.md) - Version history
- `AR_Giga_AutoDrive_GPS.ino` - Original working code (reference only)

## 📝 License

Developed for educational purposes. Feel free to use and modify for your classroom.

---

**Happy coding! 🚗🤖**
