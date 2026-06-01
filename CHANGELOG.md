# Changelog

All notable changes to the `AR_Giga_AutoDrive_GPS` project will be documented in this file.

## [2.2.0] - 2026-05-31 - Smoothing Algorithm Fix & Driver Timing Updates

### Fixed
- **Angle-aware smoothing**: Fixed smoothing algorithm to handle ±180° wrap-around correctly
  - Previous bug: Smoothing saw -179° → +179° as a 358° jump instead of 2°
  - Solution: Normalize difference before applying smooth filter
- **I2C read timing**: Added delays between sensor reads in `IMU_I2C_ReadAll()`
  - Problem: Reading sensors too fast caused stale magnetometer data (X stuck at -18.751)
  - Solution: 500µs delay between each sensor read allows IMU to update registers
- **Mag X sensitivity**: Magnetometer X-axis now updates correctly when rotating

### Clarified
- **IMU Euler output is CORRECT**: No firmware bug exists in the native Euler yaw output
  - Previous documentation incorrectly claimed "Euler jumps when W crosses zero"
  - Reality: The IMU's Euler yaw is smooth and accurate
  - The actual issue was in how we applied smoothing, not the sensor data itself
- **Quaternion math works**: Standard quaternion-to-yaw formula (`atan2`) is correct
  - Continuous yaw tracking via delta accumulation works perfectly
  - Can use either native Euler or quaternion-derived yaw (both are valid)

### Created Diagnostic Tools
- `IMUDiagnostic/` - Raw sensor data output for IMU analysis
- `IMUYawDebug/` - Multiple yaw formula comparison (MY_F1, MY_F2, MY_F3)

---

## [2.1.0] - 2026-05-31 - IMU Magnetometer Calibration Fixes

### Fixed
- **Yaw overflow bug**: Yaw values showing >360° due to missing normalization before averaging
- **Magnetometer offset persistence**: Added `IMU_I2C_SetMagOffsets()` to write calibration data to IMU RAM
- **North alignment calculation**: Now properly normalizes yaw to [-180°, +180°] before averaging samples

### Added
- `Example_2_MagCalibration/` - Complete magnetometer calibration with figure-8 method and north alignment
- Automatic offset writing to IMU RAM after successful calibration
- Pre-loaded calibration values for quick deployment (X: 42.3353, Y: 8.7161, Z: 23.6091)

### Changed
- Improved calibration output with clear step-by-step instructions
- Better telemetry display with normalized yaw values

---

## [2.0.0] - 2026-05-30 - Major Refactor for Education

### Changed
- **Complete codebase refactor** for student education
- Reorganized into sequential learning examples (1-4)
- Added extensive educational comments throughout
- Simplified PID controller to P-first approach with optional I/D

### Added
- `Example_1_GPS_Basic_Waypoint/` - Simple GPS navigation, no IMU required
- `Example_2_IMU_Heading_Maintenance/` - Dead reckoning with IMU only
- `Example_3_LIDAR_Obstacle_Avoidance/` - Reactive obstacle avoidance
- `Example_4_GPS_IMU_Integrated_Navigation/` - Full sensor fusion navigation
- `class_libs/` - Shared library headers for student use
- `REFACTOR_README.md` - Complete documentation for students

### Fixed
- **Circling bug** in integrated navigation (removed problematic integral windup)
- **Heading wrap handling** at 180° boundary (single normalized error function)
- **Steering direction confusion** (clear steeringDirection parameter)
- **PID instability** (P-first approach, I and D clearly marked as optional)

### Removed
- `Example_GPS_IMU_Integrated/` - Replaced by Example 4
- `Example_GPS_Waypoint/` - Replaced by Example 1
- `Example_IMU_DeadReckoning/` - Replaced by Example 2
- `Example_LIDAR_Avoidance.ino` - Replaced by Example 3
- Scattered `.ino` helper files (compass.ino, i2c.ino, lidar.ino)

### Documentation
- Added comprehensive calibration procedures
- Added PID tuning guide with troubleshooting table
- Added lesson sequence with time estimates
- Added hardware configuration reference

---

## [1.2.0] - 2026-05-27
### Added
- **Integrated Navigation**: Created `Example_GPS_IMU_Integrated` combining GPS and IMU.
- **PID Control**: Implemented full PID steering controller with adjustable Kp, Ki, and Kd.
- **Advanced Calibration**: 
  - Added Static Calibration (15s zeroing).
  - Added Magnetometer Figure-8 calibration.
  - Added `northOffset` for absolute map alignment.
- **Safety Features**: 
  - GPS Satellite threshold (min 4-6 satellites required).
  - Anti-windup and derivative normalization for PID stability.

### Fixed
- **Double-Conversion Bug**: Removed redundant degree-to-radian conversions causing circling.
- **Derivative Spikes**: Fixed steering "slamming" at the 180-degree wrap point.
- **Steering Polarity**: Corrected `steeringDirection` logic for inverted servos.
- **I2C Synchronicity**: Unified IMU driver files across all project folders.

## [1.1.0] - 2026-05-27
### Added
- **Sensor Examples**: Created standalone sketches for RPLidar, Yahboom IMU, and TinyGPS.
- **I2C Driver Enhancement**: Added support for reading and writing Mag offsets to RAM (workaround for Giga's lack of EEPROM).

## [1.0.0] - Initial Review
- Baseline review of original `AR_Giga_AutoDrive_GPS.ino` and I2C recovery routines.
