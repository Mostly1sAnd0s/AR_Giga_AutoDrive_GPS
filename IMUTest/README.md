# IMUTest - Stable Heading Demonstration

## Purpose

This example demonstrates how to obtain a **stable and accurate heading** from the 9-axis IMU (accelerometer + gyroscope + magnetometer) on the Arduino Giga R1 WiFi.

## Key Issues Addressed

1. **Yaw overflow/normalization** - Keeps heading in [-180°, +180°] range
2. **Magnetometer calibration** - Proper offset application for accurate magnetic heading
3. **Sensor fusion** - Uses quaternion data for best stability (fused by IMU firmware)

## Files Included

| File | Description |
|------|-------------|
| `IMUTest.ino` | Main example sketch |
| `IMU_I2C_Driver.h` | Simplified IMU driver header |
| `IMU_I2C_Driver.cpp` | Simplified IMU driver implementation |
| `README.md` | This file |

## How to Use

1. **Upload** this code to your Arduino Giga R1 WiFi
2. **Open Serial Monitor** at 115200 baud
3. **Watch the heading output** - it should be stable when device is stationary
4. **If heading drifts**, perform magnetometer calibration (see below)

## Output Overview

The sketch displays:
- Raw sensor data (accelerometer, gyroscope, magnetometer)
- Calibrated magnetometer data with offsets applied
- Quaternion fusion data from IMU
- **Two heading calculations**:
  - `IMU Fused Yaw` - From internal quaternion fusion (MOST STABLE)
  - `Mag Calculated Heading` - From raw magnetometer + offsets (INDEPENDENT CHECK)
- Quality checks for stability assessment

## For Future Developers

### The Critical Fix: Angle Normalization

```cpp
float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
```

**This is CRITICAL for stable heading calculations.** Without normalization, yaw values can overflow (>360° or <-360°), causing erratic behavior at the 180° boundary.

### Magnetometer Calibration

The default offsets in the code are from CHANGELOG v2.1.0:
```cpp
float magOffset[3] = {42.3353, 8.7161, 23.6091};
```

**Replace these with your calibrated values!** See calibration procedure below.

### North Alignment

To make yaw read 0° when pointing North:
```cpp
float northOffset = -yaw_reading_when_pointing_north;
```

## Calibration Procedure

### Magnetometer Calibration (Figure-8 Method)

1. Create a calibration sketch that:
   - Reads raw magnetometer values while rotating in figure-8 pattern
   - Tracks min/max values for each axis
   - Calculates offsets: `offset[i] = (max[i] + min[i]) / 2`

2. Apply offsets to raw magnetometer readings:
   ```cpp
   mag_calibrated[i] = mag_raw[i] - offset[i]
   ```

3. Update the `magOffset[3]` array in your code

### North Alignment

1. Point device physically North (use phone compass as reference)
2. Note the yaw reading from IMU fusion
3. Set `northOffset = -yaw_reading`
4. Now yaw reads 0° when pointing North

## Troubleshooting

| Issue | Solution |
|-------|----------|
| IMU not found | Check I2C wiring (SDA/SCL), verify power, confirm address 0x23 |
| Heading drifts wildly | Perform magnetometer calibration |
| Heading offset from expected | Adjust `northOffset` value |
| Unstable when stationary | Check for magnetic interference, recalibrate |

## Related Documentation

- `CHANGELOG.md` v2.1.0 - IMU Magnetometer Calibration Fixes
- `Example_2_MagCalibration/` - Complete calibration routine
- `Class Resources/IIC/` - Original IMU driver library

## Compilation

```bash
arduino-cli compile --fqbn arduino:mbed_giga:giga IMUTest/
```

## Upload

```bash
arduino-cli upload --fqbn arduino:mbed_giga:giga -p /dev/cu.usbmodem101 IMUTest/
```

(Note: Replace `/dev/cu.usbmodem101` with your actual port from `arduino-cli board list`)

---

**This example is designed to guide future development by demonstrating proper heading stabilization techniques.**
