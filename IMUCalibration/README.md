# IMUCalibration - Magnetometer Offset Finder

## Purpose

This sketch helps you find **accurate magnetometer calibration offsets** using the figure-8 method. These offsets are essential for stable heading calculations.

## Quick Start

1. **Upload** this code to Arduino Giga R1 WiFi
2. **Open Serial Monitor** at 115200 baud
3. **Press 'c'** to start calibration
4. **Rotate device** in figure-8 pattern for 30+ seconds
5. **Press 's'** to stop and get your offsets
6. **Copy the offset values** into your main code

## Files Included

| File | Description |
|------|-------------|
| `IMUCalibration.ino` | Main calibration sketch |
| `IMU_I2C_Driver.h` | IMU driver header |
| `IMU_I2C_Driver.cpp` | IMU driver implementation |
| `README.md` | This file |

## How It Works

The magnetometer measures Earth's magnetic field, but nearby metal objects and electronics create interference. Calibration finds the **center point** of this interference:

```
offset[i] = (max[i] + min[i]) / 2
```

Where `i` is X, Y, or Z axis.

## Serial Commands

| Key | Action |
|-----|--------|
| **c** | Start calibration (clears previous data) |
| **s** | Stop calibration and display offsets |
| **r** | Show current min/max values |
| **h** | Show instructions again |

## Calibration Procedure (Step-by-Step)

### 1. Prepare Environment
- Move away from computers, phones, metal objects
- Find an open space with minimal magnetic interference

### 2. Start Calibration
```
Press 'c' in Serial Monitor
```

### 3. Rotate Device (30-60 seconds)
Rotate your device in a **figure-8 pattern**, covering all three axes:

- **Z-axis**: Rotate like turning a steering wheel
- **X-axis**: Nod up and down  
- **Y-axis**: Tilt side to side

Watch the live display - you want the min/max range to grow as you rotate.

### 4. Stop and Get Results
```
Press 's' in Serial Monitor
```

The sketch will display:
- Min/Max values for each axis
- **Calculated offsets** (copy these!)
- Quality rating

### 5. Apply Offsets
Copy the printed offset values into your main code:

```cpp
// In IMUTest.ino or your production code
float magOffset[3] = {X_offset, Y_offset, Z_offset};
```

## Example Output

```
========================================================================
  CALIBRATION COMPLETE!
========================================================================

Axis   | Min         | Max         | Range     | Offset
-------|-------------|-------------|-----------|-------------
X     | -25.342 uT | 67.891 uT | 93.233 uT | 21.2745 uT
Y     | -15.123 uT | 32.456 uT | 47.579 uT | 8.6665 uT
Z     | 10.234 uT | 45.678 uT | 35.444 uT | 27.9560 uT

========================================================================
  COPY THESE VALUES INTO YOUR CODE:
========================================================================

// Magnetometer calibration offsets
float magOffset[3] = {21.2745, 8.6665, 27.9560};
```

## Quality Check

The sketch rates your calibration quality:

| Quality | Criteria | Action |
|---------|----------|--------|
| **EXCELLENT** | Range >30uT, Samples >500 | Use offsets confidently |
| **GOOD** | Range >15uT, Samples >200 | Acceptable for most uses |
| **FAIR** | Samples >100 | Consider recalibrating |
| **POOR** | Samples <100 | Recalibrate recommended |

## Troubleshooting

| Issue | Solution |
|-------|----------|
| IMU not found | Check I2C wiring (SDA/SCL), verify power |
| Range is very small (<10uT) | Not rotating enough, recalibrate with more movement |
| Offsets seem wrong | Recalibrate away from magnetic interference |
| Values jump around | Hold device firmly, avoid shaking |

## Default Values (CHANGELOG v2.1.0)

Pre-loaded calibration values for quick deployment:
```cpp
float magOffset[3] = {42.3353, 8.7161, 23.6091};
```

**Replace these with your calibrated values!**

## Compilation & Upload

```bash
# Compile
arduino-cli compile --fqbn arduino:mbed_giga:giga IMUCalibration/

# Upload
arduino-cli upload --fqbn arduino:mbed_giga:giga -p /dev/cu.usbmodem101 IMUCalibration/
```

## Related Documentation

- `IMUTest/` - Test your calibrated offsets
- `CHANGELOG.md` v2.1.0 - IMU calibration fixes
- `Class Resources/IIC/` - Original IMU driver library

---

**After calibration, test your offsets with the IMUTest example to verify stable heading!**
