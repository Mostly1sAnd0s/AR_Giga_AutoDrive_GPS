/**
 * IMU I2C Driver Header
 * Simplified interface for Yahboom 9-axis IMU (I2C address 0x23)
 * Based on Class Resources/IIC/imu_i2c_driver.hpp
 */

#ifndef IMU_I2C_DRIVER_H
#define IMU_I2C_DRIVER_H

#include "Wire.h"
#include <Arduino.h>

// IMU I2C Command Codes
#define IMU_FUNC_VERSION        0x01
#define IMU_FUNC_RAW_ACCEL      0x04
#define IMU_FUNC_RAW_GYRO       0x0A
#define IMU_FUNC_RAW_MAG        0x10
#define IMU_FUNC_QUAT           0x16
#define IMU_FUNC_EULER          0x26
#define IMU_FUNC_BARO           0x32
#define IMU_FUNC_CALIB_IMU      0x70
#define IMU_FUNC_CALIB_MAG      0x71
#define IMU_FUNC_REQUEST_DATA   0x80

// IMU I2C Address
#define IMU_I2C_ADDRESS 0x23

// Data structure for all IMU measurements
typedef struct {
    float accel[3];   // Acceleration in g
    float gyro[3];    // Angular velocity in rad/s
    float mag[3];     // Magnetic field in uT
    float quat[4];    // Quaternion (w, x, y, z)
    float euler[3];   // Euler angles in degrees (roll, pitch, yaw)
    float baro[4];    // Barometer data
} imu_measurement_t;

// Basic I2C communication functions
int IMU_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len);
int IMU_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *buf, uint16_t len);

// Command functions
int  IMU_I2C_SendCommand(uint8_t function, uint16_t value);

// Sensor read functions
int  IMU_I2C_ReadAccelerometer(float out[3]);
int  IMU_I2C_ReadGyroscope(float out[3]);
int  IMU_I2C_ReadMagnetometer(float out[3]);
int  IMU_I2C_ReadQuaternion(float out[4]);
int  IMU_I2C_ReadEuler(float out[3]);
int  IMU_I2C_ReadBarometer(float out[4]);
int  IMU_I2C_ReadVersion();
int  IMU_I2C_ReadAll(imu_measurement_t *out);

// Calibration functions
int  IMU_I2C_CalibrationImu(void);
int  IMU_I2C_CalibrationMag(void);

#endif
