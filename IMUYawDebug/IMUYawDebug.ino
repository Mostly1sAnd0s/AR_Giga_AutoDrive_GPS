/**
 * IMU Debug Yaw - Detailed Quaternion Analysis
 * 
 * PURPOSE: Compare my yaw calculation vs IMU firmware output byte-by-byte
 */

#include "Wire.h"
#include "imu_i2c_driver.hpp"

float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Try multiple yaw formulas to find the correct one
float calculateYawFormula1(float qw, float qx, float qy, float qz) {
    // Standard Z-axis yaw formula
    float yaw = atan2f(2.0f * (qw * qz + qx * qy), 
                       1.0f - 2.0f * (qz * qz + qy * qy));
    return normalizeAngle(yaw * 57.2957795f);
}

float calculateYawFormula2(float qw, float qx, float qy, float qz) {
    // Alternative: swap Y and Z (different axis convention)
    float yaw = atan2f(2.0f * (qw * qy + qx * qz), 
                       1.0f - 2.0f * (qy * qy + qz * qz));
    return normalizeAngle(yaw * 57.2957795f);
}

float calculateYawFormula3(float qw, float qx, float qy, float qz) {
    // Alternative: X-axis rotation (different convention)
    float yaw = atan2f(2.0f * (qw * qx + qy * qz), 
                       1.0f - 2.0f * (qx * qx + qy * qy));
    return normalizeAngle(yaw * 57.2957795f);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("\n========================================");
    Serial.println("  IMU Yaw Debug - Multiple Formulas");
    Serial.println("========================================\n");
    
    Wire.begin();
    delay(100);
    
    Serial.println("[1] Checking IMU...");
    int result = IMU_I2C_ReadVersion();
    if (result != 0) {
        Serial.println("ERROR: IMU not found!");
        while (1) { delay(1000); }
    }
    
    Serial.println("[2] IMU found! Starting stream...\n");
    result = IMU_I2C_SendCommand(IMU_FUNC_REQUEST_DATA, 0);
    
    Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("FW_YAW\t| MY_F1\t\tMY_F2\t\tMY_F3\t\t| W\t\tX\t\tY\t\tZ");
    Serial.println();
    Serial.println("--------------------------------------------------------------------------------------------------------------------------------------------------");
}

void loop() {
    imu_measurement_t data;
    
    if (IMU_I2C_ReadAll(&data) != 0) {
        delay(100);
        return;
    }
    
    float fwYaw = normalizeAngle(data.euler[2]);
    float myYaw1 = calculateYawFormula1(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
    float myYaw2 = calculateYawFormula2(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
    float myYaw3 = calculateYawFormula3(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
    
    // Output aligned columns
    Serial.print(fwYaw, 1);
    Serial.print("\t|\t");
    Serial.print(myYaw1, 1);
    Serial.print("\t\t");
    Serial.print(myYaw2, 1);
    Serial.print("\t\t");
    Serial.print(myYaw3, 1);
    Serial.print("\t\t| ");
    Serial.print(data.quat[0], 5);
    Serial.print("\t");
    Serial.print(data.quat[1], 5);
    Serial.print("\t");
    Serial.print(data.quat[2], 5);
    Serial.print("\t");
    Serial.println(data.quat[3], 5);
    
    delay(200);
}
