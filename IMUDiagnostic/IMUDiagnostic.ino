/**
 * IMU Diagnostic - Raw Data Output
 * 
 * PURPOSE: Output raw IMU sensor data and firmware-calculated Euler angles
 *          to help diagnose the yaw jump issue.
 * 
 * HOW TO USE:
 * 1. Upload this code to Arduino Giga R1 WiFi
 * 2. Open Serial Monitor at 115200 baud
 * 3. Rotate the IMU slowly and watch the output
 * 4. Pay attention to Euler yaw vs quaternion values
 */

#include "Wire.h"
#include "imu_i2c_driver.hpp"

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("\n========================================");
    Serial.println("  IMU Diagnostic - Raw Data Output");
    Serial.println("========================================\n");
    
    // Initialize I2C
    Wire.begin();
    delay(100);
    
    // Check IMU connection
    Serial.println("[1] Checking IMU connection...");
    int result = IMU_I2C_ReadVersion();
    if (result != 0) {
        Serial.println("ERROR: IMU not found!");
        while (1) { delay(1000); }
    }
    
    Serial.println("[2] IMU found! Initializing...");
    
    // Start data output
    result = IMU_I2C_SendCommand(IMU_FUNC_REQUEST_DATA, 0);
    if (result != 0) {
        Serial.println("ERROR: Failed to request data");
        while (1) { delay(1000); }
    }
    
    delay(500);
    Serial.println("[3] Streaming sensor data...\n");
    Serial.println("Rotate the IMU and observe the output.\n");
    
    // Print header
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("TIME");
    Serial.print("\tRAW_MAG_X\tRAW_MAG_Y\tRAW_MAG_Z");
    Serial.print("\tEULER_YAW\t\tQUAT_W\t\tQUAT_X\t\tQUAT_Y\t\tQUAT_Z");
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------");
}

unsigned long startTime = 0;

void loop() {
    imu_measurement_t data;
    
    if (IMU_I2C_ReadAll(&data) != 0) {
        Serial.println("ERROR: Failed to read IMU");
        delay(100);
        return;
    }
    
    // Calculate time since start
    unsigned long elapsed = millis() - startTime;
    
    // Output in tab-separated format for easy analysis
    Serial.print(elapsed);
    Serial.print("\t");
    Serial.print(data.mag[0], 4);
    Serial.print("\t\t");
    Serial.print(data.mag[1], 4);
    Serial.print("\t\t");
    Serial.print(data.mag[2], 4);
    Serial.print("\t\t");
    Serial.print(data.euler[2], 4);      // Firmware Euler Yaw (degrees)
    Serial.print("\t\t");
    Serial.print(data.quat[0], 6);       // Quaternion W
    Serial.print("\t\t");
    Serial.print(data.quat[1], 6);       // Quaternion X
    Serial.print("\t\t");
    Serial.print(data.quat[2], 6);       // Quaternion Y
    Serial.print("\t\t");
    Serial.println(data.quat[3], 6);     // Quaternion Z
    
    delay(100); // 10 Hz update rate
}
