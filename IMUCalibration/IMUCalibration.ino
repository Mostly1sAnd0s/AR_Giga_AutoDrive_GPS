/**
 * IMU Calibration - Magnetometer Offset Finder
 * 
 * PURPOSE: Find accurate magnetometer calibration offsets using the figure-8 method.
 * 
 * HOW IT WORKS:
 * 1. This sketch tracks minimum and maximum magnetometer readings in all 3 axes
 * 2. You rotate the device in a figure-8 pattern in all 3D orientations
 * 3. When done, it calculates offsets as: offset[i] = (max[i] + min[i]) / 2
 * 
 * CALIBRATION PROCEDURE:
 * 1. Upload this code to Arduino Giga R1 WiFi
 * 2. Open Serial Monitor at 115200 baud
 * 3. Press 'c' to START calibration (clears all tracking)
 * 4. Rotate device in figure-8 pattern for ~30 seconds:
 *    - Rotate around all 3 axes (X, Y, Z)
 *    - Try to cover full 3D space
 *    - Keep rotating until values stabilize
 * 5. Press 's' to STOP and calculate offsets
 * 6. Copy the printed offset values into your main code
 * 
 * OUTPUT:
 * - Live display of raw magnetometer data
 * - Min/Max tracking for each axis
 * - Calculated calibration offsets at the end
 * 
 * FOR FUTURE DEVELOPERS:
 * These offsets should be copied into IMUTest.ino and production code:
 *   float magOffset[3] = {X_offset, Y_offset, Z_offset};
 */

#include "Wire.h"
#include "IMU_I2C_Driver.h"

// ============================================================================
// CALIBRATION STATE
// ============================================================================

bool calibrationRunning = false;
unsigned long calibrationStartTime = 0;

// Magnetometer min/max tracking
float magMin[3] = {1000.0f, 1000.0f, 1000.0f};  // Initialize to large values
float magMax[3] = {-1000.0f, -1000.0f, -1000.0f}; // Initialize to small values

// Sample count for averaging
int sampleCount = 0;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void resetCalibration() {
    magMin[0] = magMin[1] = magMin[2] = 1000.0f;
    magMax[0] = magMax[1] = magMax[2] = -1000.0f;
    sampleCount = 0;
    calibrationRunning = false;
}

void updateMinMax(float mag[3]) {
    for (int i = 0; i < 3; i++) {
        if (mag[i] < magMin[i]) magMin[i] = mag[i];
        if (mag[i] > magMax[i]) magMax[i] = mag[i];
    }
    sampleCount++;
}

void printCalibrationResults() {
    Serial.println("\n");
    Serial.println("========================================================================");
    Serial.println("  CALIBRATION COMPLETE!");
    Serial.println("========================================================================");
    Serial.println();
    
    // Calculate offsets (center of min/max range)
    float offset[3];
    float range[3];
    
    Serial.println("Axis   | Min         | Max         | Range     | Offset");
    Serial.println("-------|-------------|-------------|-----------|-------------");
    
    const char* labels[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; i++) {
        offset[i] = (magMax[i] + magMin[i]) / 2.0f;
        range[i] = magMax[i] - magMin[i];
        
        Serial.print(labels[i]);
        Serial.print("     | ");
        Serial.print(magMin[i], 3);
        Serial.print(" uT | ");
        Serial.print(magMax[i], 3);
        Serial.print(" uT | ");
        Serial.print(range[i], 3);
        Serial.print(" uT | ");
        Serial.print(offset[i], 3);
        Serial.println(" uT");
    }
    
    Serial.println();
    Serial.println("========================================================================");
    Serial.println("  COPY THESE VALUES INTO YOUR CODE:");
    Serial.println("========================================================================");
    Serial.println();
    Serial.println("// Magnetometer calibration offsets");
    Serial.print("float magOffset[3] = {");
    Serial.print(offset[0], 4);
    Serial.print(", ");
    Serial.print(offset[1], 4);
    Serial.print(", ");
    Serial.print(offset[2], 4);
    Serial.println("};");
    Serial.println();
    
    // Quality check
    Serial.println("========================================================================");
    Serial.println("  CALIBRATION QUALITY CHECK:");
    Serial.println("========================================================================");
    Serial.print("Total samples: ");
    Serial.println(sampleCount);
    
    float minRange = range[0];
    if (range[1] < minRange) minRange = range[1];
    if (range[2] < minRange) minRange = range[2];
    
    if (minRange > 30.0f && sampleCount > 500) {
        Serial.println("Quality: EXCELLENT - Good coverage, reliable offsets");
    } else if (minRange > 15.0f && sampleCount > 200) {
        Serial.println("Quality: GOOD - Acceptable for most applications");
    } else if (sampleCount > 100) {
        Serial.println("Quality: FAIR - Consider recalibrating with more rotation");
    } else {
        Serial.println("Quality: POOR - Too few samples, recalibrate recommended");
    }
    
    Serial.println();
    Serial.println("========================================================================");
}

void printInstructions() {
    Serial.println("\n");
    Serial.println("========================================================================");
    Serial.println("  IMU MAGNETOMETER CALIBRATION");
    Serial.println("========================================================================");
    Serial.println();
    Serial.println("INSTRUCTIONS:");
    Serial.println("1. Press 'c' to START calibration (clears previous data)");
    Serial.println("2. Rotate device in FIGURE-8 pattern for 30+ seconds:");
    Serial.println("   - Rotate around ALL THREE axes (X, Y, Z)");
    Serial.println("   - Try to cover full 3D space");
    Serial.println("   - Keep rotating until values stabilize");
    Serial.println("3. Press 's' to STOP and calculate offsets");
    Serial.println("4. Copy the printed offset values into your code");
    Serial.println();
    Serial.println("TIPS:");
    Serial.println("- Stay away from metal objects and electronics");
    Serial.println("- Hold device firmly, avoid shaking");
    Serial.println("- The more you rotate, the better the calibration");
    Serial.println();
    Serial.println("CURRENT STATUS: Ready to calibrate");
    Serial.println("========================================================================");
}

// ============================================================================
// SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    printInstructions();
    
    // Initialize I2C
    Wire.begin();
    delay(100);
    
    Serial.println("\n[1] Checking IMU connection...");
    
    // Try to read version to verify IMU is connected
    int result = IMU_I2C_ReadVersion();
    if (result != 0) {
        Serial.println("ERROR: IMU not found! Check wiring:");
        Serial.println("  - SDA pin connected");
        Serial.println("  - SCL pin connected");
        Serial.println("  - IMU powered correctly");
        Serial.println("  - I2C address is 0x23");
        while (1) { 
            Serial.println("  Retrying in 5 seconds...");
            delay(5000);
            result = IMU_I2C_ReadVersion();
            if (result == 0) break;
        }
    }
    
    Serial.println("[2] IMU found! Initializing...");
    
    // Send request command to start data output
    result = IMU_I2C_SendCommand(IMU_FUNC_REQUEST_DATA, 0);
    if (result != 0) {
        Serial.println("ERROR: Failed to request IMU data");
        while (1) { delay(1000); }
    }
    
    delay(500); // Wait for IMU to stabilize
    
    Serial.println("[3] Ready! Press 'c' to start calibration.");
    printInstructions();
}

void loop() {
    imu_measurement_t data;
    
    // Read all sensor data
    if (IMU_I2C_ReadAll(&data) != 0) {
        Serial.println("ERROR: Failed to read IMU data");
        delay(100);
        return;
    }
    
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == 'c' || cmd == 'C') {
            // Start/Reset calibration
            resetCalibration();
            calibrationRunning = true;
            calibrationStartTime = millis();
            Serial.println("\n>>> CALIBRATION STARTED <<<");
            Serial.println("Rotate in figure-8 pattern now...");
            Serial.println("Press 's' to stop and calculate offsets.");
        }
        
        if (cmd == 's' || cmd == 'S') {
            // Stop calibration and show results
            if (calibrationRunning) {
                calibrationRunning = false;
                printCalibrationResults();
                printInstructions();
            } else {
                Serial.println("\nNo active calibration to stop.");
            }
        }
        
        if (cmd == 'r' || cmd == 'R') {
            // Show current min/max
            Serial.println("\n--- Current Min/Max ---");
            for (int i = 0; i < 3; i++) {
                Serial.print("Axis ");
                Serial.print(i == 0 ? "X" : (i == 1 ? "Y" : "Z"));
                Serial.print(": Min=");
                Serial.print(magMin[i], 2);
                Serial.print(" Max=");
                Serial.print(magMax[i], 2);
                Serial.println(" uT");
            }
        }
        
        if (cmd == 'h' || cmd == 'H') {
            printInstructions();
        }
    }
    
    // If calibration is running, update min/max and display live data
    if (calibrationRunning) {
        updateMinMax(data.mag);
        
        // Display live data (every 500ms for readability)
        static unsigned long lastDisplay = 0;
        if (millis() - lastDisplay > 500) {
            lastDisplay = millis();
            
            float elapsed = (millis() - calibrationStartTime) / 1000.0f;
            
            Serial.print("\r");
            Serial.print("Time: ");
            Serial.print(elapsed, 1);
            Serial.print("s | Samples: ");
            Serial.print(sampleCount);
            Serial.print(" | Mag: X=");
            Serial.print(data.mag[0], 2);
            Serial.print(" Y=");
            Serial.print(data.mag[1], 2);
            Serial.print(" Z=");
            Serial.print(data.mag[2], 2);
            Serial.println(" uT | Min/Max range updating...");
        }
    } else {
        // Not calibrating - just show current values
        static unsigned long lastDisplay = 0;
        if (millis() - lastDisplay > 500) {
            lastDisplay = millis();
            
            Serial.print("\r");
            Serial.print("Status: IDLE | Mag: X=");
            Serial.print(data.mag[0], 2);
            Serial.print(" Y=");
            Serial.print(data.mag[1], 2);
            Serial.print(" Z=");
            Serial.print(data.mag[2], 2);
            Serial.println(" uT | Press 'c' to calibrate");
        }
    }
    
    delay(50); // ~20 Hz update rate
}

// ============================================================================
// COMMAND SUMMARY (for reference)
// ============================================================================
/*
 * SERIAL COMMANDS:
 * 
 * 'c' - START calibration (clears all min/max tracking)
 * 's' - STOP calibration and display calculated offsets
 * 'r' - SHOW current min/max values (during calibration)
 * 'h' - SHOW instructions again
 * 
 * CALIBRATION TIPS:
 * 
 * 1. Figure-8 motion works best because it covers all orientations
 * 2. Rotate around ALL THREE axes:
 *    - Rotate like a steering wheel (Z-axis)
 *    - Nod up/down (X-axis)
 *    - Tilt side to side (Y-axis)
 * 3. Calibrate away from magnetic interference:
 *    - Stay away from computers, phones, metal desks
 *    - Don't calibrate while holding phone in other hand
 * 4. More rotation = better calibration:
 *    - Aim for 30-60 seconds of continuous rotation
 *    - Watch the min/max range grow as you rotate
 * 
 * AFTER CALIBRATION:
 * 
 * Copy the printed offset values into your main code:
 * 
 *   float magOffset[3] = {X_offset, Y_offset, Z_offset};
 * 
 * Example from CHANGELOG v2.1.0 pre-loaded values:
 *   float magOffset[3] = {42.3353, 8.7161, 23.6091};
 * 
 * If your calibration gives significantly different values,
 * your IMU may have different magnetic characteristics.
 */
