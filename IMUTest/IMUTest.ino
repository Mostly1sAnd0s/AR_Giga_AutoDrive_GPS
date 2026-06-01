/**
 * IMU Test - Stable Heading Demonstration
 * 
 * PURPOSE: Demonstrates stable heading calculation from 9-axis IMU.
 * 
 * KEY FIX: Uses angle-aware smoothing to handle ±180° wrap-around correctly.
 * The IMU's native Euler yaw output is smooth and accurate - no quaternion math needed!
 * 
 * HARDWARE: Arduino Giga R1 WiFi + Yahboom 9-axis IMU (I2C address 0x23)
 */

#include "Wire.h"
#include "imu_i2c_driver.hpp"

// ============================================================================
// CALIBRATION OFFSETS (Your calibrated values)
// ============================================================================
float magOffset[3] = {-3.1251, 0.6836, -4.1505};
float northOffset = 0.0;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/** Normalize angle to [-180°, +180°] */
float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}



/** Calculate heading from raw magnetometer with offsets */
float calculateMagneticHeading(float magX, float magY) {
    float magX_cal = magX - magOffset[0];
    float magY_cal = magY - magOffset[1];
    float heading = atan2(magY_cal, magX_cal) * 57.2957795f;
    heading = normalizeAngle(heading);
    heading += northOffset;
    return normalizeAngle(heading);
}

/** Angle-aware smoothing filter - handles ±180° wrap-around */
float smoothAngle(float newAngle, float &smoothedAngle, float alpha) {
    // Normalize the difference to account for wrap-around
    float diff = newAngle - smoothedAngle;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    // Apply smoothing to the normalized difference
    smoothedAngle += alpha * diff;
    
    // Final normalization
    return normalizeAngle(smoothedAngle);
}

// ============================================================================
// SETUP AND LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("\n========================================");
    Serial.println("  IMU Test - Continuous Yaw Tracking");
    Serial.println("========================================\n");
    
    Wire.begin();
    delay(100);
    
    Serial.println("[1] Checking IMU connection...");
    int result = IMU_I2C_ReadVersion();
    if (result != 0) {
        Serial.println("ERROR: IMU not found!");
        while (1) { delay(1000); }
    }
    
    Serial.println("[2] IMU found! Initializing...");
    result = IMU_I2C_SendCommand(IMU_FUNC_REQUEST_DATA, 0);
    if (result != 0) {
        Serial.println("ERROR: Failed to request data");
        while (1) { delay(1000); }
    }
    
    // Wait for IMU to initialize and start streaming
    delay(500);
    Serial.println("[2a] Waiting for data stream to stabilize...");
    delay(500);
    Serial.println("[3] Calibration offsets applied:");
    Serial.print("  magOffset = {"); Serial.print(magOffset[0], 2);
    Serial.print(", "); Serial.print(magOffset[1], 2);
    Serial.print(", "); Serial.println(magOffset[2], 2); Serial.println("}");
    Serial.println("\n[4] Streaming data... Rotate through ±180° to test.\n");
    delay(1000);
}

static float lastYaw = 0.0f;
static float continuousYaw = 0.0f;
bool initialized = false;

void loop() {
    imu_measurement_t data;
    
    if (IMU_I2C_ReadAll(&data) != 0) {
        Serial.println("ERROR: Failed to read IMU");
        delay(100);
        return;
    }
    
    // ========================================================================
    // READ SENSOR DATA
    // ========================================================================
    
    // Magnetic heading for comparison
    float magHeading = calculateMagneticHeading(data.mag[0], data.mag[1]);
    
    // Use IMU's native Euler yaw (it's smooth and accurate!)
    float imuYaw = data.euler[2];  // Already in degrees, normalized to [-180°, +180°]
    
    // Calculate continuous yaw without wrap jumps (delta-based tracking)
    if (!initialized) {
        lastYaw = imuYaw;
        continuousYaw = imuYaw;
        initialized = true;
    } else {
        float delta = imuYaw - lastYaw;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        continuousYaw += delta;
        lastYaw = imuYaw;
    }
    
    // Normalize continuous yaw to ±360° for display (optional)
    float normalizedContinuousYaw = continuousYaw;
    while (normalizedContinuousYaw > 360.0f) normalizedContinuousYaw -= 360.0f;
    while (normalizedContinuousYaw < -360.0f) normalizedContinuousYaw += 360.0f;
    
    // ========================================================================
    // DISPLAY OUTPUT
    // ========================================================================
    
    Serial.println("\n========================================");
    Serial.println("  RAW QUATERNION (debug)");
    Serial.println("========================================");
    Serial.print("Quaternion:            W="); Serial.print(data.quat[0], 5);
    Serial.print("  X="); Serial.print(data.quat[1], 5);
    Serial.print("  Y="); Serial.print(data.quat[2], 5);
    Serial.print("  Z="); Serial.println(data.quat[3], 5);
    
    Serial.print("Accelerometer [g]:     X="); Serial.print(data.accel[0], 3);
    Serial.print("  Y="); Serial.print(data.accel[1], 3);
    Serial.print("  Z="); Serial.println(data.accel[2], 3);
    
    Serial.print("Gyroscope [rad/s]:     X="); Serial.print(data.gyro[0], 4);
    Serial.print("  Y="); Serial.print(data.gyro[1], 4);
    Serial.print("  Z="); Serial.println(data.gyro[2], 4);
    
    // DEBUG: Print raw magnetometer bytes from driver
    Serial.print("Magnetometer [uT]:     X="); Serial.print(data.mag[0], 3);
    Serial.print("  Y="); Serial.print(data.mag[1], 3);
    Serial.print("  Z="); Serial.println(data.mag[2], 3);
    

    
    // ========================================================================
    // HEADING OUTPUT - CONTINUOUS TRACKING
    // ========================================================================
    
    Serial.println("\n========================================");
    Serial.println("  HEADING OUTPUT");
    Serial.println("========================================");
    
    Serial.print("IMU Euler Yaw:         ");
    Serial.print(imuYaw, 1);
    Serial.println("° (native IMU output)");
    
    Serial.print("Continuous Yaw:        ");
    Serial.print(continuousYaw, 1);
    Serial.println("° (smooth tracking)");
    
    // Apply angle-aware smoothing to Mag Heading
    static float smoothedMagHeading = 0.0f;
    smoothedMagHeading = smoothAngle(magHeading, smoothedMagHeading, 0.3f);
    
    Serial.print("Mag Heading:           ");
    Serial.print(smoothedMagHeading, 1);
    Serial.println("° (smoothed with wrap-aware)");
    
    // ========================================================================
    // QUALITY INDICATORS
    // ========================================================================
    
    Serial.println("\n========================================");
    Serial.println("  QUALITY CHECKS");
    Serial.println("========================================");
    
    float accelMag = sqrt(data.accel[0]*data.accel[0] + 
                          data.accel[1]*data.accel[1] + 
                          data.accel[2]*data.accel[2]);
    Serial.print("Accelerometer mag:     ");
    Serial.print(accelMag, 3);
    Serial.println(" g (should be ~1.0)");
    
    float gyroMag = sqrt(data.gyro[0]*data.gyro[0] + 
                         data.gyro[1]*data.gyro[1] + 
                         data.gyro[2]*data.gyro[2]);
    Serial.print("Gyroscope mag:         ");
    Serial.print(gyroMag, 4);
    Serial.println(" rad/s (should be ~0)");
    
    // Stability check using normalized continuous yaw
    static float prevNormalizedYaw = 0.0f;
    static int stableCount = 0;
    if (initialized) {
        float yawDelta = abs(normalizeAngle(normalizedContinuousYaw - prevNormalizedYaw));
        if (yawDelta < 1.0f) stableCount++; else stableCount = 0;
        prevNormalizedYaw = normalizedContinuousYaw;
    }
    
    Serial.print("Heading stability:     ");
    if (stableCount > 10) Serial.println("STABLE");
    else if (stableCount > 5) Serial.println("MODERATE");
    else Serial.println("UNSTABLE");
    
    Serial.println("\n----------------------------------------");
    Serial.println("TIPS:");
    Serial.println("- Yaw should be SMOOTH through ±180°");
    Serial.println("- No jumps when rotating past South!");
    Serial.println("----------------------------------------");
    
    delay(200);
}
