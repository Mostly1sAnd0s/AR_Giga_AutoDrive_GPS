/*
 * Example 2: IMU Heading Maintenance - FIXED VERSION
 * ===================================================
 * 
 * This version properly applies magnetometer offsets and normalizes yaw.
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - Yahboom 9-axis IMU connected to I2C (Wire)
 *   - Steering Servo connected to Pin 7
 *   - ESC connected to Pin 6
 */

#include <Servo.h>
#include <Wire.h>

// Include the IMU driver from class libraries
#include "imu_i2c_driver.hpp"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define STEERING_SERVO_PIN 7    // Servo that controls wheel direction
#define ESC_PIN 6               // Electronic Speed Controller (throttle)

// ============================================================================
// SERVO SETTINGS - ADJUST THESE FOR YOUR VEHICLE
// ============================================================================

#define CENTER_STEERING 90      // Servo PWM value when wheels are straight
#define MIN_STEERING   45       // Minimum servo value (full left turn)
#define MAX_STEERING  135       // Maximum servo value (full right turn)

// ESC Settings
#define ESC_STOP        90      // PWM value to stop motor
#define ESC_FORWARD    100      // PWM value for slow forward motion

// ============================================================================
// IMU SETTINGS - CALIBRATION VALUES FROM FIGURE-8 METHOD
// ============================================================================

// Magnetometer offsets from calibration (UPDATE THESE AFTER CALIBRATION)
float MAG_OFFSET_X = 42.3353;   // From your calibration run
float MAG_OFFSET_Y = 8.7161;    // From your calibration run
float MAG_OFFSET_Z = 23.6091;   // From your calibration run

// North alignment offset (will be calculated after mag calibration)
float NORTH_OFFSET = 0.0;

// Control parameters
#define HEADING_GAIN     25.0    // Proportional gain for heading error (radians)
#define CONTROL_RATE_MS  50      // How often to update steering (milliseconds)

// ============================================================================
// TARGET HEADING
// ============================================================================

float targetHeading = 0.0;  // North (will be set after calibration)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Servo steeringServo;  // Servo for steering
Servo escServo;       // Servo for ESC (throttle)

float currentYaw = 0;      // Current heading from IMU (radians, normalized)
float rawYaw = 0;          // Raw yaw before normalization

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== IMU Heading Maintenance - FIXED ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize I2C bus
  Wire.begin();
  delay(100);
  
  // Attach servos to pins
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start with vehicle stopped and wheels centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  // Initialize IMU
  Serial.println(F("Checking IMU connection..."));
  int imuResult = IMU_I2C_ReadVersion();
  
  if (imuResult != 0) {
    Serial.println(F("ERROR: IMU not found! Check wiring."));
    Serial.println(F("  - Verify I2C connections (SDA, SCL)"));
    Serial.println(F("  - Check power to IMU module"));
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println(F("IMU connected successfully!"));
  
  // =====================================================
  // CALIBRATION 1: Static Zeroing (Gyro & Accelerometer)
  // =====================================================
  Serial.println(F("\n>>> STEP 1: Static Calibration"));
  Serial.println(F("KEEP VEHICLE PERFECTLY STILL for 15 seconds..."));
  
  delay(3000);
  
  int calResult = IMU_I2C_CalibrationImu();
  if (calResult == 0) {
    Serial.println(F("Static calibration complete!"));
  } else {
    Serial.println(F("WARNING: Calibration failed, continuing anyway..."));
  }
  
  // =====================================================
  // CALIBRATION 2: Magnetometer Figure-8 Method
  // =====================================================
  Serial.println(F("\n>>> STEP 2: Magnetometer Calibration"));
  Serial.println(F("========================================"));
  Serial.println(F("PICK UP THE VEHICLE and rotate in a FIGURE-8 pattern"));
  Serial.println(F("in ALL THREE AXES (roll, pitch, yaw)"));
  Serial.println(F("Continue for ~30 seconds until calibration completes"));
  Serial.println(F("========================================"));
  
  delay(5000);
  
  Serial.println(F("\nStarting magnetometer calibration..."));
  Serial.println(F("Rotate slowly in figure-8 pattern..."));
  
  int magCalResult = IMU_I2C_CalibrationMag();
  
  if (magCalResult == 0) {
    Serial.println(F("\n>>> Magnetometer calibration complete!"));
    
    // Read the calculated offsets from IMU
    float offsets[3];
    IMU_I2C_ReadMagOffsets(offsets);
    
    MAG_OFFSET_X = offsets[0];
    MAG_OFFSET_Y = offsets[1];
    MAG_OFFSET_Z = offsets[2];
    
    Serial.println(F("\n>>> Saved Magnetometer Offsets:"));
    Serial.print(F("  X: ")); Serial.println(MAG_OFFSET_X, 4);
    Serial.print(F("  Y: ")); Serial.println(MAG_OFFSET_Y, 4);
    Serial.print(F("  Z: ")); Serial.println(MAG_OFFSET_Z, 4);
    
    // IMPORTANT: Write offsets back to IMU RAM so they're applied automatically
    Serial.println(F("\n>>> Writing offsets to IMU RAM..."));
    int writeResult = IMU_I2C_SetMagOffsets(offsets);
    if (writeResult == 0) {
      Serial.println(F("Offsets written successfully!"));
    } else {
      Serial.println(F("WARNING: Could not write offsets to IMU"));
    }
    
    Serial.println(F("\n>>> IMPORTANT: Copy these values to your code!"));
    Serial.println(F("   Add them as MAG_OFFSET_X/Y/Z for permanent use."));
  } else {
    Serial.println(F("\nWARNING: Magnetometer calibration failed!"));
    Serial.println(F("Try again - make sure you rotate in all 3 axes"));
  }
  
  // =====================================================
  // CALIBRATION 3: North Alignment
  // =====================================================
  Serial.println(F("\n>>> STEP 3: North Alignment"));
  Serial.println(F("Point vehicle physically NORTH (use phone compass)"));
  delay(5000);
  
  float euler[3];
  int samples = 20;
  float sumYaw = 0;
  
  Serial.println(F("Taking 20 samples to average..."));
  for (int i = 0; i < samples; i++) {
    if (IMU_I2C_ReadEuler(euler) == 0) {
      rawYaw = euler[2];
      
      // Normalize yaw to [-PI, +PI] radians before averaging
      while (rawYaw > PI)   rawYaw -= 2 * PI;
      while (rawYaw < -PI)  rawYaw += 2 * PI;
      
      sumYaw += rawYaw;
      
      // Print every 5th sample to show progress (in degrees)
      if ((i + 1) % 5 == 0) {
        Serial.print(F("  Sample "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(rawYaw * 180.0 / PI, 2);  // Already normalized
        Serial.println(F("°"));
      }
    }
    delay(200);
  }
  
  float avgYaw = sumYaw / samples;
  Serial.println(F("\n>>> Average yaw when facing North: "));
  Serial.print(avgYaw * 180.0 / PI, 2);
  Serial.println(F("°"));
  
  // Set north offset so yaw = 0 when facing North
  NORTH_OFFSET = -avgYaw;
  Serial.println(F(">>> Setting NORTH_OFFSET: "));
  Serial.print(NORTH_OFFSET * 180.0 / PI, 2);
  Serial.println(F("°"));
  
  // Verify the offset works
  if (IMU_I2C_ReadEuler(euler) == 0) {
    currentYaw = euler[2] + NORTH_OFFSET;
    
    // Normalize to [-PI, +PI]
    while (currentYaw > PI)   currentYaw -= 2 * PI;
    while (currentYaw < -PI)  currentYaw += 2 * PI;
    
    Serial.println(F("\n>>> Verification:"));
    Serial.print(F("  Corrected yaw when facing North: "));
    Serial.print(currentYaw * 180.0 / PI, 2);
    Serial.println(F("° (should be ~0°)"));
  }
  
  // Set target heading to North (0 radians)
  targetHeading = 0.0;
  Serial.println(F("\n>>> Target heading set to: NORTH (0°)"));
  
  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Vehicle will now maintain North heading."));
  Serial.println(F("Press reset to re-calibrate."));
  
  delay(2000);
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================

void loop() {
  // =====================================================
  // STEP 1: Read IMU data
  // =====================================================
  float euler[3];  // [0]=roll, [1]=pitch, [2]=yaw (radians)
  
  if (IMU_I2C_ReadEuler(euler) != 0) {
    Serial.println(F("ERROR: Failed to read IMU"));
    delay(100);
    return;
  }
  
  // Extract yaw and apply north offset
  rawYaw = euler[2] + NORTH_OFFSET;
  
  // IMPORTANT: Normalize yaw to [-PI, +PI] radians
  while (rawYaw > PI)   rawYaw -= 2 * PI;
  while (rawYaw < -PI)  rawYaw += 2 * PI;
  
  currentYaw = rawYaw;
  
  // =====================================================
  // STEP 2: Calculate heading error
  // =====================================================
  float error = targetHeading - currentYaw;
  
  // Normalize error to range [-PI, +PI]
  while (error > PI)    error -= 2 * PI;
  while (error < -PI)   error += 2 * PI;
  
  // =====================================================
  // STEP 3: Calculate steering using proportional control
  // =====================================================
  int steeringPosition = CENTER_STEERING + (int)(error * HEADING_GAIN);
  steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
  
  // =====================================================
  // STEP 4: Apply controls
  // =====================================================
  steeringServo.write(steeringPosition);
  escServo.write(ESC_FORWARD);
  
  // =====================================================
  // STEP 5: Print telemetry (every 200ms)
  // =====================================================
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    
    Serial.print(F("Yaw: "));
    Serial.print(currentYaw * 180.0 / PI, 1);  // Convert to degrees
    Serial.print(F("° | Target: "));
    Serial.print(targetHeading * 180.0 / PI, 1);
    Serial.print(F("° | Error: "));
    Serial.print(error * 180.0 / PI, 1);
    Serial.print(F("° | Steer: "));
    Serial.println(steeringPosition);
  }
  
  // Control loop timing
  delay(CONTROL_RATE_MS);
}

/*
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: Yaw still shows crazy values (>360°)
 *   Fix: Make sure IMU_I2C_SetMagOffsets() was called after calibration
 *   
 * Problem: Vehicle turns opposite direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 *   
 * Problem: Heading drifts over time
 *   Fix: This is normal for dead reckoning - use GPS for correction (Example 4)
 * 
 * ============================================================================
 */
