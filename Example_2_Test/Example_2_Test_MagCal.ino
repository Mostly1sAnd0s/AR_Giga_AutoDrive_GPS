/*
 * Example 2: IMU Heading Maintenance - WITH MAGNETOMETER CALIBRATION
 * ===================================================================
 * 
 * This version includes proper magnetometer calibration using the figure-8 method.
 * The magnetometer must be calibrated separately from gyro/accelerometer.
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
// IMU SETTINGS
// ============================================================================

// Calibration offsets - WILL BE AUTO-CALIBRATED, then saved here
float MAG_OFFSET_X = 0.0;
float MAG_OFFSET_Y = 0.0;
float MAG_OFFSET_Z = 0.0;

// North alignment offset (set after mag calibration)
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

float currentYaw = 0;      // Current heading from IMU (radians)
float lastYaw = 0;         // Previous yaw for gyro integration
unsigned long lastReadTime = 0;

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== IMU Heading Maintenance - WITH MAG CAL ==="));
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
  Serial.println(F("This zeros the gyro to prevent drift."));
  
  delay(3000);  // Give you time to set vehicle down
  
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
  
  delay(5000);  // Give you time to pick up vehicle
  
  Serial.println(F("\nStarting magnetometer calibration..."));
  Serial.println(F("Rotate slowly in figure-8 pattern..."));
  
  int magCalResult = IMU_I2C_CalibrationMag();
  
  if (magCalResult == 0) {
    Serial.println(F("\n>>> Magnetometer calibration complete!"));
    
    // Read the calculated offsets
    float offsets[3];
    IMU_I2C_ReadMagOffsets(offsets);
    
    MAG_OFFSET_X = offsets[0];
    MAG_OFFSET_Y = offsets[1];
    MAG_OFFSET_Z = offsets[2];
    
    Serial.println(F("\n>>> Saved Magnetometer Offsets:"));
    Serial.print(F("  X: ")); Serial.println(MAG_OFFSET_X, 4);
    Serial.print(F("  Y: ")); Serial.println(MAG_OFFSET_Y, 4);
    Serial.print(F("  Z: ")); Serial.println(MAG_OFFSET_Z, 4);
    
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
      float rawYaw = euler[2];
      sumYaw += rawYaw;
      
      // Print every 5th sample to show progress
      if ((i + 1) % 5 == 0) {
        Serial.print(F("  Sample "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(rawYaw * 180.0 / PI, 2);
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
  unsigned long currentTime = millis();
  
  // =====================================================
  // STEP 1: Read IMU data
  // =====================================================
  float euler[3];  // [0]=roll, [1]=pitch, [2]=yaw (radians)
  
  if (IMU_I2C_ReadEuler(euler) != 0) {
    Serial.println(F("ERROR: Failed to read IMU"));
    delay(100);
    return;
  }
  
  // Extract yaw (heading) and apply calibration offsets
  currentYaw = euler[2] + NORTH_OFFSET;
  
  // Normalize yaw to range [-PI, +PI]
  while (currentYaw > PI)   currentYaw -= 2 * PI;
  while (currentYaw < -PI)  currentYaw += 2 * PI;
  
  // =====================================================
  // STEP 2: Calculate heading error
  // =====================================================
  float error = targetHeading - currentYaw;
  
  // Normalize error to range [-PI, +PI]
  // This handles wrap-around at ±180 degrees
  while (error > PI)    error -= 2 * PI;
  while (error < -PI)   error += 2 * PI;
  
  // =====================================================
  // STEP 3: Calculate steering using proportional control
  // =====================================================
  // Steering = center + (error * gain)
  int steeringPosition = CENTER_STEERING + (int)(error * HEADING_GAIN);
  
  // Constrain to valid servo range
  steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
  
  // =====================================================
  // STEP 4: Apply controls
  // =====================================================
  steeringServo.write(steeringPosition);
  escServo.write(ESC_FORWARD);
  
  // =====================================================
  // STEP 5: Print telemetry (every 200ms)
  // =====================================================
  if (currentTime - lastReadTime > 200) {
    lastReadTime = currentTime;
    
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
 * MAGNETOMETER CALIBRATION GUIDE
 * ============================================================================
 * 
 * WHY CALIBRATION IS IMPORTANT:
 * - Magnetometers measure local magnetic field, not just Earth's field
 * - Motors, batteries, metal chassis create magnetic distortions
 * - Hard iron: constant offset (fixed magnets or magnetized metal)
 * - Soft iron: scaling/rotation distortion (metal nearby)
 * 
 * FIGURE-8 CALIBRATION PROCESS:
 * 1. Pick up the vehicle (don't keep it on the ground)
 * 2. Rotate slowly in figure-8 pattern
 * 3. Make sure to rotate in ALL THREE AXES:
 *    - Roll (tilt left/right)
 *    - Pitch (tilt forward/back)
 *    - Yaw (rotate horizontally)
 * 4. Continue for ~30 seconds until calibration completes
 * 5. Note the offset values printed
 * 6. Add them to your code permanently
 * 
 * PERMANENT CALIBRATION:
 * After successful calibration, add these lines to your code:
 * 
 *   #define MAG_OFFSET_X  XX.XXXX
 *   #define MAG_OFFSET_Y  YY.YYYY
 *   #define MAG_OFFSET_Z  ZZ.ZZZZ
 *   #define NORTH_OFFSET  NN.NNNN
 * 
 * Then in setup(), skip the calibration steps and use these values directly.
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: Calibration still varies after figure-8
 *   Fix: 
 *     - Move away from metal objects, cars, buildings
 *     - Keep away from motors and power wires (turn them off during calibration)
 *     - Rotate more slowly and cover all 3 axes
 *     - Try recalibrating in a different location
 * 
 * Problem: Yaw drifts over time while driving
 *   Fix: This is normal! Use GPS to periodically correct heading
 *        (See Example 4 for sensor fusion)
 * 
 * Problem: Erratic steering corrections
 *   Fix: Reduce HEADING_GAIN or add D-term to smooth response
 * 
 * ============================================================================
 */
