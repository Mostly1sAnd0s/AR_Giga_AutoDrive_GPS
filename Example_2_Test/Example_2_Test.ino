/*
 * Example 2: IMU Heading Maintenance (Dead Reckoning) - TEST VERSION
 * ===================================================================
 * 
 * This test version will:
 * 1. Read current yaw when facing North
 * 2. Set calibration offset so yaw = 0° at North
 * 3. Maintain heading automatically
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

// This will be set automatically during calibration
float MAG_CALIBRATION_OFFSET = 98.75;

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
bool calibrationDone = false;

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== IMU Heading Maintenance - TEST ==="));
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
  // CALIBRATION: Static Zeroing (Gyro & Accelerometer)
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
  // CALIBRATION: North Alignment (Auto-detect current yaw)
  // =====================================================
  Serial.println(F("\n>>> STEP 2: North Alignment"));
  Serial.println(F("Make sure truck is facing NORTH..."));
  delay(3000);
  
  float euler[3];
  int readCount = 0;
  float sumYaw = 0;
  
  // Take multiple readings and average them
  Serial.println(F("Reading yaw values (take 10 samples)..."));
  for (int i = 0; i < 10; i++) {
    if (IMU_I2C_ReadEuler(euler) == 0) {
      float rawYaw = euler[2];
      sumYaw += rawYaw;
      Serial.print(F("  Sample "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(rawYaw * 180.0 / PI, 2);
      Serial.println(F("°"));
    }
    delay(200);
  }
  
  float avgYaw = sumYaw / 10.0;
  Serial.println(F("\n>>> Average raw yaw: "));
  Serial.print(avgYaw * 180.0 / PI, 2);
  Serial.println(F("°"));
  
  // Set calibration offset so yaw = 0 when facing North
  MAG_CALIBRATION_OFFSET = -avgYaw;
  Serial.println(F(">>> Setting MAG_CALIBRATION_OFFSET: "));
  Serial.print(MAG_CALIBRATION_OFFSET * 180.0 / PI, 2);
  Serial.println(F("°"));
  
  // Verify the offset works
  if (IMU_I2C_ReadEuler(euler) == 0) {
    currentYaw = euler[2] + MAG_CALIBRATION_OFFSET;
    Serial.println(F("\n>>> Verification:"));
    Serial.print(F("  Corrected yaw when facing North: "));
    Serial.print(currentYaw * 180.0 / PI, 2);
    Serial.println(F("° (should be ~0°)"));
  }
  
  // Set target heading to North (0 radians)
  targetHeading = 0.0;
  Serial.println(F("\n>>> Target heading set to: NORTH (0°)"));
  
  calibrationDone = true;
  
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
  
  // Extract yaw (heading) and apply calibration offset
  currentYaw = euler[2] + MAG_CALIBRATION_OFFSET;
  
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
 * Problem: Vehicle spins in circles immediately
 *   Fix: Check that MIN/MAX steering values match your servo direction
 *   
 * Problem: Vehicle turns opposite direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 *   
 * Problem: Erratic heading readings
 *   Fix: 
 *     - Calibrate magnetometer (run again with figure-8 rotation)
 *     - Keep away from large metal objects or magnets
 *     - Check I2C connections are secure
 *   
 * Problem: IMU not found
 *   Fix:
 *     - Verify I2C address (should be 0x68 for most IMUs)
 *     - Check SDA/SCL wiring
 *     - Ensure IMU has proper power (3.3V or 5V)
 * 
 * ============================================================================
 */
