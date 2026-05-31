/*
 * Example 2: IMU Heading Maintenance (Dead Reckoning)
 * ===================================================
 * 
 * This example demonstrates heading control using only the IMU (Inertial 
 * Measurement Unit), without GPS. The vehicle will maintain a set heading
 * even when GPS signal is lost.
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - Yahboom 9-axis IMU connected to I2C (Wire)
 *   - Steering Servo connected to Pin 7
 *   - ESC connected to Pin 6
 * 
 * How It Works:
 *   The IMU provides yaw (heading) angle via the magnetometer. We compare
 *   the current heading to a target heading and adjust steering proportionally.
 * 
 * Student Notes:
 *   - This is DEAD RECKONING - position drifts over time without GPS correction
 *   - Magnetometer must be calibrated for accurate heading
 *   - Works indoors where GPS doesn't work!
 *   - Magnetic interference can affect accuracy
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

// Heading offset to align IMU "North" with real North
// Adjust this so yaw reads 0° when vehicle points North
#define MAG_CALIBRATION_OFFSET  0.0   // Start at 0, calibrate later

// Control parameters
#define HEADING_GAIN     25.0    // Proportional gain for heading error (radians)
#define CONTROL_RATE_MS  50      // How often to update steering (milliseconds)

// ============================================================================
// TARGET HEADING
// ============================================================================

// Target heading in radians (0 = North, PI/2 = East, PI = South, etc.)
// You can set this to any direction you want the vehicle to maintain
float targetHeading = 0.0;  // Start facing North

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Servo steeringServo;  // Servo for steering
Servo escServo;       // Servo for ESC (throttle)

float currentYaw = 0;      // Current heading from IMU (radians)
float lastError = 0;       // Previous error (for potential D-term later)

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== IMU Heading Maintenance ==="));
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
      blinkError();
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
  // CALIBRATION: Magnetometer (Optional but recommended)
  // =====================================================
  Serial.println(F("\n>>> STEP 2: Magnetometer Check"));
  Serial.println(F("Current yaw reading: "));
  
  float euler[3];
  if (IMU_I2C_ReadEuler(euler) == 0) {
    currentYaw = euler[2];  // Yaw is the third value
    Serial.print(currentYaw * 180.0 / PI);  // Convert to degrees for display
    Serial.println(F(" degrees"));
    
    // Apply calibration offset
    currentYaw += MAG_CALIBRATION_OFFSET;
    
    // Set initial target heading to current heading
    targetHeading = currentYaw;
    Serial.print(F("Target heading set to: "));
    Serial.print(currentYaw * 180.0 / PI);
    Serial.println(F(" degrees"));
  }
  
  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Vehicle will now maintain its heading."));
  Serial.println(F("Press reset to change target heading."));
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

// ============================================================================
// HELPER FUNCTION: Blink LED to indicate error
// ============================================================================

void blinkError() {
  // Simple error indicator - could be enhanced with LED or buzzer
  static int blinkCount = 0;
  if (blinkCount++ >= 3) blinkCount = 0;
}

/*
 * ============================================================================
 * CALIBRATION GUIDE FOR STUDENTS
 * ============================================================================
 * 
 * Magnetometer Calibration (Figure-8 Method):
 *   1. Add this code to setup():
 *      IMU_I2C_CalibrationMag();
 *   2. Run the program and pick up the vehicle
 *   3. Slowly rotate it in a figure-8 pattern in ALL THREE AXES
 *   4. Continue for ~30 seconds until calibration completes
 *   5. Note the offset values printed to Serial
 *   6. Add those offsets to MAG_CALIBRATION_OFFSET
 * 
 * North Alignment:
 *   1. Point vehicle physically North (use phone compass)
 *   2. Note the yaw reading in Serial monitor
 *   3. Set MAG_CALIBRATION_OFFSET = -yaw_reading
 *   4. Now yaw should read 0° when pointing North
 * 
 * Servo Direction:
 *   If vehicle turns opposite to expected:
 *   - Swap MIN_STEERING and MAX_STEERING values
 *   - Or negate HEADING_GAIN (make it negative)
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: Vehicle spins in circles
 *   Fix: Check that MIN/MAX steering values match your servo direction
 *   
 * Problem: Heading drifts over time
 *   Fix: This is normal for dead reckoning - use GPS for correction
 *   
 * Problem: Erratic heading readings
 *   Fix: 
 *     - Calibrate magnetometer (figure-8 method)
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
