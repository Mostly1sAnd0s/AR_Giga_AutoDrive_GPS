/*
 * Example 2: Multi-Waypoint Navigation with IMU Heading
 * ======================================================
 * 
 * This example combines GPS waypoint navigation with IMU-based heading control:
 * - IMU calibration (static zeroing + magnetometer figure-8 + north alignment)
 * - Read current position from GPS
 * - Use IMU for stable heading reference (works when GPS course is unreliable)
 * - Navigate through multiple waypoints in sequence
 * - Simple proportional steering control
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - GPS Module (NEMA compatible) connected to Serial3
 *   - IMU (Yahboom 9-axis) connected to I2C (Wire)
 *   - Steering Servo connected to Pin 7
 *   - ESC (Electronic Speed Controller) connected to Pin 6
 * 
 * How It Works:
 *   The IMU provides a stable heading reference using the magnetometer.
 *   GPS provides waypoint positions. We steer toward each waypoint while
 *   maintaining the correct heading from the IMU.
 * 
 * IMPORTANT - First Time Setup:
 *   1. Upload this code
 *   2. Open Serial Monitor (115200 baud)
 *   3. Follow the calibration prompts:
 *      - Keep vehicle still for 15 seconds (static calibration)
 *      - Rotate in figure-8 pattern for ~30 seconds (mag calibration)
 *      - Point vehicle North and wait for samples (north alignment)
 *   4. Copy the printed offset values into the code below
 *   5. Re-upload with PERFORM_MAG_CALIBRATION = false
 * 
 * Student Notes:
 *   - This is a P-ONLY controller (no I or D terms)
 *   - IMU heading is more stable than GPS course for steering
 *   - No obstacle avoidance - will drive straight into obstacles!
 */

#include <Servo.h>
#include <TinyGPS++.h>
#include <Wire.h>

// ============================================================================
// CONFIGURATION - Set this to true for first-time calibration only
// ============================================================================

#define PERFORM_MAG_CALIBRATION  true   // Set false after initial calibration

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define STEERING_SERVO_PIN 7    // Servo that controls wheel direction
#define ESC_PIN 6               // Electronic Speed Controller (throttle)
#define GPS_SERIAL Serial3      // Hardware serial port for GPS

// ============================================================================
// SERVO SETTINGS - ADJUST THESE FOR YOUR VEHICLE
// ============================================================================

#define CENTER_STEERING 90      // Servo PWM value when wheels are straight
#define MIN_STEERING   70       // Minimum servo value (full left turn)
#define MAX_STEERING  130       // Maximum servo value (full right turn)

// ESC Settings
#define ESC_STOP        90      // PWM value to stop motor
#define ESC_FORWARD    105      // PWM value for slow forward motion

// Steering Direction - FLIP THIS if vehicle turns opposite direction
// Set to 1 for normal, -1 if vehicle turns wrong way
#define STEERING_DIRECTION  1

// ============================================================================
// NAVIGATION SETTINGS
// ============================================================================

#define MIN_SATELLITES  4       // Minimum satellites needed for valid fix
#define WAYPOINT_DISTANCE_THRESHOLD 2.0   // Meters - distance to consider "arrived"

// Steering gain: Higher = more aggressive turning
// Start with 0.3-0.5 and adjust based on vehicle behavior
#define STEERING_GAIN    0.4    

// ============================================================================
// IMU SETTINGS - CALIBRATION VALUES (UPDATE AFTER CALIBRATION)
// ============================================================================

// Magnetometer offsets from calibration (from Example_2_MagCalibration)
float MAG_OFFSET_X = 42.3353;   // Update after calibration run
float MAG_OFFSET_Y = 8.7161;    // Update after calibration run
float MAG_OFFSET_Z = 23.6091;   // Update after calibration run

// North alignment offset (calculated during setup)
float NORTH_OFFSET = 0.0;

// ============================================================================
// WAYPOINT ARRAY
// ============================================================================

#define NUM_WAYPOINTS 5

// IMPORTANT: Replace these with YOUR actual starting coordinates!
// Get your current GPS position from Serial Monitor and use that as waypoint 1
// Then add waypoints in FRONT of where you're standing

const double waypoints[NUM_WAYPOINTS][2] = {
  {40.34237395057527, -74.699881032483},   // Waypoint 1: START HERE (replace with your coords)
  {40.34238000000000, -74.69985000000000},   // Waypoint 2: ~4m North-East from start
  {40.34240000000000, -74.69980000000000},   // Waypoint 3: ~7m further NE
  {40.34243000000000, -74.69975000000000},   // Waypoint 4: ~7m further NE  
  {40.34246000000000, -74.69970000000000}    // Waypoint 5: ~7m further NE
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

TinyGPSPlus gps;        // GPS parsing library
Servo steeringServo;    // Servo for steering
Servo escServo;         // Servo for ESC (throttle)

int currentWaypoint = 0;  // Index of current target waypoint
float currentYaw = 0;     // Current heading from IMU (degrees, normalized)
float rawYaw = 0;         // Raw yaw before north offset

// ============================================================================
// IMU DRIVER FUNCTIONS (copied from Example_2_MagCalibration)
// ============================================================================

#define IMU_I2C_ADDRESS 0x23
#define IMU_FUNC_VERSION        0x01
#define IMU_FUNC_EULER          0x26
#define IMU_FUNC_CALIB_IMU      0x70
#define IMU_FUNC_CALIB_MAG      0x71

static int i2c_wait_calibration(uint8_t function, uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    while (1) {
        uint8_t state = 0;
        if (IMU_ReadBytes(IMU_I2C_ADDRESS, function, &state, 1) != 0) {
            return -2; 
        }
        if (state != 0) {
            return state;
        }
        if (timeout_ms != 0 && elapsed >= timeout_ms) {
            return -1;
        }
        delay(100);
        elapsed += 100;
    }
}

int IMU_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission(false);
    Wire.requestFrom(dev_addr, len);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return 0;
}

int IMU_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *buf, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(buf[i]);
    }
    Wire.endTransmission();
    return 0;
}

static int write_register(uint8_t reg, const uint8_t *register_data, uint16_t length) {
    return IMU_WriteBytes(IMU_I2C_ADDRESS, reg, register_data, length);
}

int IMU_I2C_SendCommand(uint8_t function, uint16_t value) {
    uint8_t register_data[2];
    register_data[0] = (uint8_t)(value & 0xFF);
    register_data[1] = (uint8_t)((value >> 8) & 0xFF);
    return write_register(function, register_data, 2);
}

int IMU_I2C_CalibrationImu(void) {
    uint8_t data = 0x01;
    if (write_register(IMU_FUNC_CALIB_IMU, &data, 1) != 0) {
        Serial.println("[IMU] Calibration send failed");
        return -2;
    }
    int state = i2c_wait_calibration(IMU_FUNC_CALIB_IMU, 15000);
    if (state == 1) {
        Serial.println("[IMU] Static calibration success");
        return 0;
    }
    if (state == -1) {
        Serial.println("[IMU] Calibration timeout");
        return -1;
    }
    return -state;
}

int IMU_I2C_CalibrationMag(void) {
    uint8_t data = 0x01;
    if (write_register(IMU_FUNC_CALIB_MAG, &data, 1) != 0) {
        Serial.println("[IMU] Mag calibration send failed");
        return -2;
    }
    int state = i2c_wait_calibration(IMU_FUNC_CALIB_MAG, 0);
    if (state == 1) {
        Serial.println("[IMU] Magnetometer calibration success");
        return 0;
    }
    if (state == -1) {
        Serial.println("[IMU] Mag calibration timeout");
        return -1;
    }
    return -state;
}

int IMU_I2C_ReadEuler(float out[3]) {
    uint8_t register_data[12];
    if (IMU_ReadBytes(IMU_I2C_ADDRESS, IMU_FUNC_EULER, register_data, 12) != 0) {
        return -1;
    }
    if (out != NULL) {
        // Data comes as floats in little-endian format
        out[0] = *(float*)&register_data[0];
        out[1] = *(float*)&register_data[4];
        out[2] = *(float*)&register_data[8];
    }
    return 0;
}

int IMU_I2C_ReadVersion() {
    uint8_t register_data[3];
    if (IMU_ReadBytes(IMU_I2C_ADDRESS, IMU_FUNC_VERSION, register_data, 3) != 0) {
        return -1;
    }
    Serial.print("IMU Version: ");
    Serial.print(register_data[0]);
    Serial.print(".");
    Serial.print(register_data[1]);
    Serial.print(".");
    Serial.println(register_data[2]);
    return 0;
}

int IMU_I2C_SetMagOffsets(float offsets[3]) {
    // For this IMU, offsets are stored in memory after calibration
    // This function exists for interface compatibility
    return 0;
}

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== Multi-Waypoint Navigation with IMU ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize I2C bus for IMU
  Wire.begin();
  delay(100);
  
  // Initialize GPS serial port
  GPS_SERIAL.begin(9600);  // Most GPS modules use 9600 baud
  
  // Attach servos to pins
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start with vehicle stopped and wheels centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  // =====================================================
  // IMU CALIBRATION
  // =====================================================
  Serial.println(F("\n>>> Checking IMU connection..."));
  int imuResult = IMU_I2C_ReadVersion();
  
  if (imuResult != 0) {
    Serial.println(F("ERROR: IMU not found! Check wiring."));
    Serial.println(F("  - Verify I2C connections (SDA, SCL)"));
    Serial.println(F("  - Check power to IMU module"));
    while (1) {
      delay(1000);
    }
  }
  
  // Static calibration (gyro/accel zeroing)
  Serial.println(F("\n>>> STEP 1: Static Calibration"));
  Serial.println(F("KEEP VEHICLE PERFECTLY STILL for 15 seconds..."));
  delay(3000);
  IMU_I2C_CalibrationImu();
  
  // Magnetometer calibration (if enabled)
  if (PERFORM_MAG_CALIBRATION) {
    Serial.println(F("\n>>> STEP 2: Magnetometer Calibration"));
    Serial.println(F("========================================"));
    Serial.println(F("PICK UP THE VEHICLE and rotate in a FIGURE-8 pattern"));
    Serial.println(F("in ALL THREE AXES for ~30 seconds"));
    Serial.println(F("========================================"));
    delay(5000);
    
    IMU_I2C_CalibrationMag();
    
    Serial.println(F("\n>>> IMPORTANT: Copy the offset values to your code!"));
    Serial.println(F("   Set PERFORM_MAG_CALIBRATION = false and re-upload"));
  } else {
    Serial.println(F("\n>>> Using pre-loaded magnetometer offsets"));
    Serial.print(F("   X: ")); Serial.println(MAG_OFFSET_X, 4);
    Serial.print(F("   Y: ")); Serial.println(MAG_OFFSET_Y, 4);
    Serial.print(F("   Z: ")); Serial.println(MAG_OFFSET_Z, 4);
  }
  
  // North alignment
  Serial.println(F("\n>>> STEP 3: North Alignment"));
  Serial.println(F("Point vehicle physically NORTH (use phone compass)"));
  delay(5000);
  
  float euler[3];
  int samples = 20;
  float sumYaw = 0;
  
  Serial.println(F("Taking 20 samples to average..."));
  for (int i = 0; i < samples; i++) {
    if (IMU_I2C_ReadEuler(euler) == 0) {
      rawYaw = euler[2];  // Yaw in degrees
      
      // Normalize yaw to [-180, +180] before averaging
      while (rawYaw > 180)   rawYaw -= 360;
      while (rawYaw < -180)  rawYaw += 360;
      
      sumYaw += rawYaw;
      
      if ((i + 1) % 5 == 0) {
        Serial.print(F("  Sample "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(rawYaw, 2);
        Serial.println(F("°"));
      }
    }
    delay(200);
  }
  
  float avgYaw = sumYaw / samples;
  NORTH_OFFSET = -avgYaw;
  Serial.println(F("\n>>> Setting NORTH_OFFSET: "));
  Serial.print(NORTH_OFFSET, 2);
  Serial.println(F("°"));
  
  // Verify offset works
  if (IMU_I2C_ReadEuler(euler) == 0) {
    currentYaw = euler[2] + NORTH_OFFSET;
    while (currentYaw > 180)   currentYaw -= 360;
    while (currentYaw < -180)  currentYaw += 360;
    
    Serial.println(F(">>> Verification: Yaw when facing North = "));
    Serial.print(currentYaw, 2);
    Serial.println(F("° (should be ~0°)"));
  }
  
  Serial.print(F("\nLoaded "));
  Serial.print(NUM_WAYPOINTS);
  Serial.println(F(" waypoints"));
  Serial.print(F("Steering direction: "));
  if (STEERING_DIRECTION == 1) {
    Serial.println(F("NORMAL"));
  } else {
    Serial.println(F("FLIPPED (-1)"));
  }
  Serial.println(F("Waiting for GPS fix..."));
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================

void loop() {
  // Step 1: Feed GPS data to the library
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  
  // Step 2: Check if we have valid GPS data
  if (!gps.location.isValid()) {
    Serial.println(F("Waiting for GPS fix..."));
    delay(1000);
    return;
  }
  
  // Step 3: Check satellite count
  int satelliteCount = gps.satellites.value();
  if (satelliteCount < MIN_SATELLITES) {
    Serial.print(F("Need more satellites. Current: "));
    Serial.println(satelliteCount);
    delay(500);
    return;
  }
  
  // Step 4: Read IMU heading
  float euler[3];
  if (IMU_I2C_ReadEuler(euler) == 0) {
    rawYaw = euler[2] + NORTH_OFFSET;
    
    // Normalize yaw to [-180, +180]
    while (rawYaw > 180)   rawYaw -= 360;
    while (rawYaw < -180)  rawYaw += 360;
    
    currentYaw = rawYaw;
  }
  
  // Step 5: Get current position and target waypoint
  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();
  
  double targetLat = waypoints[currentWaypoint][0];
  double targetLon = waypoints[currentWaypoint][1];
  
  // Step 6: Calculate distance and bearing to current waypoint
  float distanceToWaypoint = gps.distanceBetween(
    currentLat, currentLon, 
    targetLat, targetLon
  );
  
  float bearingToWaypoint = gps.courseTo(
    currentLat, currentLon,
    targetLat, targetLon
  );
  
  // Step 7: Calculate heading error (IMU heading vs target bearing)
  float headingError = bearingToWaypoint - currentYaw;
  
  // Normalize error to range [-180, +180]
  while (headingError > 180)  headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Step 8: Calculate steering using proportional control
  // Apply steering direction multiplier (1 = normal, -1 = flipped)
  int steeringPosition = CENTER_STEERING + (int)(headingError * STEERING_GAIN * STEERING_DIRECTION);
  steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
  
  // Step 9: Apply controls and check if we've arrived
  if (distanceToWaypoint > WAYPOINT_DISTANCE_THRESHOLD) {
    // Not there yet - steer and move forward
    steeringServo.write(steeringPosition);
    escServo.write(ESC_FORWARD);
    
    // Print telemetry every half second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      
      Serial.print(F("Waypoint "));
      Serial.print(currentWaypoint + 1);
      Serial.print(F("/"));
      Serial.print(NUM_WAYPOINTS);
      Serial.print(F(" | Dist: "));
      Serial.print(distanceToWaypoint, 1);
      Serial.print(F("m | Yaw: "));
      Serial.print(currentYaw, 1);
      Serial.print(F("° | Target: "));
      Serial.print(bearingToWaypoint, 1);
      Serial.print(F("° | Error: "));
      Serial.print(headingError, 1);
      Serial.print(F("° | Steer: "));
      Serial.println(steeringPosition);
    }
  } else {
    // Arrived at waypoint!
    Serial.print(F("\n*** WAYPOINT "));
    Serial.print(currentWaypoint + 1);
    Serial.println(F(" REACHED! ***"));
    
    // Stop briefly at waypoint
    escServo.write(ESC_STOP);
    steeringServo.write(CENTER_STEERING);
    delay(2000);
    
    // Move to next waypoint (cycle back to 0 after last one)
    currentWaypoint++;
    if (currentWaypoint >= NUM_WAYPOINTS) {
      currentWaypoint = 0;
      Serial.println(F(">>> Cycling back to Waypoint 1"));
    }
    
    Serial.print(F("Next target: Waypoint "));
    Serial.println(currentWaypoint + 1);
  }
  
  // Small delay to prevent overwhelming the GPS
  delay(50);
}

/*
 * ============================================================================
 * TROUBLESHOOTING GUIDE FOR STUDENTS
 * ============================================================================
 * 
 * Problem: IMU not found
 *   Fix: Check I2C wiring (SDA/SCL), verify IMU power
 *   
 * Problem: Yaw shows crazy values (>360°)
 *   Fix: Run magnetometer calibration again, ensure NORTH_OFFSET is set
 *   
 * Problem: Vehicle circles in place
 *   Fix: Increase STEERING_GAIN (try 0.5, then 0.6)
 *   
 * Problem: Vehicle oscillates left-right wildly
 *   Fix: Decrease STEERING_GAIN (try 0.3, then 0.2)
 *   
 * Problem: Vehicle turns opposite direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 *   
 * Problem: Heading drifts over time
 *   Fix: This is normal for magnetometer - re-calibrate periodically
 * 
 * ============================================================================
 */
