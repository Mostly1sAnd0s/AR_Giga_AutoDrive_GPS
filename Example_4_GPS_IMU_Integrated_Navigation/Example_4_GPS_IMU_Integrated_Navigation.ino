/*
 * Example 4: GPS + IMU Integrated Navigation
 * ===========================================
 * 
 * This is the CAPSTONE example that combines all previous examples:
 * - GPS provides absolute position and waypoint navigation
 * - IMU provides high-frequency heading updates between GPS fixes
 * - PID controller steers toward waypoints smoothly
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - GPS Module connected to Serial3
 *   - Yahboom 9-axis IMU connected to I2C (Wire)
 *   - RPLidar A1 connected to Serial2 (optional, for future expansion)
 *   - Steering Servo connected to Pin 7
 *   - ESC connected to Pin 6
 * 
 * How It Works:
 *   GPS gives us accurate position but updates slowly (~5Hz).
 *   IMU gives us fast heading updates (~50Hz) but drifts over time.
 *   We use GPS to set the target heading, then IMU to track it precisely.
 * 
 * Student Notes:
 *   - This is SENSOR FUSION - combining multiple sensors for better results
 *   - Start with P-only control, add I and D only if needed
 *   - Calibration is CRITICAL for good performance!
 */

#include <Servo.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "imu_i2c_driver.hpp"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define STEERING_SERVO_PIN 7      // Servo that controls wheel direction
#define ESC_PIN 6                 // Electronic Speed Controller (throttle)
#define GPS_SERIAL Serial3        // Hardware serial port for GPS

// ============================================================================
// SERVO SETTINGS - ADJUST THESE FOR YOUR VEHICLE
// ============================================================================

#define CENTER_STEERING 90        // Servo PWM value when wheels are straight
#define MIN_STEERING   70         // Minimum servo value (full left turn)
#define MAX_STEERING  130         // Maximum servo value (full right turn)

// ESC Settings  
#define ESC_STOP        90        // PWM value to stop motor
#define ESC_FORWARD    105        // PWM value for forward motion

// ============================================================================
// GPS SETTINGS
// ============================================================================

#define MIN_SATELLITES  5         // Minimum satellites for reliable navigation
#define WAYPOINT_DISTANCE_THRESHOLD  3.0   // Meters to consider "arrived"

// ============================================================================
// IMU SETTINGS - CRITICAL FOR ACCURACY!
// ============================================================================

// Magnetometer calibration offset
// Set this so yaw reads 0° when vehicle points North
// Use the calibration routine below to find this value
#define MAG_CALIBRATION_OFFSET  0.0

// Perform figure-8 calibration on startup? (set true once, then false)
#define PERFORM_MAG_CALIBRATION  false

// Hardcoded magnetometer offsets from calibration
// Fill these in after running calibration once
float hardcodedMagOffsets[3] = {0.0, 0.0, 0.0};

// ============================================================================
// PID CONTROLLER SETTINGS
// ============================================================================

// START HERE: Use P-only control first!
// Once it works reliably, you can experiment with I and D terms

float Kp = 0.8;     // Proportional gain - primary steering control
float Ki = 0.0;     // Integral gain - REMOVE FIRST if vehicle circles
float Kd = 0.1;     // Derivative gain - helps smooth turns

// Steering direction: +1 or -1
// If vehicle turns opposite to expected, flip this sign!
int steeringDirection = 1;

// PID state variables
float lastError = 0;
float integral = 0;

// Control rates
#define IMU_CONTROL_RATE_MS   50    // Update steering every 50ms (20Hz)
#define GPS_UPDATE_RATE_MS   200    // Process GPS every 200ms

// ============================================================================
// WAYPOINTS - DEFINE YOUR NAVIGATION COURSE
// ============================================================================

const double waypoints[][2] = {
  {40.342215, -74.696843},   // Waypoint 1: Starting point / Home
  {40.342259, -74.696685},   // Waypoint 2
  {40.342600, -74.696513},   // Waypoint 3
  {40.342708, -74.696605},   // Waypoint 4
  {40.342668, -74.696777},   // Waypoint 5
  {40.342348, -74.696938}    // Waypoint 6
};

const int waypointCount = 6;
int currentWaypointIndex = 0;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

TinyGPSPlus gps;
Servo steeringServo;
Servo escServo;

// Navigation state
float targetHeading = 0;      // Heading we want to go (from GPS)
float currentYaw = 0;         // Current heading (from IMU)
float distanceToWaypoint = 9999;
bool hasValidGPS = false;

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("  GPS + IMU Integrated Navigation"));
  Serial.println(F("========================================\n"));
  
  // Initialize hardware
  GPS_SERIAL.begin(9600);     // GPS baud rate
  Wire.begin();               // I2C for IMU
  
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start stopped and centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  Serial.println(F("Hardware initialized."));
  
  // =====================================================
  // STEP 1: IMU Connection Check
  // =====================================================
  Serial.println(F("\n[1/4] Checking IMU connection..."));
  
  int imuResult = IMU_I2C_ReadVersion();
  if (imuResult != 0) {
    Serial.println(F("ERROR: IMU not found!"));
    Serial.println(F("Check I2C wiring (SDA, SCL) and power."));
    while (1) { blinkError(); }
  }
  Serial.println(F("IMU connected successfully!"));
  
  // =====================================================
  // STEP 2: Static Calibration (Gyro & Accelerometer)
  // This MUST be done on every startup - don't skip!
  // =====================================================
  Serial.println(F("\n[2/4] Static Calibration (Gyro/Accel)..."));
  Serial.println(F("KEEP VEHICLE PERFECTLY STILL for 15 seconds..."));
  Serial.println(F("This zeros the gyro to prevent drift."));
  
  delay(3000);  // Time to set vehicle down
  
  if (IMU_I2C_CalibrationImu() == 0) {
    Serial.println(F("Static calibration complete!"));
  } else {
    Serial.println(F("WARNING: Static calibration failed!"));
  }
  
  // =====================================================
  // STEP 3: Magnetometer Calibration (One-time setup)
  // Only run this once to get offset values
  // =====================================================
  if (PERFORM_MAG_CALIBRATION) {
    Serial.println(F("\n[3/4] Magnetometer Calibration..."));
    Serial.println(F("1. Pick up the vehicle"));
    Serial.println(F("2. Rotate slowly in FIGURE-8 pattern"));
    Serial.println(F("3. Rotate around ALL THREE AXES"));
    Serial.println(F("4. Continue for ~30 seconds..."));
    
    if (IMU_I2C_CalibrationMag() == 0) {
      float offsets[3];
      IMU_I2C_ReadMagOffsets(offsets);
      
      Serial.println(F("\n========================================"));
      Serial.println(F("COPY THESE VALUES INTO YOUR CODE:"));
      Serial.print(F("float hardcodedMagOffsets[3] = {"));
      Serial.print(offsets[0], 4); Serial.print(F(", "));
      Serial.print(offsets[1], 4); Serial.print(F(", "));
      Serial.print(offsets[2], 4); Serial.println(F("};"));
      Serial.println(F("========================================"));
      
      delay(15000);  // Time to copy values
    } else {
      Serial.println(F("Magnetometer calibration failed!"));
    }
  }
  
  // Apply hardcoded magnetometer offsets
  if (hardcodedMagOffsets[0] != 0 || hardcodedMagOffsets[1] != 0) {
    IMU_I2C_SetMagOffsets(hardcodedMagOffsets);
    Serial.println(F("Magnetometer offsets applied."));
  }
  
  // =====================================================
  // STEP 4: GPS Initialization
  // =====================================================
  Serial.println(F("\n[4/4] Initializing GPS..."));
  Serial.println(F("Waiting for GPS fix (this may take 1-2 minutes)..."));
  Serial.println(F("Ensure antenna has clear view of sky!"));
  
  // Wait for valid GPS fix before proceeding
  unsigned long gpsStartTime = millis();
  while (!gps.location.isValid()) {
    // Feed GPS data
    while (GPS_SERIAL.available()) {
      gps.encode(GPS_SERIAL.read());
    }
    
    // Show progress every 5 seconds
    if (millis() - gpsStartTime > 5000) {
      gpsStartTime = millis();
      Serial.print(F("Satellites: "));
      Serial.print(gps.satellites.value());
      Serial.println(F(" - Keep waiting..."));
    }
    
    delay(100);
    
    // Timeout after 3 minutes
    if (millis() > 180000) {
      Serial.println(F("ERROR: GPS timeout! Check antenna and wiring."));
      while (1) { blinkError(); }
    }
  }
  
  Serial.println(F("GPS fix acquired!"));
  Serial.print(F("Location: "));
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(", "));
  Serial.println(gps.location.lng(), 6);
  
  // =====================================================
  // Setup Complete - Print Configuration
  // =====================================================
  Serial.println(F("\n========================================"));
  Serial.println(F("  SETUP COMPLETE!"));
  Serial.println(F("========================================"));
  Serial.print(F("Waypoints: "));
  Serial.println(waypointCount);
  Serial.print(F("Current WP: #"));
  Serial.println(currentWaypointIndex + 1);
  Serial.print(F("PID Gains: Kp="));
  Serial.print(Kp);
  Serial.print(F(", Ki="));
  Serial.print(Ki);
  Serial.print(F(", Kd="));
  Serial.println(Kd);
  Serial.println(F("\nVehicle will start moving in 3 seconds..."));
  
  delay(3000);
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================

void loop() {
  // =====================================================
  // HIGH-PRIORITY: Feed GPS data (must be frequent!)
  // =====================================================
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }
  
  // =====================================================
  // STEP 1: Update GPS-based navigation (200ms rate)
  // =====================================================
  static unsigned long lastGPSTime = 0;
  if (millis() - lastGPSTime >= GPS_UPDATE_RATE_MS) {
    lastGPSTime = millis();
    
    // Check if GPS data is fresh (< 2 seconds old)
    unsigned long gpsAge = gps.location.age();
    hasValidGPS = (gpsAge < 2000 && gps.satellites.value() >= MIN_SATELLITES);
    
    if (hasValidGPS) {
      // Get current position
      double currentLat = gps.location.lat();
      double currentLon = gps.location.lng();
      
      // Calculate distance and bearing to current waypoint
      distanceToWaypoint = gps.distanceBetween(
        currentLat, currentLon,
        waypoints[currentWaypointIndex][0],
        waypoints[currentWaypointIndex][1]
      );
      
      targetHeading = gps.bearingTo(
        currentLat, currentLon,
        waypoints[currentWaypointIndex][0],
        waypoints[currentWaypointIndex][1]
      );
      
      // Normalize target heading to [-180, +180] to match IMU
      if (targetHeading > 180) targetHeading -= 360;
      if (targetHeading < -180) targetHeading += 360;
      
      // Check if we've reached the waypoint
      if (distanceToWaypoint < WAYPOINT_DISTANCE_THRESHOLD) {
        Serial.println(F("\n*** WAYPOINT REACHED! ***"));
        currentWaypointIndex++;
        
        // Reset integral for new heading (prevents windup)
        integral = 0;
        lastError = 0;
        
        // Cycle back to first waypoint if we've completed the course
        if (currentWaypointIndex >= waypointCount) {
          currentWaypointIndex = 0;
          Serial.println(F("Course complete! Returning to start."));
        }
        
        Serial.print(F("Next waypoint: #"));
        Serial.println(currentWaypointIndex + 1);
      }
    } else {
      // No valid GPS - stop and wait
      distanceToWaypoint = 9999;
      if (gps.satellites.value() < MIN_SATELLITES) {
        static unsigned long lastSatsWarn = 0;
        if (millis() - lastSatsWarn > 2000) {
          lastSatsWarn = millis();
          Serial.print(F("Waiting for satellites: "));
          Serial.println(gps.satellites.value());
        }
      }
    }
  }
  
  // =====================================================
  // STEP 2: IMU-based steering control (50ms rate - 20Hz!)
  // This runs much faster than GPS updates for smooth control
  // =====================================================
  static unsigned long lastIMUTime = 0;
  if (millis() - lastIMUTime >= IMU_CONTROL_RATE_MS) {
    lastIMUTime = millis();
    
    // Read IMU Euler angles
    float euler[3];  // [0]=roll, [1]=pitch, [2]=yaw (radians)
    
    if (IMU_I2C_ReadEuler(euler) != 0) {
      Serial.println(F("ERROR: Failed to read IMU"));
      return;
    }
    
    // Extract yaw and apply calibration offset
    currentYaw = euler[2] + MAG_CALIBRATION_OFFSET;
    
    // Normalize yaw to [-PI, +PI]
    while (currentYaw > PI)   currentYaw -= 2 * PI;
    while (currentYaw < -PI)  currentYaw += 2 * PI;
    
    // Convert to degrees for easier debugging
    float currentYawDeg = currentYaw * 180.0 / PI;
    float targetHeadingDeg = targetHeading;  // Already in degrees
    
    // =====================================================
    // STEP 3: Calculate heading error
    // =====================================================
    float error = targetHeading - currentYawDeg;
    
    // Normalize error to [-180, +180]
    while (error > 180)   error -= 360;
    while (error < -180)  error += 360;
    
    // =====================================================
    // STEP 4: PID Calculation
    // =====================================================
    
    // PROPORTIONAL term: Primary steering control
    float P = error * Kp;
    
    // INTEGRAL term: Eliminates steady-state error (use cautiously!)
    // Clear integral if error is large (>90°) to prevent windup
    if (abs(error) > 90) {
      integral = 0;
    } else {
      integral += error;
      // Anti-windup: Constrain integral contribution
      integral = constrain(integral, -50, 50);
    }
    float I = integral * Ki;
    
    // DERIVATIVE term: Smooths response, reduces overshoot
    float derivative = error - lastError;
    // Normalize derivative to prevent 180° wrap "slamming"
    while (derivative > 180)   derivative -= 360;
    while (derivative < -180)  derivative += 360;
    float D = derivative * Kd;
    
    // Total PID output
    float pidOutput = P + I + D;
    
    // Apply steering direction and convert to servo position
    int steeringOffset = (int)(pidOutput * steeringDirection);
    int steeringPosition = CENTER_STEERING + steeringOffset;
    
    // Constrain to valid servo range
    steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
    
    // Store error for next derivative calculation
    lastError = error;
    
    // =====================================================
    // STEP 5: Apply Controls
    // =====================================================
    steeringServo.write(steeringPosition);
    
    // Only move if we have valid GPS and are not at waypoint
    if (hasValidGPS && distanceToWaypoint < 500) {
      escServo.write(ESC_FORWARD);
    } else {
      escServo.write(ESC_STOP);
    }
    
    // =====================================================
    // STEP 6: Telemetry Output (every 500ms)
    // =====================================================
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      
      Serial.print(F("WP#"));
      Serial.print(currentWaypointIndex + 1);
      Serial.print(F(" | Dist: "));
      Serial.print(distanceToWaypoint, 1);
      Serial.print(F("m | Target: "));
      Serial.print(targetHeadingDeg, 1);
      Serial.print(F("° | Yaw: "));
      Serial.print(currentYawDeg, 1);
      Serial.print(F("° | Error: "));
      Serial.print(error, 1);
      Serial.print(F("° | P="));
      Serial.print(P, 1);
      Serial.print(F(" I="));
      Serial.print(I, 1);
      Serial.print(F(" D="));
      Serial.print(D, 1);
      Serial.print(F(" | Steer: "));
      Serial.println(steeringPosition);
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

// ============================================================================
// HELPER FUNCTION: Error indicator
// ============================================================================

void blinkError() {
  static int count = 0;
  if (count++ >= 5) count = 0;
  delay(200);
}

/*
 * ============================================================================
 * CALIBRATION GUIDE - READ THIS FIRST!
 * ============================================================================
 * 
 * STEP 1: Static Calibration (Automatic on every startup)
 *   - Vehicle must be perfectly still for 15 seconds at power-on
 *   - This zeros the gyro and accelerometer
 *   - No user action needed beyond waiting
 * 
 * STEP 2: Magnetometer Calibration (One-time setup)
 *   1. Set PERFORM_MAG_CALIBRATION = true in the code
 *   2. Upload and open Serial Monitor
 *   3. When prompted, pick up vehicle and rotate in FIGURE-8 pattern
 *   4. Rotate around ALL THREE AXES (not just flat!)
 *   5. Continue for ~30 seconds until calibration completes
 *   6. Copy the printed offset values into hardcodedMagOffsets[]
 *   7. Set PERFORM_MAG_CALIBRATION = false
 *   8. Upload again - calibration is now complete!
 * 
 * STEP 3: North Alignment
 *   1. Point vehicle physically North (use phone compass app)
 *   2. Note the Yaw reading in Serial monitor
 *   3. Set MAG_CALIBRATION_OFFSET = -yaw_reading
 *   4. Now yaw should read ~0° when pointing North
 * 
 * ============================================================================
 * PID TUNING GUIDE
 * ============================================================================
 * 
 * START HERE: P-ONLY CONTROL
 *   - Set Ki = 0, Kd = 0
 *   - Adjust Kp until vehicle reaches waypoints acceptably
 *   - Typical starting value: Kp = 0.5 to 1.0
 *   
 * If vehicle circles around waypoint:
 *   - REDUCE Kp (try 0.3, then 0.2)
 *   - This is usually a steering direction or gain issue
 *   
 * If vehicle oscillates left-right:
 *   - REDUCE Kp slightly
 *   - Add small D term (Kd = 0.05 to 0.1)
 *   
 * If vehicle consistently undershoots turns:
 *   - Increase Kp slightly
 *   - Or add small I term (Ki = 0.001 to 0.005)
 *   
 * If vehicle overshoots and hunts:
 *   - Add D term (Kd = 0.1 to 0.3)
 *   - This dampens the response
 * 
 * ORDER OF TUNING:
 *   1. Set Ki=0, Kd=0, find good Kp
 *   2. If needed, add small Kd to smooth response
 *   3. Only add Ki if there's consistent steady-state error
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: Vehicle drives AWAY from waypoint (circles)
 *   Fix: 
 *     - Check steeringDirection (try flipping sign)
 *     - Reduce Kp significantly (try 0.3)
 *     - Verify MIN/MAX steering values match your servo
 *   
 * Problem: Vehicle jitters at waypoint
 *   Fix:
 *     - Increase WAYPOINT_DISTANCE_THRESHOLD (try 5.0m)
 *     - Add D term to smooth response
 *     - Check GPS satellite count (need 5+)
 *   
 * Problem: Heading drifts over time
 *   Fix:
 *     - Re-run magnetometer calibration
 *     - Check for magnetic interference (motors, metal objects)
 *     - Verify MAG_CALIBRATION_OFFSET is correct
 *   
 * Problem: GPS never gets fix
 *   Fix:
 *     - Ensure antenna has CLEAR view of sky
 *     - Wait 1-2 minutes for initial fix
 *     - Check Serial3 wiring and baud rate
 *     - Try different GPS location (away from buildings)
 * 
 * ============================================================================
 * STUDENT CHALLENGES
 * ============================================================================
 * 
 * 1. Add LIDAR obstacle avoidance to this navigation system
 * 2. Implement adaptive speed (slow down for sharp turns)
 * 3. Add a "return to home" button/feature
 * 4. Log the actual path taken vs. planned waypoints
 * 5. Create a way to upload new waypoints via Serial
 * 
 * ============================================================================
 */
