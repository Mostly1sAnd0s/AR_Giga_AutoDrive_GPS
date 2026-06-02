/*
 * Example 2a: Multiple Waypoint Navigation with IMU Stabilization
 * ================================================================
 * 
 * STEERING DIRECTION FIX:
 * If vehicle drives AWAY from waypoint instead of toward it:
 *   - Change STEERING_DIRECTION from +1 to -1 (line ~53)
 * 
 * GPS NOT UPDATING?
 *   - Check GPS baud rate (default 9600)
 *   - Ensure antenna has clear sky view
 *   - Verify Serial3 wiring
 * 
 * This example demonstrates continuous waypoint navigation:
 * - Calibrates IMU gyro and accelerometer at startup
 * - Waits for GPS fix with required satellites
 * - Navigates through multiple waypoints in sequence
 * - Cycles back to first waypoint after reaching the last
 * - Uses IMU yaw for smooth heading stabilization
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - GPS Module (NEMA compatible) connected to Serial3
 *   - IMU (Yahboom 9-axis) connected to I2C (Wire)
 *   - Steering Servo connected to Pin 7
 *   - ESC (Electronic Speed Controller) connected to Pin 6
 * 
 * Student Notes:
 *   - Edit the waypoint array below to set your route
 *   - Vehicle will automatically cycle through waypoints continuously
 *   - IMU provides smooth heading during GPS updates
 */

#include <Servo.h>
#include <TinyGPSPlus.h>
#include "Wire.h"
#include "imu_i2c_driver.hpp"

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

// ============================================================================
// NAVIGATION SETTINGS
// ============================================================================

#define MIN_SATELLITES  4       // Minimum satellites needed for valid GPS fix
#define WAYPOINT_DISTANCE_THRESHOLD 3.0   // Meters - distance to consider "arrived"

// Steering gain: Higher = more aggressive turning
// Start with 0.3-0.5 and adjust based on vehicle behavior
#define STEERING_GAIN    0.4

// Steering direction: +1 for normal, -1 if vehicle turns opposite direction
// If vehicle drives AWAY from waypoint, change this to -1
#define STEERING_DIRECTION  1    

// IMU heading smoothing (0.1 = smooth but slow, 0.5 = responsive)
#define HEADING_SMOOTHING 0.3

// ============================================================================
// MAGNETOMETER CALIBRATION OFFSETS - ENTER YOUR CALIBRATED VALUES HERE
// ============================================================================
// Run IMUCalibration first to get these values, then paste them below.
// If using IMU's native Euler yaw (default), these are optional but recommended.
float magOffset[3] = {-3.1251, 0.6836, -4.1505};
float northOffset = 0.0;                 // North alignment offset in degrees

// ============================================================================
// WAYPOINT ARRAY - EDIT THESE COORDINATES
// ============================================================================

// Add or remove waypoints as needed. Vehicle will cycle through them continuously.
// Get coordinates from Google Maps (right-click on a location)
struct Waypoint {
    double latitude;
    double longitude;
};

const Waypoint waypoints[] = {
  {40.34221549544099, -74.69684288556215}, //Home
  {40.34225882621459, -74.69668462475289},
  {40.342599825494446, -74.69651325674255},
  {40.34270813578766, -74.69660530051675},
  {40.34266760030346, -74.69677739146624},
  {40.34234797923362, -74.69693786300246}
};

const int NUM_WAYPOINTS = sizeof(waypoints) / sizeof(waypoints[0]);

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

TinyGPSPlus gps;      // GPS parsing library
Servo steeringServo;  // Servo for steering
Servo escServo;       // Servo for ESC (throttle)

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/** Normalize angle to [-180°, +180°] */
float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/** Angle-aware smoothing filter - handles ±180° wrap-around */
float smoothHeading(float newHeading, float &smoothedHeading, float alpha) {
    float diff = newHeading - smoothedHeading;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    smoothedHeading += alpha * diff;
    return normalizeAngle(smoothedHeading);
}

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println(F("\n=== Multiple Waypoint Navigation with IMU ==="));
    Serial.println(F("Initializing..."));
    
    // Initialize GPS serial port
    GPS_SERIAL.begin(9600);  // Most GPS modules use 9600 baud
    
    // Attach servos to pins
    steeringServo.attach(STEERING_SERVO_PIN);
    escServo.attach(ESC_PIN);
    
    // Start with vehicle stopped and wheels centered
    steeringServo.write(CENTER_STEERING);
    escServo.write(ESC_STOP);
    
    Serial.println(F("[1] Initializing IMU..."));
    Wire.begin();
    delay(100);
    
    // Check IMU connection
    int result = IMU_I2C_ReadVersion();
    if (result != 0) {
        Serial.println(F("ERROR: IMU not found! Check I2C wiring."));
        while (1) { delay(1000); }
    }
    
    // Initialize IMU for data streaming
    result = IMU_I2C_SendCommand(IMU_FUNC_REQUEST_DATA, 0);
    if (result != 0) {
        Serial.println(F("ERROR: Failed to initialize IMU"));
        while (1) { delay(1000); }
    }
    
    // Wait for IMU to stabilize
    delay(500);
    Serial.println(F("[2] Calibrating Gyro and Accelerometer..."));
    
    // Collect samples for calibration
    float gyroSum[3] = {0, 0, 0};
    float accelSum[3] = {0, 0, 0};
    const int CALIBRATION_SAMPLES = 50;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        imu_measurement_t data;
        if (IMU_I2C_ReadAll(&data) == 0) {
            gyroSum[0] += data.gyro[0];
            gyroSum[1] += data.gyro[1];
            gyroSum[2] += data.gyro[2];
            accelSum[0] += data.accel[0];
            accelSum[1] += data.accel[1];
            accelSum[2] += data.accel[2];
        }
        delay(20);
    }
    
    // Calculate calibration offsets (average values)
    float gyroOffset[3] = {
        gyroSum[0] / CALIBRATION_SAMPLES,
        gyroSum[1] / CALIBRATION_SAMPLES,
        gyroSum[2] / CALIBRATION_SAMPLES
    };
    
    float accelOffset[3] = {
        (accelSum[0] / CALIBRATION_SAMPLES),
        (accelSum[1] / CALIBRATION_SAMPLES),
        (accelSum[2] / CALIBRATION_SAMPLES) - 1.0f  // Subtract 1g for Z-axis
    };
    
    Serial.print(F("  Gyro offsets: X=")); Serial.print(gyroOffset[0], 4);
    Serial.print(F(" Y=")); Serial.print(gyroOffset[1], 4);
    Serial.print(F(" Z=")); Serial.println(gyroOffset[2], 4);
    
    Serial.print(F("  Accel offsets: X=")); Serial.print(accelOffset[0], 3);
    Serial.print(F(" Y=")); Serial.print(accelOffset[1], 3);
    Serial.print(F(" Z=")); Serial.println(accelOffset[2], 3);
    
    Serial.println(F("[3] Waiting for GPS fix..."));
    Serial.println(F("Please ensure GPS antenna has clear view of sky."));
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
    
    // GPS fix acquired - start navigation
    Serial.println(F("\n*** GPS FIX ACQUIRED - STARTING NAVIGATION ***"));
    Serial.print(F("Waypoints in route: "));
    Serial.println(NUM_WAYPOINTS);
    delay(2000);
    
    int currentWaypoint = 0;  // Start at first waypoint
    
    while (true) {
        // CRITICAL: Keep parsing GPS data continuously
        int charsProcessed = 0;
        while (GPS_SERIAL.available() > 0) {
            if (gps.encode(GPS_SERIAL.read())) {
                charsProcessed++;
            }
        }
        
        // Check if GPS fix is still valid
        if (!gps.location.isValid()) {
            Serial.println(F("WARNING: GPS fix lost! Waiting for reacquisition..."));
            escServo.write(ESC_STOP);
            steeringServo.write(CENTER_STEERING);
            delay(2000);
            continue;  // Restart the waypoint loop
        }
        
        // Get current position
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();
        
        // Get target waypoint
        double targetLat = waypoints[currentWaypoint].latitude;
        double targetLon = waypoints[currentWaypoint].longitude;
        
        // Calculate distance and bearing to current waypoint
        float distanceToWaypoint = gps.distanceBetween(
            currentLat, currentLon, 
            targetLat, targetLon
        );
        
        float bearingToWaypoint = gps.courseTo(
            currentLat, currentLon,
            targetLat, targetLon
        );
        
        // Get IMU yaw for heading stabilization
        imu_measurement_t imuData;
        float imuHeading = 0.0f;
        
        if (IMU_I2C_ReadAll(&imuData) == 0) {
            // Option 1: Use IMU's native Euler yaw (already fused, smooth)
            imuHeading = imuData.euler[2];
            
            // Option 2: Calculate heading from magnetometer with your offsets
            // Uncomment below to use manual mag calibration instead:
            /*
            float magX_cal = imuData.mag[0] - magOffset[0];
            float magY_cal = imuData.mag[1] - magOffset[1];
            imuHeading = atan2(magY_cal, magX_cal) * 57.2957795f;
            imuHeading = normalizeAngle(imuHeading + northOffset);
            */
        }
        
        // Smooth the heading to reduce noise
        static float smoothedHeading = 0.0f;
        smoothHeading(imuHeading, smoothedHeading, HEADING_SMOOTHING);
        
        // Calculate heading error: where we want to go - where we're facing
        float headingError = bearingToWaypoint - smoothedHeading;
        
        // Normalize error to range [-180, +180]
        while (headingError > 180)  headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // Calculate steering using proportional control
        // Apply steering direction multiplier to fix opposite-turn issue
        int steeringPosition = CENTER_STEERING + (int)(headingError * STEERING_GAIN * STEERING_DIRECTION);
        steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
        
        // Apply controls - always moving forward (no stopping at waypoints)
        steeringServo.write(steeringPosition);
        escServo.write(ESC_FORWARD);
        
        // Print telemetry every half second
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {
            lastPrint = millis();
            Serial.print(F("WP "));
            Serial.print(currentWaypoint + 1);
            Serial.print(F("/"));
            Serial.print(NUM_WAYPOINTS);
            Serial.print(F(" | Sat: "));
            Serial.print(gps.satellites.value());
            Serial.print(F(" | Dist: "));
            Serial.print(distanceToWaypoint, 1);
            Serial.print(F("m | Bearing: "));
            Serial.print(bearingToWaypoint, 1);
            Serial.print(F("° | Heading: "));
            Serial.print(smoothedHeading, 1);
            Serial.print(F("° | Error: "));
            Serial.print(headingError, 1);
            Serial.print(F("° | Steer: "));
            Serial.print(steeringPosition);
            Serial.print(F(" | GPS Chars: "));
            Serial.println(charsProcessed);
            Serial.print(F("  Lat/Lon: "));
            Serial.print(currentLat, 6);
            Serial.print(F(", "));
            Serial.println(currentLon, 6);
        }
        
        // Check if we've reached the current waypoint
        if (distanceToWaypoint < WAYPOINT_DISTANCE_THRESHOLD) {
            Serial.print(F("\n*** Waypoint "));
            Serial.print(currentWaypoint + 1);
            Serial.println(F(" REACHED! ***"));
            
            // Move to next waypoint (cycle back to first after last)
            currentWaypoint++;
            if (currentWaypoint >= NUM_WAYPOINTS) {
                currentWaypoint = 0;
                Serial.println(F("Cycling back to first waypoint..."));
            }
            
            delay(1000);  // Brief pause before continuing
        }
        
        // Small delay to prevent overwhelming the GPS
        delay(50);
    }
}

/*
 * ============================================================================
 * HOW TO MODIFY THIS CODE
 * ============================================================================
 * 
 * 1. Change Waypoints:
 *    Edit the waypoints[] array in the WAYPOINT ARRAY section.
 *    Get coordinates from Google Maps (right-click on location).
 *    
 * 2. Adjust Speed:
 *    - ESC_FORWARD: Higher = faster (try 100-110)
 *    - Lower values = slower movement
 *    
 * 3. Tune Steering:
 *    - STEERING_GAIN: 0.3 (gentle) to 0.6 (aggressive)
 *    - MIN_STEERING / MAX_STEERING: Adjust turn radius
 *    
 * 4. Change Waypoint Distance:
 *    - WAYPOINT_DISTANCE_THRESHOLD: Meters to trigger "arrival"
 *    - Smaller = more precise, larger = earlier transition
 *    
 * 5. GPS Satellite Requirement:
 *    - MIN_SATELLITES: 4 (basic) to 6+ (more accurate)
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: Vehicle circles in place
 *   Fix: Increase STEERING_GAIN (try 0.5)
 *   
 * Problem: Vehicle oscillates left-right
 *   Fix: Decrease STEERING_GAIN (try 0.3)
 *   
 * Problem: Vehicle turns wrong direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 *   
 * Problem: Never gets GPS fix
 *   Fix: Ensure antenna has clear sky view, wait 1-2 minutes
 *   
 * Problem: IMU not found
 *   Fix: Check I2C wiring (SDA/SCL), verify power to IMU
 * 
 * ============================================================================
 */
