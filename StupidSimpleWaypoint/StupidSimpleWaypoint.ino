/*
 * Stupid Simple Waypoint Navigation
 * ==================================
 * 
 * Cycles through waypoints using GPS position and IMU heading.
 * Based on Example_1 (TinyGPS++) with Yahboom IMU integration.
 * 
 * Hardware:
 *   - Arduino Giga R1 WiFi
 *   - GPS Module connected to Serial3 (9600 baud)
 *   - IMU (Yahboom 9-axis) connected to I2C (Wire, pins 20/21)
 *   - Steering Servo on Pin 7
 *   - ESC on Pin 6
 * 
 * Calibration:
 *   Run IMUCalibration first to get magnetometer offsets, then paste them below.
 * 
 * Runtime Commands (send via Serial Monitor):
 *   q/a - Increase/decrease WAYPOINT_DISTANCE_THRESHOLD
 *   w/s - Increase/decrease STEERING_GAIN
 *   e/d - Increase/decrease HEADING_SMOOTHING
 *   r/f - Increase/decrease northOffset
 */

#include <Servo.h>
#include <TinyGPS++.h>
#include "Wire.h"
#include "imu_i2c_driver.hpp"

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

#define STEERING_SERVO_PIN 7    // Servo that controls wheel direction
#define ESC_PIN 6               // Electronic Speed Controller (throttle)
#define GPS_SERIAL Serial3      // Hardware serial port for GPS

// ============================================================================
// SERVO SETTINGS
// ============================================================================

#define CENTER_STEERING 90
#define MIN_STEERING   70
#define MAX_STEERING  130

// ESC Settings
#define ESC_STOP        90
#define ESC_FORWARD    105

// ============================================================================
// NAVIGATION SETTINGS - Adjustable at runtime
// ============================================================================

#define MIN_SATELLITES  4

float WAYPOINT_DISTANCE_THRESHOLD = 3.0;
float STEERING_GAIN = 0.4;
float HEADING_SMOOTHING = 0.3;
float northOffset = 0.0;

// Adjustment step sizes
#define DISTANCE_STEP   0.5
#define GAIN_STEP       0.05
#define SMOOTH_STEP     0.1
#define NORTH_STEP      5.0

// ============================================================================
// MAGNETOMETER CALIBRATION OFFSETS - Paste values from IMUCalibration here
// ============================================================================
float magOffset[3] = {-3.1251, 0.6836, -4.1505};

// ============================================================================
// WAYPOINT ARRAY
// ============================================================================

const double TARGET_LATITUDE  = 40.34221549544099;
const double TARGET_LONGITUDE = -74.69684288556215;

double waypoints[5][2]={
  {40.34225882621459, -74.69668462475289},
  {40.342599825494446, -74.69651325674255},
  {40.34270813578766, -74.69660530051675},
  {40.34266760030346, -74.69677739146624},
  {40.34234797923362, -74.69693786300246}
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

TinyGPSPlus gps;
Servo steeringServo;
Servo escServo;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float normalizeAngle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float smoothHeading(float newHeading, float &smoothedHeading, float alpha) {
    float diff = newHeading - smoothedHeading;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    smoothedHeading += alpha * diff;
    return normalizeAngle(smoothedHeading);
}

void printStatusLine(double currentLat, double currentLon, 
                     double targetLat, double targetLon,
                     float distanceToWaypoint, float bearingToWaypoint,
                     float smoothedHeading, float headingError,
                     int steeringPosition, int currentWaypoint) {
  Serial.print(F("[GPS] "));
  Serial.print(currentLat, 6);
  Serial.print(F(", "));
  Serial.print(currentLon, 6);
  Serial.print(F(" | [Target] "));
  Serial.print(targetLat, 6);
  Serial.print(F(", "));
  Serial.print(targetLon, 6);
  Serial.print(F(" | [Dist] "));
  Serial.print(distanceToWaypoint, 1);
  Serial.print(F("m | [Bearing] "));
  Serial.print(bearingToWaypoint, 1);
  Serial.print(F(" deg | [Heading] "));
  Serial.print(smoothedHeading, 1);
  Serial.print(F(" deg | [Error] "));
  Serial.print(headingError, 1);
  Serial.print(F(" deg | [Steer] "));
  Serial.print(steeringPosition);
  Serial.print(F(" | [WP] "));
  Serial.print(currentWaypoint + 1);
  Serial.print(F("/5"));
  Serial.print(F(" | [Params] TH="));
  Serial.print(WAYPOINT_DISTANCE_THRESHOLD, 1);
  Serial.print(F(" G="));
  Serial.print(STEERING_GAIN, 2);
  Serial.print(F(" S="));
  Serial.print(HEADING_SMOOTHING, 2);
  Serial.print(F(" N="));
  Serial.println(northOffset, 1);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== Stupid Simple Waypoint Navigation ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize GPS
  GPS_SERIAL.begin(9600);
  
  // Attach servos
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start stopped and centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  // Initialize IMU
  Serial.println(F("[1] Initializing IMU..."));
  Wire.begin();
  delay(100);
  
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
  
  // Calibrate gyro and accelerometer at startup
  Serial.println(F("[2] Calibrating Gyro and Accelerometer..."));
  
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
  
  float gyroOffset[3] = {
    gyroSum[0] / CALIBRATION_SAMPLES,
    gyroSum[1] / CALIBRATION_SAMPLES,
    gyroSum[2] / CALIBRATION_SAMPLES
  };
  
  float accelOffset[3] = {
    accelSum[0] / CALIBRATION_SAMPLES,
    accelSum[1] / CALIBRATION_SAMPLES,
    (accelSum[2] / CALIBRATION_SAMPLES) - 1.0f
  };
  
  Serial.print(F("  Gyro offsets: X=")); Serial.print(gyroOffset[0], 4);
  Serial.print(F(" Y=")); Serial.print(gyroOffset[1], 4);
  Serial.print(F(" Z=")); Serial.println(gyroOffset[2], 4);
  
  Serial.print(F("  Accel offsets: X=")); Serial.print(accelOffset[0], 3);
  Serial.print(F(" Y=")); Serial.print(accelOffset[1], 3);
  Serial.print(F(" Z=")); Serial.println(accelOffset[2], 3);
  
  // Set magnetometer offsets (from calibration)
  IMU_I2C_SetMagOffsets(magOffset);
  
  Serial.println(F("[3] Waiting for GPS fix..."));
  Serial.println(F("Ensure GPS antenna has clear view of sky."));
  Serial.println();
  Serial.println(F("=== Runtime Commands ==="));
  Serial.println(F("q/a - Distance threshold (+/-)"));
  Serial.println(F("w/s - Steering gain (+/-)"));
  Serial.println(F("e/d - Heading smoothing (+/-)"));
  Serial.println(F("r/f - North offset (+/-)"));
  Serial.println();
}

// ============================================================================
// HANDLE SERIAL COMMANDS
// ============================================================================

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'q': // Increase distance threshold
        WAYPOINT_DISTANCE_THRESHOLD += DISTANCE_STEP;
        Serial.print(F("> Distance Threshold: "));
        Serial.println(WAYPOINT_DISTANCE_THRESHOLD, 1);
        break;
      case 'a': // Decrease distance threshold
        WAYPOINT_DISTANCE_THRESHOLD = max(0.5, WAYPOINT_DISTANCE_THRESHOLD - DISTANCE_STEP);
        Serial.print(F("> Distance Threshold: "));
        Serial.println(WAYPOINT_DISTANCE_THRESHOLD, 1);
        break;
        
      case 'w': // Increase steering gain
        STEERING_GAIN += GAIN_STEP;
        Serial.print(F("> Steering Gain: "));
        Serial.println(STEERING_GAIN, 2);
        break;
      case 's': // Decrease steering gain
        STEERING_GAIN = max(0.1, STEERING_GAIN - GAIN_STEP);
        Serial.print(F("> Steering Gain: "));
        Serial.println(STEERING_GAIN, 2);
        break;
        
      case 'e': // Increase heading smoothing
        HEADING_SMOOTHING = min(1.0, HEADING_SMOOTHING + SMOOTH_STEP);
        Serial.print(F("> Heading Smoothing: "));
        Serial.println(HEADING_SMOOTHING, 2);
        break;
      case 'd': // Decrease heading smoothing
        HEADING_SMOOTHING = max(0.05, HEADING_SMOOTHING - SMOOTH_STEP);
        Serial.print(F("> Heading Smoothing: "));
        Serial.println(HEADING_SMOOTHING, 2);
        break;
        
      case 'r': // Increase north offset
        northOffset += NORTH_STEP;
        Serial.print(F("> North Offset: "));
        Serial.println(northOffset, 1);
        break;
      case 'f': // Decrease north offset
        northOffset -= NORTH_STEP;
        Serial.print(F("> North Offset: "));
        Serial.println(northOffset, 1);
        break;
        
      case 'h': // Help
        Serial.println();
        Serial.println(F("=== Commands ==="));
        Serial.println(F("q/a - Distance threshold (+/-)"));
        Serial.println(F("w/s - Steering gain (+/-)"));
        Serial.println(F("e/d - Heading smoothing (+/-)"));
        Serial.println(F("r/f - North offset (+/-)"));
        Serial.println();
        break;
    }
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Handle runtime commands
  handleSerialCommands();
  
  // Feed GPS data to library
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  
  // Check for valid GPS fix
  if (!gps.location.isValid()) {
    Serial.println(F("Waiting for GPS fix..."));
    delay(1000);
    return;
  }
  
  // Check satellite count
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
  Serial.println(5);
  delay(2000);
  
  int currentWaypoint = 0;
  
  while (true) {
    // Handle commands during navigation too
    handleSerialCommands();
    
    // Get current position
    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();
    
    // Get target waypoint
    double targetLat, targetLon;
    
    if (currentWaypoint == 0) {
      targetLat = TARGET_LATITUDE;
      targetLon = TARGET_LONGITUDE;
    } else {
      targetLat = waypoints[currentWaypoint - 1][0];
      targetLon = waypoints[currentWaypoint - 1][1];
    }
    
    // Calculate distance and bearing to waypoint
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
      imuHeading = imuData.euler[2];
      
      // Alternative: Calculate heading from magnetometer with offsets
      /*
      float magX_cal = imuData.mag[0] - magOffset[0];
      float magY_cal = imuData.mag[1] - magOffset[1];
      imuHeading = atan2(magY_cal, magX_cal) * 57.2957795f;
      imuHeading = normalizeAngle(imuHeading + northOffset);
      */
    }
    
    // Smooth the heading
    static float smoothedHeading = 0.0f;
    smoothHeading(imuHeading, smoothedHeading, HEADING_SMOOTHING);
    
    // Calculate heading error
    float headingError = bearingToWaypoint - smoothedHeading;
    
    // Normalize error to [-180, +180]
    while (headingError > 180)  headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    // Calculate steering using proportional control
    int steeringPosition = CENTER_STEERING + (int)(headingError * STEERING_GAIN);
    steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
    
    // Apply controls - always moving forward
    steeringServo.write(steeringPosition);
    escServo.write(ESC_FORWARD);
    
    // Print status line every 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      printStatusLine(currentLat, currentLon, 
                      targetLat, targetLon,
                      distanceToWaypoint, bearingToWaypoint,
                      smoothedHeading, headingError,
                      steeringPosition, currentWaypoint);
    }
    
    // Check if we've reached the waypoint
    if (distanceToWaypoint < WAYPOINT_DISTANCE_THRESHOLD) {
      Serial.print(F("\n*** Waypoint "));
      Serial.print(currentWaypoint + 1);
      Serial.println(F(" REACHED! ***"));
      
      // Move to next waypoint (cycle back to first after last)
      currentWaypoint++;
      if (currentWaypoint >= 5) {
        currentWaypoint = 0;
        Serial.println(F("Cycling back to first waypoint..."));
      }
      
      delay(1000);
    }
    
    delay(50);
  }
}
