/*
 * Example 1: GPS Basic Waypoint Navigation
 * =========================================
 * 
 * This example demonstrates the simplest form of GPS navigation:
 * - Read current position from GPS
 * - Calculate heading to a target waypoint
 * - Steer toward the waypoint using proportional control
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - GPS Module (NEMA compatible) connected to Serial3
 *   - Steering Servo connected to Pin 7
 *   - ESC (Electronic Speed Controller) connected to Pin 6
 * 
 * How It Works:
 *   The GPS provides latitude/longitude. We calculate the bearing (heading)
 *   from current position to the waypoint using TinyGPS++. The steering servo
 *   is adjusted proportionally to the heading error.
 * 
 * Student Notes:
 *   - This is a P-ONLY controller (no I or D terms)
 *   - Works best on open terrain with good GPS signal (4+ satellites)
 *   - No obstacle avoidance - will drive straight into obstacles!
 */

#include <Servo.h>
#include <TinyGPS++.h>

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

#define MIN_SATELLITES  4       // Minimum satellites needed for valid fix
#define WAYPOINT_DISTANCE_THRESHOLD 3.0   // Meters - distance to consider "arrived"

// Steering gain: Higher = more aggressive turning
// Start with 0.3-0.5 and adjust based on vehicle behavior
#define STEERING_GAIN    0.4    

// ============================================================================
// TARGET WAYPOINT
// ============================================================================

// Replace these coordinates with your desired waypoint
// You can get coordinates from Google Maps (right-click on a location)
const double TARGET_LATITUDE  = 40.342258;   // Example: near starting point
const double TARGET_LONGITUDE = -74.696684;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

TinyGPSPlus gps;      // GPS parsing library
Servo steeringServo;  // Servo for steering
Servo escServo;       // Servo for ESC (throttle)

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== GPS Basic Waypoint Navigation ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize GPS serial port
  GPS_SERIAL.begin(9600);  // Most GPS modules use 9600 baud
  
  // Attach servos to pins
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start with vehicle stopped and wheels centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  Serial.println(F("Hardware initialized. Waiting for GPS fix..."));
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================

void loop() {
  // Step 1: Feed GPS data to the library
  // This parses incoming NMEA sentences from the GPS module
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  
  // Step 2: Check if we have valid GPS data
  if (!gps.location.isValid()) {
    Serial.println(F("Waiting for GPS fix..."));
    delay(1000);
    return;  // Skip this loop iteration
  }
  
  // Step 3: Check satellite count (more satellites = better accuracy)
  int satelliteCount = gps.satellites.value();
  if (satelliteCount < MIN_SATELLITES) {
    Serial.print(F("Need more satellites. Current: "));
    Serial.println(satelliteCount);
    delay(500);
    return;
  }
  
  // Step 4: Get current position
  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();
  
  // Step 5: Calculate distance and bearing to waypoint
  float distanceToWaypoint = gps.distanceBetween(
    currentLat, currentLon, 
    TARGET_LATITUDE, TARGET_LONGITUDE
  );
  
  float bearingToWaypoint = gps.bearingTo(
    currentLat, currentLon,
    TARGET_LATITUDE, TARGET_LONGITUDE
  );
  
  // Step 6: Get current GPS course (direction of travel)
  float currentCourse = gps.course.value();
  
  // Step 7: Calculate heading error
  // Error = where we want to go - where we're going
  float headingError = bearingToWaypoint - currentCourse;
  
  // Normalize error to range [-180, +180]
  // This handles the wrap-around at 360 degrees
  while (headingError > 180)  headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Step 8: Calculate steering using proportional control
  // Steering = center + (error * gain)
  int steeringPosition = CENTER_STEERING + (int)(headingError * STEERING_GAIN);
  
  // Constrain to valid servo range
  steeringPosition = constrain(steeringPosition, MIN_STEERING, MAX_STEERING);
  
  // Step 9: Apply controls
  if (distanceToWaypoint > WAYPOINT_DISTANCE_THRESHOLD) {
    // Not there yet - steer and move forward
    steeringServo.write(steeringPosition);
    escServo.write(ESC_FORWARD);
    
    // Print telemetry every half second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("Dist: "));
      Serial.print(distanceToWaypoint, 1);
      Serial.print(F("m | Bearing: "));
      Serial.print(bearingToWaypoint, 1);
      Serial.print(F("° | Course: "));
      Serial.print(currentCourse, 1);
      Serial.print(F("° | Error: "));
      Serial.print(headingError, 1);
      Serial.print(F("° | Steer: "));
      Serial.println(steeringPosition);
    }
  } else {
    // Arrived at waypoint!
    Serial.println(F("\n*** WAYPOINT REACHED! ***"));
    escServo.write(ESC_STOP);
    steeringServo.write(CENTER_STEERING);
    
    // Wait here (or you could add code to go to next waypoint)
    delay(5000);
  }
  
  // Small delay to prevent overwhelming the GPS
  delay(50);
}

/*
 * ============================================================================
 * TROUBLESHOOTING GUIDE FOR STUDENTS
 * ============================================================================
 * 
 * Problem: Vehicle circles in place
 *   Fix: Increase STEERING_GAIN (try 0.5, then 0.6)
 *   
 * Problem: Vehicle oscillates left-right wildly
 *   Fix: Decrease STEERING_GAIN (try 0.3, then 0.2)
 *   
 * Problem: Vehicle drifts away from waypoint
 *   Fix: Check that MIN_SATELLITES is set correctly (try 4 or 5)
 *   
 * Problem: No GPS fix ever
 *   Fix: 
 *     - Ensure GPS antenna has clear view of sky
 *     - Verify Serial3 baud rate matches your GPS module
 *     - Check wiring connections
 *   
 * Problem: Vehicle turns opposite direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 * 
 * ============================================================================
 */
