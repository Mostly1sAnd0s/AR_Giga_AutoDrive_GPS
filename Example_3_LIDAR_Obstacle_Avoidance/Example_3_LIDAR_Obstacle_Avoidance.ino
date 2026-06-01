/*
 * Example 3: LIDAR Obstacle Avoidance
 * ====================================
 * 
 * This example demonstrates reactive obstacle avoidance using RPLidar.
 * The vehicle scans its surroundings and steers away from detected obstacles.
 * 
 * Hardware Required:
 *   - Arduino Giga R1 WiFi
 *   - RPLidar A1 connected to Serial3
 *   - Steering Servo connected to Pin 7
 *   - ESC connected to Pin 6
 *   - LIDAR Motor Control pin (PWM) - use any digital pin
 * 
 * How It Works:
 *   The LIDAR rotates and measures distance at every angle (0-360°).
 *   We divide the scan into sectors (front, left, right) and steer toward
 *   the clearest path.
 * 
 * Student Notes:
 *   - This is REACTIVE control - no memory or planning
 *   - Works great indoors or in cluttered environments
 *   - Can be combined with GPS for waypoint navigation + obstacle avoidance
 *   - LIDAR angle 0° = front of vehicle (check your setup!)
 */

#include <Servo.h>
#include "RPLidar.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define STEERING_SERVO_PIN 7      // Servo that controls wheel direction
#define ESC_PIN 6                 // Electronic Speed Controller (throttle)
#define LIDAR_MOTOR_PIN 3         // Pin to control LIDAR motor (PWM)
#define LIDAR_SERIAL Serial3      // Hardware serial port for LIDAR

// ============================================================================
// SERVO SETTINGS - ADJUST THESE FOR YOUR VEHICLE
// ============================================================================

#define CENTER_STEERING 90        // Servo PWM value when wheels are straight
#define MIN_STEERING   70         // Minimum servo value (full left turn)
#define MAX_STEERING  130         // Maximum servo value (full right turn)

// ESC Settings
#define ESC_STOP        90        // PWM value to stop motor
#define ESC_SLOW       95         // PWM value for slow forward motion
#define ESC_FAST       110        // PWM value for faster forward motion

// ============================================================================
// LIDAR SETTINGS
// ============================================================================

#define OBSTACLE_THRESHOLD_MM  500   // Distance (mm) to consider an obstacle
#define SCAN_UPDATE_MS         100   // How often to process new scan data

// Sector angles (adjust based on your LIDAR orientation)
// RPLidar: 0° = front, angles increase counterclockwise
#define FRONT_SECTOR_MIN      345    // Front sector starts at 345°
#define FRONT_SECTOR_MAX       15    // Front sector ends at 15°
#define LEFT_SECTOR_MIN       270    // Left sector: 270-345°  
#define LEFT_SECTOR_MAX       345
#define RIGHT_SECTOR_MIN        15   // Right sector: 15-90°
#define RIGHT_SECTOR_MAX        90

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RPLidar lidar;            // LIDAR driver object
Servo steeringServo;      // Servo for steering
Servo escServo;           // Servo for ESC (throttle)

// Minimum distance detected in each sector
float minDistanceFront = 9999;
float minDistanceLeft  = 9999;
float minDistanceRight = 9999;

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=== LIDAR Obstacle Avoidance ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize LIDAR motor control pin
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  analogWrite(LIDAR_MOTOR_PIN, 255);  // Full speed
  
  // Attach servos to pins
  steeringServo.attach(STEERING_SERVO_PIN);
  escServo.attach(ESC_PIN);
  
  // Start with vehicle stopped and wheels centered
  steeringServo.write(CENTER_STEERING);
  escServo.write(ESC_STOP);
  
  // Initialize LIDAR serial communication
  Serial.println(F("Connecting to RPLidar..."));
  lidar.begin(LIDAR_SERIAL);
  
  // Try to get device info to verify connection
  rplidar_response_device_info_tDeviceInfo;
  delay(100);
  
  if (IS_OK(lidar.getDeviceInfo(deviceInfo, 100))) {
    Serial.println(F("RPLidar connected!"));
    Serial.print(F("Model: "));
    Serial.println(deviceInfo.model);
    
    // Start scanning
    lidar.startScan();
    Serial.println(F("Scanning started..."));
  } else {
    Serial.println(F("ERROR: Could not connect to RPLidar!"));
    Serial.println(F("Check wiring and power."));
    while (1) {
      delay(500);
      Serial.println(F("Retrying..."));
      if (IS_OK(lidar.getDeviceInfo(deviceInfo, 100))) break;
    }
  }
  
  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Vehicle will avoid obstacles automatically."));
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================

void loop() {
  // =====================================================
  // STEP 1: Read LIDAR data point
  // =====================================================
  // waitPoint() blocks until a new measurement arrives
  if (IS_OK(lidar.waitPoint())) {
    RPLidarMeasurement measurement = lidar.getCurrentPoint();
    
    float distance = measurement.distance;  // Distance in mm
    float angle = measurement.angle;        // Angle in degrees [0, 360)
    
    // Filter out invalid readings (distance = 0 means no detection)
    if (distance > 0 && distance < 10000) {
      
      // =====================================================
      // STEP 2: Categorize into sectors
      // =====================================================
      
      // Front sector: crosses 0° boundary (345-360 and 0-15)
      if (angle >= FRONT_SECTOR_MIN || angle <= FRONT_SECTOR_MAX) {
        if (distance < minDistanceFront) {
          minDistanceFront = distance;
        }
      }
      // Left sector: 270-345°
      else if (angle >= LEFT_SECTOR_MIN && angle <= LEFT_SECTOR_MAX) {
        if (distance < minDistanceLeft) {
          minDistanceLeft = distance;
        }
      }
      // Right sector: 15-90°
      else if (angle >= RIGHT_SECTOR_MIN && angle <= RIGHT_SECTOR_MAX) {
        if (distance < minDistanceRight) {
          minDistanceRight = distance;
        }
      }
    }
  } else {
    // LIDAR not returning data - try to recover
    Serial.println(F("LIDAR timeout - restarting scan..."));
    lidar.stop();
    delay(500);
    lidar.startScan();
  }
  
  // =====================================================
  // STEP 3: Make steering decision (at controlled rate)
  // =====================================================
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= SCAN_UPDATE_MS) {
    lastUpdate = millis();
    
    int targetSteering = CENTER_STEERING;
    float speed = ESC_SLOW;
    
    // Debug output every second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
      lastDebug = millis();
      Serial.print(F("Front: "));
      Serial.print(minDistanceFront);
      Serial.print(F("mm | Left: "));
      Serial.print(minDistanceLeft);
      Serial.print(F("mm | Right: "));
      Serial.print(minDistanceRight);
      Serial.print(F("mm | Steer: "));
      Serial.println(targetSteering);
    }
    
    // =====================================================
    // STEP 4: Obstacle avoidance logic
    // =====================================================
    
    if (minDistanceFront < OBSTACLE_THRESHOLD_MM) {
      // OBSTACLE DIRECTLY AHEAD - must turn!
      Serial.println(F(">>> OBSTACLE DETECTED!"));
      
      if (minDistanceLeft > minDistanceRight) {
        // Left is clearer - turn left
        targetSteering = MIN_STEERING;
        speed = ESC_STOP;  // Stop while turning
      } else {
        // Right is clearer - turn right
        targetSteering = MAX_STEERING;
        speed = ESC_STOP;  // Stop while turning
      }
    }
    else if (minDistanceLeft < OBSTACLE_THRESHOLD_MM * 0.6) {
      // Obstacle on left - veer right
      targetSteering = CENTER_STEERING + 20;  // Gentle right turn
      speed = ESC_SLOW;
    }
    else if (minDistanceRight < OBSTACLE_THRESHOLD_MM * 0.6) {
      // Obstacle on right - veer left
      targetSteering = CENTER_STEERING - 20;  // Gentle left turn
      speed = ESC_SLOW;
    }
    else {
      // Clear path ahead - go straight
      targetSteering = CENTER_STEERING;
      speed = ESC_FAST;  // Go faster when clear
    }
    
    // =====================================================
    // STEP 5: Apply steering command
    // =====================================================
    targetSteering = constrain(targetSteering, MIN_STEERING, MAX_STEERING);
    steeringServo.write(targetSteering);
    escServo.write(speed);
    
    // Reset sector minimums for next scan window
    minDistanceFront = 9999;
    minDistanceLeft  = 9999;
    minDistanceRight = 9999;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

/*
 * ============================================================================
 * TROUBLESHOOTING GUIDE FOR STUDENTS
 * ============================================================================
 * 
 * Problem: LIDAR not connecting
 *   Fix:
 *     - Verify Serial3 baud rate (RPLidar A1 uses 115200)
 *     - Check power supply (LIDAR needs ~5V, can draw 500mA+)
 *     - Ensure TX/RX wiring is correct (cross them!)
 *   
 * Problem: LIDAR motor not spinning
 *   Fix:
 *     - Check LIDAR_MOTOR_PIN connection
 *     - Verify analogWrite() is sending signal
 *     - Some LIDARs auto-start when powered
 *   
 * Problem: Vehicle turns wrong direction
 *   Fix: Swap MIN_STEERING and MAX_STEERING values
 *   
 * Problem: Vehicle doesn't avoid obstacles
 *   Fix:
 *     - Increase OBSTACLE_THRESHOLD_MM (try 800 or 1000)
 *     - Check LIDAR is at appropriate height (not blocked)
 *     - Verify sector angles match your mounting orientation
 *   
 * Problem: Erratic steering
 *   Fix:
 *     - Increase SCAN_UPDATE_MS for more stable decisions
 *     - Add averaging/smoothing to distance readings
 *     - Lower ESC speed for more controlled turns
 * 
 * ============================================================================
 * ADVANCED IDEAS FOR STUDENTS
 * ============================================================================
 * 
 * 1. Add more sectors (front-left, front-right, etc.) for finer control
 * 2. Implement "wall following" behavior using LIDAR data
 * 3. Combine with GPS: Navigate to waypoint, avoid obstacles when detected
 * 4. Add state machine: SEARCH -> AVOID -> RECOVER -> SEARCH
 * 5. Log obstacle positions for mapping
 * 
 * ============================================================================
 */
