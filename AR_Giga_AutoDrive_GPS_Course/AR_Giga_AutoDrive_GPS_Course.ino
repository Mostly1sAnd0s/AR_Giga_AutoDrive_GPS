#include <Servo.h>
#include <Wire.h>
#include "RPLidar.h"
#include "imu_i2c_driver.hpp"
#include "TinyGPS.h"

// --- Pin & Hardware Definitions ---
#define STEERING_SERVO_PIN 5
#define ESC_PIN 6
#define LIDAR_MOTOR_PIN 3

#define GPS_SERIAL Serial2
#define LIDAR_SERIAL Serial3
// IMU is on I2C (Wire)

// --- Steering & ESC Constants ---
#define CENTER_STEERING 88
#define MIN_STEERING 48
#define MAX_STEERING 128
#define ESC_STOP 90
#define ESC_FORWARD_SPEED 98

// --- Navigation Config ---
const double waypoints[][2] = {
  {40.34244788, -74.69993579},
  {40.34227594, -74.69981059},
  {40.34211543, -74.69971597},
  {40.34184372, -74.69963018}
};
int waypointCount = 4;
int currentWaypoint = 0;
float distanceThreshold = 2.0; // meters

// --- Sensor Objects ---
RPLidar lidar;
TinyGPS gps;
Servo steer;
Servo esc;

// --- State Variables ---
float currentYaw = 0;
float targetHeading = 0;
float distanceToWaypoint = 999;
float lidarObstacleCorrection = 0;

// LIDAR Sector Mins
float sector_front_min = 9999;
float sector_left_min = 9999;
float sector_right_min = 9999;

void setup() {
  Serial.begin(115200);
  GPS_SERIAL.begin(9600);
  lidar.begin(LIDAR_SERIAL);
  Wire.begin();
  
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  analogWrite(LIDAR_MOTOR_PIN, 255);
  
  steer.attach(STEERING_SERVO_PIN);
  esc.attach(ESC_PIN);
  
  steer.write(CENTER_STEERING);
  esc.write(ESC_STOP);
  
  Serial.println("AR_Giga_AutoDrive_GPS_Course Starting...");
}

void loop() {
  // 1. Process GPS (Background)
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // 2. Process LIDAR (Continuous)
  if (IS_OK(lidar.waitPoint())) {
    float dist = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    if (dist > 0 && dist < 2000) { // Only care about things within 2m
      if (angle > 340 || angle < 20) {
        if (dist < sector_front_min) sector_front_min = dist;
      } else if (angle >= 270 && angle <= 340) {
        if (dist < sector_left_min) sector_left_min = dist;
      } else if (angle >= 20 && angle <= 90) {
        if (dist < sector_right_min) sector_right_min = dist;
      }
    }
  }

  // 3. Sensor Fusion & Control (50Hz)
  static unsigned long lastControlTime = 0;
  if (millis() - lastControlTime > 20) {
    lastControlTime = millis();
    
    // A. Get IMU Yaw
    float euler[3];
    if (IMU_I2C_ReadEuler(euler) == 0) {
      currentYaw = euler[2] * 57.2958; // Convert to degrees
    }
    
    // B. Get GPS Target
    float lat, lon;
    unsigned long age;
    gps.f_get_position(&lat, &lon, &age);
    if (age < 2000) {
      targetHeading = TinyGPS::course_to(lat, lon, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
      distanceToWaypoint = TinyGPS::distance_between(lat, lon, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
      
      if (distanceToWaypoint < distanceThreshold) {
        currentWaypoint++;
        if (currentWaypoint >= waypointCount) {
          esc.write(ESC_STOP);
          Serial.println("Mission Complete!");
          while(1);
        }
      }
    }
    
    // C. Calculate LIDAR Correction
    lidarObstacleCorrection = 0;
    if (sector_front_min < 600) {
       // Emergency avoid
       lidarObstacleCorrection = (sector_left_min > sector_right_min) ? -40 : 40;
    } else if (sector_left_min < 400) {
       lidarObstacleCorrection = 20; // Turn right
    } else if (sector_right_min < 400) {
       lidarObstacleCorrection = -20; // Turn left
    }
    
    // D. Compute Final Steering
    float headingError = targetHeading - currentYaw;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    
    int gpsSteering = CENTER_STEERING + (int)(headingError * 0.8);
    int finalSteering = constrain(gpsSteering + lidarObstacleCorrection, MIN_STEERING, MAX_STEERING);
    
    steer.write(finalSteering);
    
    // E. Speed Control
    if (sector_front_min < 400) {
      esc.write(ESC_STOP);
    } else {
      esc.write(ESC_FORWARD_SPEED);
    }
    
    // Reset LIDAR sectors for next window
    sector_front_min = 9999; sector_left_min = 9999; sector_right_min = 9999;
  }
}
