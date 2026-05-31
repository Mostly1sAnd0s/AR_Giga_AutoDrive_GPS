#include "NemaGPS.h"
#include <QMC5883L.h>
#include <Servo.h>
#include <Delta2D.h>
#include "SSD1306_Text.h"

#define sda_pin 20
#define scl_pin 21

// Create an instance of the HMC5883L class
QMC5883L compass;
NemaGPS gps;
SSD1306_Text display1(&Wire1, 0x3C, 128, 64);   // Primary I2C

#define MAX_STEERING_POS 128
#define MIN_STEERING_POS 48
#define CENTER_STEERING_POS 88

const float TURN_GAIN = 0.50;
int steering = 88;  // Initialize to center
int steeringUpdate;

Servo steer;
int servoTime = 0;
int servoRate = 20;

// Define a waypoint (e.g., Eiffel Tower coordinates)
// You can change these to any desired latitude and longitude
const double WAYPOINT_LATITUDE = 40.34268936844941;
const double WAYPOINT_LONGITUDE = -74.7001053570769;

float waypoints[12][2]={
  {40.34244788207505,-74.69993579320852},
  {40.34227594354479,-74.69981059217406},
  {40.34211543874844,-74.69971597243},
  {40.34184372088285,-74.69963018965232},
  {40.34161618346126,-74.69961315491135},
  {40.34141418930037,-74.69965379142566},
  {40.34123505813008,-74.69969580047662},
  {40.34115592993258,-74.69970076675425},
  {40.341064515326764,-74.69966037121965},
  {40.34092488959907,-74.69958216282437},
  {40.34078672500925,-74.69946471140618},
  {40.34069957040727,-74.69935050797547}
};
int waypointCounter = 0;
float distance = 99999.99;
float distanceThreshold = 0.5;

float currentHeading = 0.00;
unsigned long compassTime =  0;
int compassRate = 100;
float compassNoiseThreshold = 100.0;
float compassAlpha = 0.8;

int i2cWDTimer = 0;
int i2cWDTRate = 1000;

Delta2D lidar;
float lidarQ1 = 0.0;
float lidarQ2 = 0.0;
float lidarQ3 = 0.0;
float lidarQ4 = 0.0;

int q1min = 305;
int q1max = 315;
int q2min = 340;
int q2max = 355;
int q3min = 5;
int q3max = 20;
int q4min = 45;
int q4max = 55;
int lidarMax = 1000;
int lidarMin = 10;
float q14Gain = 500.0;
float q23Gain = 500.0;
float lidarGain = 20.0;
float lidarAlpha = 0.2;

void setup() {
  steer.attach(50);
  steer.write(steering);
  // Start serial for debugging output
  Serial.begin(115200);
  Serial2.begin(115200); //GPS
  Serial3.begin(115200); //Lidar
  lidar.begin(Serial3);
  lidar.setZeroOffset(-15 + 90);
 
  if (compass.init()) {
    Serial.println("QMC5883L initialized successfully!");
  } else {
    Serial.println("Failed to initialize QMC5883L");
    i2cRecovery();
  }

  // Optional: Set magnetic declination for your location
  compass.setMagneticDeclination(-12.5);
  // Optional: Run calibration
  // compass.calibrate();
  // Optional: Set custom calibration values
  compass.setCalibration(-1650, 1770, -1008, 2051, -1751, 1691);

  // Start the serial communication with the GPS module
  gps.begin(Serial2, 115200);

  // Set the waypoint
  gps.setWaypoint(WAYPOINT_LATITUDE, WAYPOINT_LONGITUDE);
  Serial.print("Waypoint set to: ");
  Serial.print(WAYPOINT_LATITUDE, 6);
  Serial.print(", ");
  Serial.println(WAYPOINT_LONGITUDE, 6);
  Serial.println("----------------------");

  Wire1.begin();
  display1.begin();
  display1.clear();
  
  compassTime = millis();
  i2cWDTimer = millis();
  currentHeading = compass.getAzimuth();
}

void loop() { // run over and over
  if(i2cWDTimer + i2cWDTRate < millis()){
    //ESP.reset();
    i2cRecovery();
  }
  
  checkCompass();
  display1.setCursor(0,0);
  display1.print("Heading: ");
  display1.print(String(currentHeading));
  checkLidar();
  
  if (gps.update()) {
    if (gps.isValid()) {
      Serial.print("Latitude: ");
      Serial.print(gps.getLatitude(), 6);
      Serial.print(" ");
      Serial.println(gps.getLatitudeHemisphere());

      Serial.print("Longitude: ");
      Serial.print(gps.getLongitude(), 6);
      Serial.print(" ");
      Serial.println(gps.getLongitudeHemisphere());

      Serial.print("Altitude: ");
      Serial.print(gps.getAltitude());
      Serial.println(" meters");

      Serial.print("Speed: ");
      Serial.print(gps.getSpeedKmh());
      Serial.println(" km/h");
      //float currentHeading = compass.getHeading();
      //float currentHeading = gps.getCourse();
      Serial.print("Course: ");
      Serial.print(currentHeading);
      Serial.println(" degrees");

      Serial.print("Satellites: ");
      Serial.println(gps.getSatellitesUsed());

      Serial.print("Date: ");
      Serial.print(gps.getDay());
      Serial.print("/");
      Serial.print(gps.getMonth());
      Serial.print("/");
      Serial.println(gps.getYear());

      Serial.print("Time: ");
      Serial.print(gps.getHour());
      Serial.print(":");
      Serial.print(gps.getMinute());
      Serial.print(":");
      Serial.println(gps.getSecond());

      // Waypoint Navigation Output
      float targetHeading = gps.getWaypointHeading();      
      Serial.print("Waypoint Heading: ");
      Serial.print(targetHeading, 2);
      Serial.println(" degrees");
      distance = gps.getWaypointDistance();
      Serial.print("Waypoint Distance: ");
      Serial.print(distance, 2);
      Serial.println(" meters");

      if(distance <= distanceThreshold){
        gps.setWaypoint(waypoints[waypointCounter][0], waypoints[waypointCounter][1]);
        waypointCounter++;
        if(waypointCounter >= 12){
          steer.write(CENTER_STEERING_POS);
          while(1);
        }
      }
      
      //Robot Control Output
      updateSteering(currentHeading, targetHeading);
      Serial.print("Compass steering: ");
      Serial.print(steering);
      steeringUpdate = steering;
      steeringUpdate += calculateSteeringCorrection(lidarQ1, lidarQ2, lidarQ3, lidarQ4, q14Gain, q23Gain, q23Gain, q14Gain)*lidarGain;
      steeringUpdate = constrain(steeringUpdate, MIN_STEERING_POS, MAX_STEERING_POS); 
      Serial.print(" Lidar steering: ");
      Serial.print(steeringUpdate);
      Serial.print(" Correction: ");
      Serial.println(calculateSteeringCorrection(lidarQ1, lidarQ2, lidarQ3, lidarQ4, q14Gain, q23Gain, q23Gain, q14Gain)*lidarGain);
      display1.setCursor(1,0);
      display1.print("Waypoint: ");
      display1.println(String(targetHeading));
      display1.print("Waypoint #: ");
      display1.print(String(waypointCounter+1));
      display1.println("/12");
      display1.print("Waypoint Dist: ");
      display1.print(String(distance));
      display1.println("m");
      display1.setCursor(4,0);
      display1.print("Q1: ");
      display1.println(String(lidarQ1));
      display1.print("Q2: ");
      display1.println(String(lidarQ2));
      display1.print("Q3: ");
      display1.println(String(lidarQ3));
      display1.print("Q4: ");
      display1.println(String(lidarQ4));
      Serial.println("----------------------");
    } else {
      display1.setCursor(3,0);
      display1.print("No GPS fix.");
    }
  }
  if(servoTime + servoRate < millis()){
    servoTime = millis();
    steer.write(steeringUpdate);
  }
}

float calculateHeadingError(float _currentHeading, float _targetHeading) {
    float error = _targetHeading - _currentHeading;
    
    // Normalize to [-180, 180] range
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    return error;
}

void updateSteering(float _currentHeading, float _targetHeading) {
    float error = calculateHeadingError(_currentHeading, _targetHeading);
    
    if (abs(error) < 5) {
        // Go straight - within tolerance
        steering = CENTER_STEERING_POS;  // Center position
    } else {
        // Convert error to servo position
        // Positive error = turn right (>CENTER_STEERING_POS), negative error = turn left (<CENTER_STEERING_POS)
        float steeringOffset = error * TURN_GAIN;
        steering = CENTER_STEERING_POS + steeringOffset;
        
        // Constrain to servo limits
        steering = constrain(steering, MIN_STEERING_POS, MAX_STEERING_POS);
    }
}
