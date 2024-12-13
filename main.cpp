#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>

#define MOTOR1_PIN 3
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 9
Adafruit_MPU6050 mpu;
Adafruit_L3GD20_Unified gyro;

SoftwareSerial ss(4, 3); //rx, rt
TinyGPSPlus gps;

Servo motor1, motor2, motor3, motor4;

float latA = 40.7128;
float lonA = -74.0060;
float latB = 40.7306;
float lonB = -73.9352;

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }

  if (!gyro.begin()) {
    Serial.println("Failed to initialize Gyroscope!");
    while (1);
  }
  
  Serial.println("Initializing GPS...");
  delay(1000);
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    
    Serial.print("Current Location: ");
    Serial.print("Lat: "); Serial.print(currentLat, 6);
    Serial.print(" Lon: "); Serial.println(currentLon, 6);
    
    if (currentLat != latB && currentLon != lonB) {
      float distance = getDistance(currentLat, currentLon, latB, lonB);
      float bearing = getBearing(currentLat, currentLon, latB, lonB);
      
      controlMotors(bearing);
    }
  }
}

float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);
  
  float dlat = lat2Rad - lat1Rad;
  float dlon = lon2Rad - lon1Rad;
  
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371 * c; 
  
  return distance;
}

float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);
  
  float dLon = lon2Rad - lon1Rad;
  
  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  
  float bearing = atan2(y, x);
  bearing = degrees(bearing);
  if (bearing < 0) bearing += 360;
  
  return bearing;
}

void controlMotors(float bearing) {
  int baseSpeed = 1500;
  
  if (bearing < 90) {
    motor1.write(baseSpeed + 20);
    motor2.write(baseSpeed);
    motor3.write(baseSpeed); 
    motor4.write(baseSpeed + 20); 

    } else if (bearing >= 90 && bearing < 180) {
    motor1.write(baseSpeed);
    motor2.write(baseSpeed + 20); 
    motor3.write(baseSpeed + 20); 
    motor4.write(baseSpeed); 
  } else if (bearing >= 180 && bearing < 270) {
    motor1.write(baseSpeed + 20); 
    motor2.write(baseSpeed); 
    motor3.write(baseSpeed); 
    motor4.write(baseSpeed + 20); 
  } else {
    motor1.write(baseSpeed); 
    motor2.write(baseSpeed + 20); 
    motor3.write(baseSpeed + 20); 
    motor4.write(baseSpeed); 
  }
}