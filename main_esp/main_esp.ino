#include <Adafruit_VL53L0X.h>   // for LiDAR
#include <ArduinoJson.h>        // for wifi package parsing
#include <JY901.h>              // for IMU
#include <math.h>       
// #include <WiFi.h>               // for wifi
#include <Wire.h>
// #include "ESPAsyncWebServer.h"  // for matlab/wifi
#include "NewPing.h"            // for ultrasonic

// motor
const int motor1Pin1 = 25;
const int motor1Pin2 = 26;
const int enable1Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// fan
const int fanPin = 32;

// ultrasonic
const int usTrigPin = 34;
const int usEchoPin = 35;

// camera

// constants associated with IMU
float acc     [3];
float gyro    [3];
float angles  [3];
float mag     [3];
short dstatus [4];

float gpsHeight;
float gpsYaw;
float gpsVelocity;

float lpressure;
float laltitude;

// lidar
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
double lidar = 0;

// // wifi communication with matlab
// const char* ssid = "Michaela-ESP-Access";  
// const char* password = "123456789"; 

// AsyncWebServer server(80);
// DynamicJsonDocument jsonBuffer(256);

void setup() {
  // setup serial
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }

  // setup IMU
  Serial.println("IMU setup");
  JY901.StartIIC();
  
  // setup lidar
  Serial.println("LiDAR setup");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  lox.startRangeContinuous(); // start continuous ranging

  // setup motor (uses L298N board)
  Serial.println("Motor setup");
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  // setup Fan
  Serial.println("setup fan");
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);  // start with fan off

  Serial.println("=======================\nDone setup\n=======================\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(motor1Pin1, HIGH);
  // digitalWrite(motor1Pin2, LOW);
  // while (dutyCycle <= 255){
  //   ledcWrite(pwmChannel, dutyCycle);   
  //   Serial.print("Forward with duty cycle: ");
  //   Serial.println(dutyCycle);
  //   dutyCycle = dutyCycle + 5;
  //   delay(500);
  // }
  // dutyCycle = 200;

  digitalWrite(fanPin, HIGH);
  delay(5000);
  digitalWrite(fanPin, LOW);
  delay(5000);
  
}
