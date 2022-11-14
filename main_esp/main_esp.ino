#include <Adafruit_VL53L0X.h>   // for LiDAR
#include <ArduinoJson.h>        // for wifi package parsing
#include <JY901.h>              // for IMU
#include <math.h>       
#include <WiFi.h>               // for wifi
#include <Wire.h>
#include "ESPAsyncWebServer.h"  // for matlab/wifi
#include "NewPing.h"            // for ultrasonic

// motor
const int motor1Pin1 = 25;
const int motor1Pin2 = 26;
const int enable1Pin = 27; 

// Setting PWM properties for motor speed
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

int motorSpeedMin = 50;
int motorSpeedMax = 150;
int motorSpeedMean = (motorSpeedMin + motorSpeedMax)/2;
int motorSpeed = motorSpeedMean;

// fan
// reference: https://www.youtube.com/watch?v=UJK2JF8wOu8&t=137s 
const int fanPin = 32;
int fanSpeed = 0;
int fanSpeedMin = 0;
int fanSpeedMax = 0;


// ultrasonic
// reference: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// especially the Event Timer Sketch
const int usTrigPin = 34;
const int usEchoPin = 35;
const int usMaxDistance = 200;  // maximum sensor distance (in cm). sensor is rated for ___ to ___
double ultrasonic = 0;
NewPing sonar(usTrigPin, usEchoPin, usMaxDistance);

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

// wifi communication with matlab
const char* ssid = "Michaela-ESP-Access";  
const char* password = "123456789"; 

AsyncWebServer server(80);
DynamicJsonDocument jsonBuffer(256);

// possible gestures are: ['okay', 'peace', 'thumbs up', 'thumbs down', 'call me', 'stop', 'rock', 'live long', 'fist', 'smile']
String hand_gesture = "";
String prev_hand_gesture = "";

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

  // setup ultrasonic

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

  // WiFi
  Serial.println("setup WiFi");
  setup_server();

  Serial.println("=======================\nDone setup\n=======================\n");
}

void loop() {

  // ================================
  // ==== UPDATE SENSOR READINGS ====
  // ================================
  // IMU
  get_angles(angles);

  // LiDAR
  read_lidar();

  // MOTOR & FAN SETTINGS ARE CONTROLLED BY WIFI (see process_hand_gesture), 
  // SO THE ONLY THING HERE IS TO CHECK THE SAFETY CONDITIONS

  
}

double read_lidar()
{
  lidar = lox.readRange();
  return lidar;
}

// double read_ultrasonic()
// {

// }

void get_angles(float (& angle) [3])
{
  JY901.GetAngle();
  angle[0] = (float)JY901.stcAngle.Angle[0]/32768*180;
  angle[1] = (float)JY901.stcAngle.Angle[1]/32768*180;
  angle[2] = (float)JY901.stcAngle.Angle[2]/32768*180;

}

void set_motor(int dutyCycle)
{
  if (dutyCycle < 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);

  } else if (dutyCycle > 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);

  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

  }
  ledcWrite(pwmChannel, dutyCycle); 

}

void setup_server() 
{
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP Address = "); Serial.println(IP);

  // setup for recieving from matlab
  server.on("/get_gesture",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {

      String msg = String((char *)data_in, len); // takes the given value 

     // parse the json message. the format is 
      DeserializationError error = deserializeJson(jsonBuffer, msg); 
      if (error) {
        request->send_P(200, "text/plain", "-1"); 
        Serial.println("error in json parsing");
        return;
      }

      hand_gesture = jsonBuffer["hand_gesture"].as<String>();
      process_hand_gesture(hand_gesture);

      request->send_P(200, "text/plain", "1"); 

  });

  // Start server (needed)
  server.begin();  

}

void process_hand_gesture(String hand_gesture) {
  Serial.print("Received hand gesture = "); Serial.println(hand_gesture);
  // possible gestures are: ['okay', 'peace', 'thumbs up', 'thumbs down', 'call me', 'stop', 'rock', 'live long', 'fist', 'smile']
  if (hand_gesture.equalsIgnoreCase("stop") || hand_gesture.equalsIgnoreCase("fist")) {
    // STOP
    motorSpeed = 0;
    Serial.println("turning motor off\n");

  } else if (hand_gesture.equalsIgnoreCase("thumbs_up")) {
    // SPEED UP, limited
    motorSpeed = min(motorSpeed+1, motorSpeedMax);
    Serial.println("increasing motor speed\n");

  } else if (hand_gesture.equalsIgnoreCase("thumbs_down")) {
    // SLOW DOWN, limited
    motorSpeed = max(motorSpeed-1, motorSpeedMin);
    Serial.println("decreasing motor speed\n");
    
  } else if ((hand_gesture.equalsIgnoreCase("okay")  || \
              hand_gesture.equalsIgnoreCase("peace") || \
              hand_gesture.equalsIgnoreCase("rock")  || \
              hand_gesture.equalsIgnoreCase("live long")) && 
              motorSpeed == 0) {
    // START
    motorSpeed = motorSpeedMean;
    Serial.println("turning on motor\n");
  } // OTHERWISE, NOTHING! 
  set_motor(motorSpeed);
}

void check_safety_features() {
  
}