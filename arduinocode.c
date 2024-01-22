#include "Wire.h"
#include "MPU6050.h"
#include "CustomVector.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <WebSerial.h>

#define LED 2

// AsyncWebServer server(80);

int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;


const char* ssid = "Pixel_1890";          // Your WiFi SSID
const char* password = "12345678";  // Your WiFi Password


Adafruit_MPU6050 mpu;

// void recvMsg(uint8_t *data, size_t len){
//   WebSerial.println("Received Data...");
//   String d = "";
//   for(int i=0; i < len; i++){
//     d += char(data[i]);
//   }
//   WebSerial.println(d);
//   // if (d == "ON"){
//   //   digitalWrite(LED, HIGH);
//   // }
//   // if (d=="OFF"){
//   //   digitalWrite(LED, LOW);
//   // }
// }


MPU6050 neck;
CustomVector initialNeckAngle(0,0,0), currentNeckAngle(0,0,0);

const double angleThreshold = 5; // Change in angle to detect
const double deltaTime = 0.01; // Time in seconds between measurements (10ms)
const double sesntitivty = 100.0;
const double tolerance = 0.1;
const double sendData = 60000;
bool bad = false;
int coolDown = 0;
int timesVibrated = 0;
bool badPosture = false;
void setup() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
    ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
    ledcAttachPin(enable1Pin, pwmChannel);


    //Set up the MPU
    Serial.begin(115200);
    Wire.begin();
    neck.initialize();
    //spine.initialize();
    //Check the connection
    if (!neck.testConnection()) {
        Serial.println("Neck MPU connection failed");
        while (1);
    }
    Serial.println("Neck MPU connection successful");
  //   WiFi.mode(WIFI_STA);
  //   WiFi.begin(ssid, password);
  //   if (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //     Serial.printf("WiFi Failed!\n");
  //     return;
  //   }
  //   Serial.print("IP Address: ");
  //   Serial.println(WiFi.localIP());
  // // WebSerial is accessible at "<IP Address>/webserial" in browser
  //   WebSerial.begin(&server);
  //   WebSerial.msgCallback(recvMsg);
  //   server.begin();

    Serial.println("");

    //if (!neck.testConnection()) {
    //   Serial.println("Spine MPU connection failed");
    //    while (1);
    //}

    //Serial.println("spine NPU connection successful");

    //Get the initial angle of neck
    int16_t gx, gy, gz;
    neck.getRotation(&gx, &gy, &gz);
    int16_t initialAngleX = gx / sesntitivty * deltaTime; 
    int16_t initialAngleY = gy / sesntitivty * deltaTime;
    int16_t initialAngleZ = gz / sesntitivty * deltaTime;

    Serial.println(initialNeckAngle.x);
    Serial.println(initialNeckAngle.y);
    Serial.println(initialNeckAngle.z);
    currentNeckAngle = CustomVector(initialAngleX, initialAngleY, initialAngleZ);

    

    //Get the initial angle of spine
    //spine.getRotation(&gx, &gy, &gz);
    //initialAngleX = gx / sesntitivty; 
    //initialAngleY = gy / sesntitivty;
    //initialAngleZ = gz / sesntitivty;



    //initialSpineAngle = Vector(initialAngleX, initialAngleY, initialAngleZ);
    //currentSpineAngle = Vector(initialAngleX, initialAngleY, initialAngleZ);
}

// bool bad_posture_tilt(CustomVector initial, CustomVector final){
//   CustomVector result = final - initial;
//   if (result.magnitude() > angleThreshold)
//     return true;
//   else 
//     return false;
// }

bool bad_posture_tilt(CustomVector initial, CustomVector final){
  if (abs(final.x - initial.x) > angleThreshold || abs(final.y - initial.y) > angleThreshold || abs(final.z - initial.z) > angleThreshold) {
    return true;
  } else {
    return false;
  }
}


bool bad_posture_symmetry(){

}

CustomVector calculateAngle(MPU6050 mpu){
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  //Get the current gyro
  double deltaAngleX = (gx / sesntitivty) * deltaTime;
  double deltaAngleY = (gy / sesntitivty) * deltaTime;
  double deltaAngleZ = (gz / sesntitivty) * deltaTime;

  return CustomVector(deltaAngleX, deltaAngleY, deltaAngleZ);
}

void loop() {

  if (bad){
      delay(3000);

      currentNeckAngle.x = 0;
      currentNeckAngle.y = 0;
      currentNeckAngle.z= 0;

      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);

      bad = false;
  }

  int val = analogRead(0);
  CustomVector newAngle = calculateAngle(neck);

  // checks to see if new angle changed by more than a threshold amount that is too small for humans to emulate, to avoid accelerometer "drift"
  if (abs(newAngle.x - currentNeckAngle.x) > tolerance && abs(newAngle.y - currentNeckAngle.y) > tolerance && abs(newAngle.z - currentNeckAngle.z) > tolerance) {
    currentNeckAngle = currentNeckAngle + newAngle;
  }
  Serial.println(currentNeckAngle.x);
  Serial.println(currentNeckAngle.y);
  Serial.println(currentNeckAngle.z);

  if(bad_posture_tilt(initialNeckAngle, currentNeckAngle)){
    if (coolDown / 500 >= 1) {
      Serial.println("Bad neck position");
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      ledcWrite(pwmChannel, dutyCycle);
      bad = true;
      
    }
    // badPosture = true;
    
    // delay(1000);
    // if (coolDown / 5000 >= 1){
    //   WebSerial.println("Bad posture detected");
    //   coolDown = 0;
    // }
  }else{
  // if (currentTime % 10000 == 0) {
  //   if (badPosture) {
  //     timesVibrated ++;
  //   }
  // badPosture = false;
  // currentTime = millis();
  // WebSerial.println(timesVibrated);
  // }
  // if (currentTime % 86400000 == 0) {
  //   WebSerial.println("Today, you were reminded this many times: " + String(timesVibrated));
  //   timesVibrated = 0;
  // }
  coolDown += 10;
  delay(10);
  }

}