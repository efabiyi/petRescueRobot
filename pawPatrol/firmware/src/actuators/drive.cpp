#include <Arduino.h>
#include "driver/ledc.h"
#include "drive.h"
#include "logger.h"

// Constants
const int BASE_SPEED = 1000;
const int MIN_SPEED = 0;
const int MAX_SPEED = 1600;
const int THRESHOLD = 130;
const float KP = 0.8f;
const float KD = 0.0f;

// Pin definitions
const int LEFT_SENSOR = 36;
const int RIGHT_SENSOR = 34;
const int FWD_LEFT_PWM = 14;
const int BWD_LEFT_PWM = 4;
const int FWD_RIGHT_PWM = 13;
const int BWD_RIGHT_PWM = 5;
const int FWD_LEFT_CHAN = 0;
const int BWD_LEFT_CHAN = 1;
const int FWD_RIGHT_CHAN = 2;
const int BWD_RIGHT_CHAN = 3;

// Global variables
float lastError = 0.0;
unsigned long lastTime = 0;
float lastCorrection = 0.0;
bool offLine = false;

const int OFF_LINE_THRESHOLD = 700;
int offLineCounter = OFF_LINE_THRESHOLD;

void debugPrint(const String &message) {
  // Serial.print(message);     // Comment out to improve performance
}

void debugPrintln(const String &message) {
  debugPrint(message);
  Serial.println();
}

void initializeDrive() {
  ledcSetup(FWD_LEFT_CHAN, 250, 12);
  ledcSetup(BWD_LEFT_CHAN, 250, 12);
  ledcSetup(FWD_RIGHT_CHAN, 250, 12);
  ledcSetup(BWD_RIGHT_CHAN, 250, 12);

  ledcAttachPin(FWD_LEFT_PWM, FWD_LEFT_CHAN);
  ledcAttachPin(BWD_LEFT_PWM, BWD_LEFT_CHAN);
  ledcAttachPin(FWD_RIGHT_PWM, FWD_RIGHT_CHAN);
  ledcAttachPin(BWD_RIGHT_PWM, BWD_RIGHT_CHAN);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  lastTime = millis();
}

void leftDriveForward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(BWD_LEFT_CHAN, 0);
  ledcWrite(FWD_LEFT_CHAN, speed);
} 

void leftDriveBackward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(FWD_LEFT_CHAN, 0);
  ledcWrite(BWD_LEFT_CHAN, speed);  
}

void rightDriveForward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(BWD_RIGHT_CHAN, 0);
  ledcWrite(FWD_RIGHT_CHAN, speed);
}

void rightDriveBackward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(FWD_RIGHT_CHAN, 0);
  ledcWrite(BWD_RIGHT_CHAN, speed);  
}

float calculateError(int l, int r) {
  return (l * -1.0 + r * 1.0) / (l + r + 0.001);
}

bool isOffLine(int l, int r) {
  return (l < THRESHOLD && r < THRESHOLD);
}

void drive() {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;
  lastTime = now;

  int leftReading = analogRead(LEFT_SENSOR);
  int rightReading = analogRead(RIGHT_SENSOR);
  debugPrint("Left: ");
  debugPrint(String(leftReading));
  debugPrint(" - Right: ");
  debugPrint(String(rightReading));
  bool offLine = isOffLine(leftReading, rightReading);
  String pidData = "";

  if (offLine) {
    debugPrint(" - OFF LINE");
    int speed = BASE_SPEED * 1.2;
    if (lastCorrection > 0) {
      leftDriveBackward(speed);
      rightDriveForward(speed);
      debugPrint(" - L BACKWARD: ");
      debugPrint(String(speed));
      debugPrint(" - R FORWARD: ");
      debugPrintln(String(speed));
      pidData = "PID Data: Last Correction: " + String(lastCorrection) + ", Left Backward: " + String(speed) + ", Right Forward: " + String(speed);
    } else {
      leftDriveForward(speed);
      rightDriveBackward(speed);
      debugPrint(" - L FORWARD: ");
      debugPrint(String(speed));
      debugPrint(" - R BACKWARD: ");
      debugPrintln(String(speed));
      pidData = "PID Data: Last Correction: " + String(lastCorrection) + ", Left Forward: " + String(speed) + ", Right Backward: " + String(speed);
    }
  } else { // If on line, calculate error and correction
    float error = calculateError(leftReading, rightReading);
    float derivative = (error - lastError) / deltaTime;
    float correction = (KP * error) + (KD * derivative);

    int leftSpeed = BASE_SPEED - (correction * BASE_SPEED/2);
    int rightSpeed = BASE_SPEED + (correction * BASE_SPEED/2);
  
    leftDriveForward(leftSpeed);
    rightDriveForward(rightSpeed);
    
    lastError = error;
    lastCorrection = correction;

    debugPrint(" - ON LINE");
    debugPrint(" - Error: ");
    debugPrint(String(error));
    debugPrint(" - Derivative: ");
    debugPrint(String(derivative));

    debugPrint(" - Correction: ");
    debugPrint(String(correction));
    debugPrint(" - L speed: ");
    debugPrint(String(constrain(leftSpeed, MIN_SPEED, MAX_SPEED)));
    debugPrint(" - R speed: ");
    debugPrintln(String(constrain(rightSpeed, MIN_SPEED, MAX_SPEED)));
    pidData = "PID Data: Error: " + String(error) + ", Derivative: " + String(derivative) + ", Correction: " + String(correction) + ", Left Speed: " + String(leftSpeed) + ", Right Speed: " + String(rightSpeed);
  }
  String driveData = "Reflectance Data: Left:" + String(leftReading) + " - Right: " + String(rightReading) + " - Off Line: " + (offLine ? "Yes" : "No");
  debugPrint(driveData + " | " + pidData);

  delay(10);
}

void testDrive(int leftSpeed, int rightSpeed) {
  leftDriveForward(leftSpeed);
  rightDriveForward(rightSpeed);
  delay(10);
}

void testDriveBwd(int leftSpeed, int rightSpeed) {
  leftDriveBackward(leftSpeed);
  rightDriveBackward(rightSpeed);
  delay(10);
}
