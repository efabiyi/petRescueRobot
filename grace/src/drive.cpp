#include <Arduino.h>
#include "driver/ledc.h"
#include "drive.h"
#include "utils.h"
#include "pins.h"

const int MIN_SPEED = 0;
const int MAX_SPEED = 1600;

float lastError = 0.0;
float lastOnLineError = 0.0;
unsigned long lastTime = 0;

void initializeDrive() {
  ledcSetup(FWD_LEFT_CHAN, 200, 12);
  ledcSetup(BWD_LEFT_CHAN, 200, 12);
  ledcSetup(FWD_RIGHT_CHAN, 200, 12);
  ledcSetup(BWD_RIGHT_CHAN, 200, 12);

  ledcAttachPin(FWD_LEFT_PWM, FWD_LEFT_CHAN);
  ledcAttachPin(BWD_LEFT_PWM, BWD_LEFT_CHAN);
  ledcAttachPin(FWD_RIGHT_PWM, FWD_RIGHT_CHAN);
  ledcAttachPin(BWD_RIGHT_PWM, BWD_RIGHT_CHAN);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  lastTime = millis();
}

void leftDrive(int speed) {
  debugPrint(" - L speed: ");
  if (speed > MIN_SPEED) {
    speed = min(speed, MAX_SPEED);
    ledcWrite(FWD_LEFT_CHAN, speed);
    ledcWrite(BWD_LEFT_CHAN, 0);
  } else {
    speed = MIN_SPEED - speed;
    speed = min(MIN_SPEED + speed, MAX_SPEED);
    ledcWrite(FWD_LEFT_CHAN, 0);
    ledcWrite(BWD_LEFT_CHAN, speed);  
    debugPrint("-");
  }
  debugPrint(String(speed));
}

void rightDrive(int speed) {
  debugPrint(" - R speed: ");
  if (speed > MIN_SPEED) {
    speed = min(speed, MAX_SPEED);
    ledcWrite(FWD_RIGHT_CHAN, speed);
    ledcWrite(BWD_RIGHT_CHAN, 0);
  } else {
    speed = MIN_SPEED - speed;
    speed = min(MIN_SPEED + speed, MAX_SPEED);
    ledcWrite(FWD_RIGHT_CHAN, 0);
    ledcWrite(BWD_RIGHT_CHAN, speed);  
    debugPrint("-");
  }
  debugPrint(String(speed));
}

void stopMotors() {
  ledcWrite(FWD_LEFT_CHAN, 0);
  ledcWrite(BWD_LEFT_CHAN, 0);
  ledcWrite(FWD_RIGHT_CHAN, 0);
  ledcWrite(BWD_RIGHT_CHAN, 0);  
}

void lineFollow(int baseSpeed, int threshold, float KP) {
  int leftReading = analogRead(LEFT_SENSOR);
  int rightReading = analogRead(RIGHT_SENSOR);
  debugPrint("Left: ");
  debugPrint(String(leftReading));
  debugPrint(" - Right: ");
  debugPrint(String(rightReading));

  bool offLine = (leftReading <= threshold) && (rightReading <= threshold);

  float error;
  float correction;
  int leftSpeed;
  int rightSpeed;

  if (!offLine) {
    unsigned long now = millis();
    float deltaTime = (now - lastTime) / 1000.0;
    lastTime = now;

    error = (leftReading - rightReading) / (leftReading + rightReading + 0.001);
    correction = (KP * error);

    debugPrint(" - ONLINE");
    debugPrint(" - Error: ");
    debugPrint(String(error));
    debugPrint(" - Correction: ");
    debugPrint(String(correction));

    lastOnLineError = error;
  } else {
    error = 20 * lastOnLineError;
    correction = KP * error;

    debugPrint(" - OFFLINE");
    debugPrint(" - Error: ");
    debugPrint(String(error));
    debugPrint(" - Derivative: N/A");
    debugPrint(" - Correction: ");
    debugPrint(String(correction));
  }

  leftSpeed = baseSpeed - (correction * baseSpeed/2);
  rightSpeed = baseSpeed + (correction * baseSpeed/2);
  leftDrive(leftSpeed);
  rightDrive(rightSpeed);
  
  debugPrintln("");
  
  lastError = error;
}

void drive(int leftSpeed, int rightSpeed) {
  leftDrive(leftSpeed);
  rightDrive(rightSpeed);
}

// void searchForLine() {
//   int l = 0;
//   int r = 0;
//   int direction = 1;
//   unsigned long startSearch = millis();
//   while (l < 100 && r < 100) {
//     l = analogRead(LEFT_SENSOR);
//     r = analogRead(RIGHT_SENSOR);

//     if ((millis() - startSearch) >= 1000) {
//       direction = -1 * direction;
//       startSearch = millis();
//     }

//     leftDrive(1000 * direction);
//     rightDrive(-1000 * direction);
//     delay(10);
//   }
// }

void uTurnRight(int threshold) {
  drive(1000, -1000);
  delay(500);
  int l = analogRead(LEFT_SENSOR);
  int r = analogRead(RIGHT_SENSOR);
  while (l < threshold && r < threshold) {
    drive(1000, -1000);
    l = analogRead(LEFT_SENSOR);
    r = analogRead(RIGHT_SENSOR);
  }
  stopMotors();
  delay(1000);
}

void uTurnLeft(int threshold) {
  drive(-1000, 1000);
  delay(500);
  int l = analogRead(LEFT_SENSOR);
  int r = analogRead(RIGHT_SENSOR);
  while (l < threshold && r < threshold) {
    drive(-1000, 1000);
    l = analogRead(LEFT_SENSOR);
    r = analogRead(RIGHT_SENSOR);
  }
  stopMotors();
  delay(1000);
}

void searchLine(int threshold) {
  int l = analogRead(LEFT_SENSOR);
  int r = analogRead(RIGHT_SENSOR);
  while (l < threshold && r < threshold) {
    drive(1000, -1000);
    delay(1000);
    drive(-1000, 1000);
    delay(1000);
    l = analogRead(LEFT_SENSOR);
    r = analogRead(RIGHT_SENSOR);
  }
  stopMotors();
  delay(1000);
}