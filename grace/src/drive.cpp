#include <Arduino.h>
#include "driver/ledc.h"
#include "drive.h"
#include "utils.h"
#include "pins.h"

const int MIN_SPEED = 500;
const int MAX_SPEED = 2000;
const float KP = 0.5f;
const float KD = 0.0f;

float lastError = 0.0;
float lastOnLineError = 0.0;
unsigned long lastTime = 0;
unsigned long lastWriteTime = 0;

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
  lastWriteTime = millis();
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

void lineFollow(int baseSpeed, int threshold) {
  int leftReading = analogRead(LEFT_SENSOR);
  int rightReading = analogRead(RIGHT_SENSOR);
  debugPrint("Left: ");
  debugPrint(String(leftReading));
  debugPrint(" - Right: ");
  debugPrint(String(rightReading));

  bool offLine = leftReading < threshold || rightReading < threshold;

  float error;
  float derivative;
  float correction;
  int leftSpeed;
  int rightSpeed;

  if (!offLine) {
    unsigned long now = millis();
    float deltaTime = (now - lastTime) / 1000.0;
    lastTime = now;

    error = (leftReading - rightReading) / 10000.0;
    derivative = (error - lastError) / deltaTime;
    correction = (KP * error) + (KD * derivative);

    debugPrint(" - ONLINE");
    debugPrint(" - Error: ");
    debugPrint(String(error));
    debugPrint(" - Derivative: ");
    debugPrint(String(derivative));
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

  // if ((millis() - lastWriteTime) >= 50) {
    leftSpeed = baseSpeed - (correction * baseSpeed/2);
    rightSpeed = baseSpeed + (correction * baseSpeed/2);
    leftDrive(leftSpeed);
    rightDrive(rightSpeed);
  //   lastWriteTime = millis();
  // }

  debugPrintln("");
  
  lastError = error;
}

void testDrive(int leftSpeed, int rightSpeed) {
  leftDrive(leftSpeed);
  rightDrive(rightSpeed);
}