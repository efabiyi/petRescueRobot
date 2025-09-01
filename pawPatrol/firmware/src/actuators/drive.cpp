#include <Arduino.h>
#include "driver/ledc.h"
#include "drive.h"
#include "utils.h"
#include "constants.h"

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

void  lineFollow(int baseSpeed, int threshold, float KP, Logger* logger) {
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
    if (lastOnLineError < 0) {
      leftDrive(900);
      rightDrive(-900);
    } else {
      leftDrive(-900);
      rightDrive(900);
    }
    // debugPrint(" - OFFLINE");
    // debugPrint(" - Error: ");
    // debugPrint(String(error));
    // debugPrint(" - Derivative: N/A");
    // debugPrint(" - Correction: ");
    // debugPrint(String(correction));
  }
    
  //debugPrintln("");

  leftSpeed = baseSpeed - (correction * baseSpeed/2);
  rightSpeed = baseSpeed + (correction * baseSpeed/2);
  leftDrive(leftSpeed);
  rightDrive(rightSpeed);
  
  debugPrintln("");
  String reflectanceData = "[Drive] Drive Data: Off Line? :  " + String(offLine ? " OFFLINE " : " ONLINE ");
  String driveData = "Left: "+String(leftReading)+" - Right: " + String(rightReading) + " Error: "+ String(error) + " - Correction: "+String(correction);
  logger->log(reflectanceData +" | "+ driveData);
  
  lastError = error;
}

void drive(int leftSpeed, int rightSpeed) {
  leftDrive(leftSpeed);
  rightDrive(rightSpeed);
}


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
  delay(500);
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
  delay(500);
}

void searchLine(int threshold) {
  unsigned long start = millis();
  int dir = 1;
  int count = 0;
  while (true) {
    int l = analogRead(LEFT_SENSOR);
    int r = analogRead(RIGHT_SENSOR);
    if (l >= threshold || r >= threshold) break;
    if ((millis() - start) > (500 + 100 * count)) {
      dir *= -1;
      start = millis();
      count++;
    }
    drive(900 * dir, -900 * dir);
    
  }
  stopMotors();
  delay(500);
}