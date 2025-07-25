#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 100;
const int BASE_SPEED = 1500;
const int THRESHOLD = 0;
const int APPROACH_SPEED = 700;
const int COOL_DOWN = 3000;

Scanner scanner;
Claw claw;

enum State {
  DRIVE, // line following and scanning until object is in range
  VERIFY, // checking if object is a wall
  SEARCH, // searching for pet magnet
  GRAB, // grabbing pet
  ESCAPE, // time up, escape
  STOP // stop
};

State currentState = DRIVE;
int petAngle = -1;
int petDistance = 9999;
unsigned long lastSeenPetTime = 0;

void setup() {
  Serial.begin(9600);
  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");
  initializeDrive();
  claw.initialize();
  delay(1000);
  // lastSeenPetTime = millis();
}

void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
}

void loop() {
  switch (currentState) {
    case DRIVE:
      lineFollow(BASE_SPEED, THRESHOLD);
      if (scanner.completedScan()) {
        PolarPoint closestObject = scanner.getClosestObject();
        if (closestObject.distance <= IN_RANGE_DISTANCE && closestObject.distance >= REACHABLE_DISTANCE) {
          // if (millis() - lastSeenPetTime >= COOL_DOWN) {
            currentState = VERIFY;
            // Serial.println("something seen, stopping motors");
            stopMotors();
            delay(100); // make sure wheels are stopped
          // }
        }
      }
      scanner.scanOneStep(0);
      break;
    case VERIFY:
      if (scanner.completedScan()) {
        if (!scanner.closestObjectIsWall()) {
          PolarPoint pet = scanner.honeIn(scanner.getClosestObject().angle);
          // if (pet.angle >= 0) {
            
          //   currentState = SEARCH;
          // } else {
          //   scanner.reset();
          //   currentState = DRIVE;
          // }
          // Serial.println("not wall, stopping motors");
          scanner.setServoAngle(pet.angle);
          stopMotors();
          delay(3000);
          // Serial.println("continuing");
          currentState = DRIVE;
          // lastSeenPetTime = millis();
        } else {
          // Serial.println("wall, resetting");
          scanner.reset();
          currentState = DRIVE;
        }
      }
      scanner.scanOneStep(100);
      break;
    case SEARCH:
      break;
    case GRAB:
      break;
    case ESCAPE:
      break;
  }
}
