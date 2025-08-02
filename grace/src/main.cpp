#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"

#define COUNT 10

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 250;
const int BASE_SPEED = 1100;
const int SLOW_SPEED = 800;
const int FAST_SPEED = 1200;
const int THRESHOLD = 0;
const int COOL_DOWN = 3000;

int inRange = REACHABLE_DISTANCE;
int reachable = REACHABLE_DISTANCE;

Scanner scanner;
Claw claw;
PolarPoint pet;

enum State {
  PRE_GATE,
  DRIVE, // line following and scanning until object is in range
  APPROACH,
  VERIFY, // checking if object is a wall
  GET_PET, // searching for pet magnet
  RAMP,
  ESCAPE, // time up, escape
  STOP // stop
};

State currentState = PRE_GATE;
unsigned long lastSeenPetTime = 0;
int petCount = 0;
unsigned long cutOffTime = 0;


void setup() {
  Serial.begin(9600);
  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");
  initializeDrive();
  claw.initialize();
  // claw.closeGripper();
  // PolarPoint p;
  // p.distance = 200;
  // p.angle = 30;
  // scanner.setServoAngle(p.angle);
  // delay(100);
  // PolarPoint pp = calculateOffset(p);
  // claw.setZAxisServo(pp.angle);
  // delay(500);
  lastSeenPetTime = millis();
  cutOffTime = millis() + 90000;
  pet.angle = -1;
  pet.distance = 9999;
  delay(1000);
}

void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
  pet.angle = -1;
  pet.distance = 9999;
  delay(500);
}

void loop() {
  // if (millis() >= cutOffTime) {
  //   resetAll();
  //   currentState = ESCAPE;
  // }
  switch (currentState) {
    case PRE_GATE:
      scanner.setServoAngle(10);
      lineFollow(FAST_SPEED, THRESHOLD);
      if ((scanner.readDistance() <= 200)) {
        currentState = DRIVE;
        lastSeenPetTime = millis() - (COOL_DOWN - 500);
      }
      break;
    case DRIVE:
      lineFollow(BASE_SPEED, THRESHOLD);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE)) {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN) {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
      break;
    case APPROACH:
      while (true) {
        pet = scanner.getClosestObject(0, 180);
        if (pet.distance > IN_RANGE_DISTANCE) {
          currentState = DRIVE;
          break;
        }
        if ((1.0 * pet.distance * sin(radians(pet.angle))) < 101.6) {
          currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= 300) {
          lineFollow(SLOW_SPEED, THRESHOLD);
        }
        stopMotors();
        delay(500);
      }
    case VERIFY: {
      // Serial.println("pet spotted at " + String(pet.angle) + " degrees at " + String(pet.distance) + " mm");
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > IN_RANGE_DISTANCE) {
        currentState = DRIVE;
        break;
      }
      if (pet.distance > REACHABLE_DISTANCE) {
        currentState = APPROACH;
        break;
      }
      scanner.setServoAngle(pet.angle);
      stopMotors();
      delay(500);
      scanner.setServoAngle(150);
      currentState = GET_PET;
      break;
    }
    case GET_PET: {
      // Serial.println("pet found at " + String(pet.angle) + " degrees at " + String(pet.distance) + " mm");
      PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw
      if (claw.searchPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1)) {
        petCount++;
        if (petCount == 1) {
          scanner.setServoAngle(10);
          while (scanner.readDistance() > 300) {
            lineFollow(SLOW_SPEED, THRESHOLD);
            delay(10);
          }
          unsigned long startRamp = millis();
          while ((millis() - startRamp) <= 4000) {
            lineFollow(BASE_SPEED, THRESHOLD);
            delay(10);
          }
          stopMotors();
          delay(500);
          claw.rampToss();
          currentState = RAMP;
        } else if (petCount == 2) {
          scanner.setServoAngle(150);
          delay(1000);
          claw.windowToss();
          lastSeenPetTime = millis();
          currentState = DRIVE;
        } else {
          scanner.setServoAngle(150);
          delay(1000);
          claw.dump();
          lastSeenPetTime = millis();
          currentState = DRIVE;
        }
      } else {
        currentState = DRIVE;
      }
      if (petCount == 6) currentState = ESCAPE;
      resetAll();
      break;
    }
    case RAMP:
      scanner.setServoAngle(180);
      lineFollow(BASE_SPEED, THRESHOLD);
      if ((scanner.readDistance() <= 400)) {
        while (scanner.readDistance() <= 400) {
          lineFollow(BASE_SPEED, THRESHOLD);
        }
        stopMotors();
        delay(500); // make sure wheels are stopped
        currentState = VERIFY;
      }
      break;
    case ESCAPE:
      stopMotors();
      delay(100);
      break;
  }
}
