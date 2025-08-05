#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"

const int BASE_SPEED = 1200;
const int SLOW_SPEED = 800;
const int FAST_SPEED = 1600;
const int THRESHOLD = 60;
const float KP = 0.6f;

int petAngle[] = {15, 135, 175, 175, 175, 0};
int inRange[] = {300, 300, 250, 400, 450, 450};
int cooldown[] = {500, 1000, 0, 3000, 5000, 3000}; // 3rd delay is hard-coded slow driving
int minAngle[] = {0, 90, 90, 90, 90, 0};
int maxAngle[] = {90, 180, 180, 180, 180, 90};

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
unsigned long endTime = 0;

void setup() {
  Serial.begin(9600);
  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");
  initializeDrive();
  claw.initialize();
  lastSeenPetTime = millis();
  cutOffTime = millis() + 90000;
  endTime = millis() + 120000;
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
  // if (millis() >= cutOffTime && currentState != ESCAPE) {
  //   // resetAll();
  //   // claw.setElbowServo(120);
  //   // delay(100);
  //   // uTurn(THRESHOLD);
  //   // currentState = ESCAPE;
  //   stopMotors();
  //   return;
  // }
  if (millis() >= endTime) {
    stopMotors();
    return;
  }
  switch (currentState) {
    case PRE_GATE:
      scanner.setServoAngle(10);
      lineFollow(FAST_SPEED, THRESHOLD, KP);
      if ((scanner.readDistance() <= 200)) {
        currentState = DRIVE;
        lastSeenPetTime = millis();
      }
      break;
    case DRIVE:
      scanner.setServoAngle(petAngle[petCount]);
      lineFollow(BASE_SPEED, THRESHOLD, KP);
      if (scanner.readDistance() <= inRange[petCount]) {
        if ((millis() - lastSeenPetTime) >= cooldown[petCount]) {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
      break;
    case APPROACH:
      while (true) {
        pet = scanner.getClosestObject(minAngle[petCount], maxAngle[petCount]);
        if (pet.distance > inRange[petCount]) {
          currentState = DRIVE;
          break;
        }
        if ((1.0 * pet.distance * sin(radians(pet.angle))) < 102) {
          scanner.setServoAngle(pet.angle);
          stopMotors();
          delay(500);
          scanner.setServoAngle(150);
          currentState = GET_PET;
          // currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= pet.distance) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP);
        }
        stopMotors();
        delay(500);
      }
      break;
    case VERIFY: {
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > inRange[petCount]) {
        currentState = DRIVE;
        break;
      }
      if ((1.0 * pet.distance * sin(radians(pet.angle))) > 102) {
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
      PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw
      claw.grabPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1);
      petCount++;
      if (petCount == 3) petCount = 4;
      if (petCount == 1) {
        scanner.setServoAngle(10);
        while (scanner.readDistance() > 300) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP);
        }
        unsigned long startRamp = millis();
        while ((millis() - startRamp) <= 3000) {
          lineFollow(FAST_SPEED, THRESHOLD, KP);
        }
        stopMotors();
        scanner.setServoAngle(150);
        delay(500);
        claw.rampToss();
        currentState = RAMP;
        break;
      } else if (petCount == 2) {
        scanner.setServoAngle(150);
        delay(500);
        claw.windowToss();
        unsigned long start = millis();
        while ((millis() - start) <= 2000) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP);
        }
        currentState = DRIVE;
      } else {
        scanner.setServoAngle(150);
        delay(500);
        claw.dump();
        currentState = DRIVE;
      }
      lastSeenPetTime = millis();
      if (petCount == 6) {
        resetAll();
        claw.setElbowServo(120);
        delay(100);
        uTurn(THRESHOLD);
        currentState = ESCAPE;
        break;
      }
      resetAll();
      break;
    }
    case RAMP:
      scanner.setServoAngle(180);
      lineFollow(BASE_SPEED, THRESHOLD, KP);
      if (scanner.readDistance() <= 400 && (millis() - lastSeenPetTime) > cooldown[petCount]) {
        while (scanner.readDistance() <= 400) {
          lineFollow(BASE_SPEED, THRESHOLD, KP);
        }
        stopMotors();
        delay(500); // make sure wheels are stopped
        currentState = APPROACH;
      }
      break;
    case ESCAPE:
      lineFollow(BASE_SPEED, THRESHOLD, KP);
      break;
  }
}
