#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"

#define COUNT 10

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 100;
const int BASE_SPEED = 800;
const int THRESHOLD = 0;
const int COOL_DOWN = 1000;
 
int petCount = 0;

Scanner scanner;
Claw claw;
PolarPoint pet;

enum State {
  PRE_GATE,
  DRIVE, // line following and scanning until object is in range
  VERIFY, // checking if object is a wall
  GET_PET, // searching for pet magnet
  ESCAPE, // time up, escape
  STOP // stop
};

State currentState = PRE_GATE;
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
  currentState = DRIVE;
  petCount = 0;
  lastSeenPetTime = millis();
  pet.angle = -1;
  pet.distance = 9999;
}

void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
  pet.angle = -1;
  pet.distance = 9999;
}

void loop() {
  switch (currentState) {
    case PRE_GATE:
    scanner.setServoAngle(10);
      lineFollow(BASE_SPEED, THRESHOLD);
      if ((scanner.readDistance() <= 200)) {
        currentState = DRIVE;
        lastSeenPetTime = millis() - 500;
      }
      break;
    case DRIVE:
      lineFollow(BASE_SPEED, THRESHOLD);
      // if (scanner.scanOneStep(0)) {
      //   if ((millis() - lastSeenPetTime) >= COOL_DOWN) {
      //     stopMotors();
      //     delay(500); // make sure wheels are stopped
      //     currentState = VERIFY;
      //   }
      // }
      break;
    case VERIFY: {
      pet = scanner.getClosestObject();
      pet = scanner.honeIn(pet.angle);
      if (pet.distance <= IN_RANGE_DISTANCE) {
        scanner.setServoAngle(pet.angle);
        stopMotors();
        delay(1000);
        // Serial.println("distance: " + String(pet.distance) + " mm");
        // Serial.println("angle: " + String(pet.angle) + " degrees");
        // currentState = DRIVE;
        currentState = GET_PET;
      } else {
        currentState = DRIVE;
      }
      break;
    }
    case GET_PET: {
      Serial.println("pet found at " + String(pet.angle) + " degrees at " + String(pet.distance) + " mm");
      PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw
      if (claw.searchPet(offsetPet.angle, offsetPet.distance)) {
        claw.setElbowServo(90);
        claw.setBaseServo(90);
        delay(3000);
        claw.openGripper();
        delay(1000);
        petCount++;
        lastSeenPetTime = millis();
        if (petCount == 1) lastSeenPetTime = millis() + 9000;
      }
      resetAll();
      currentState = DRIVE;
      break;
    }
    case ESCAPE:
      break;
  }
}
