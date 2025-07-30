#include <WiFi.h>
#include <Arduino.h>

#include "secrets.h"
#include "driver/ledc.h"
#include "wifiManager.h"
#include "logger.h"
#include "constants.h"

#include "drive.h"
#include "hallsensor.h"
#include "scanner.h"
#include "claw.h"

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 100;
const int BASE_SPEED = 1300;
const int THRESHOLD = 0;
const int COOL_DOWN = 0;

Logger logger;
WifiManager wifiMgr;

HallSensor hallSensor(logger);
Scanner scanner(logger);
Claw claw(logger);

enum State {
  PRE_GATE,
  DRIVE, // line following and scanning until object is in range
  VERIFY, // checking if object is a wall
  RAMP,
  SEARCH, // searching for pet magnet
  GRAB, // grabbing pet
  ESCAPE, // time up, escape
  STOP // stop
};

State currentState = PRE_GATE;
int petAngle = -1;
int petDistance = 9999;
unsigned long lastSeenPetTime = 0;
 
int petCount = 0;

void setup() {
  Serial.begin(115200);

  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }

    wifiMgr.startWifi();
  logger.begin(); // Launch logger task (on second core!)
  initializeDrive();
  claw.initialize();
  delay(100);

  currentState = PRE_GATE;
  petCount = 0;
  lastSeenPetTime = millis();
}


void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
}

void loop() {
  switch (currentState) {
    case PRE_GATE:
      logger.log("[State] PRE_GATE");
      logger.log("[PetCount] " + String(petCount));
      
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.completedScan()) {
        PolarPoint closestObject = scanner.getClosestObject();
        if (closestObject.distance <= IN_RANGE_DISTANCE) {
          currentState = DRIVE;
        }
      }
      scanner.scanOneStep(0);
      break;

    case DRIVE:
      logger.log("[State] DRIVE");
      logger.log("[PetCount] " + String(petCount));

      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.completedScan()) {
        PolarPoint closestObject = scanner.getClosestObject();
        if (closestObject.distance <= IN_RANGE_DISTANCE) {
          if ((millis() - lastSeenPetTime) >= COOL_DOWN) {
            stopMotors();
            delay(100); // make sure wheels are stopped
            currentState = VERIFY;
            if (petCount == 1) {
              currentState = RAMP;
            }
          }
        }
      }
      scanner.scanOneStep(0);
      break;
    case VERIFY:
      logger.log("[State] VERIFY");
      logger.log("[PetCount] " + String(petCount));
      if (scanner.completedScan()) {
        PolarPoint pet = scanner.honeIn(scanner.getClosestObject().angle);
        if (pet.distance <= IN_RANGE_DISTANCE) {
          scanner.setServoAngle(pet.angle);
          stopMotors();
          delay(3000);
          petCount++;
          lastSeenPetTime = millis();
        }
        currentState = DRIVE;
      }
      scanner.scanOneStep(100);
      break;
    case RAMP:
      logger.log("[State] RAMP");
      logger.log("[PetCount] Pet Count : " + String(petCount));

      lineFollow(2 * BASE_SPEED, 50, logger);
      if (scanner.completedScan()) {
        PolarPoint closestObject = scanner.getClosestObject();
        if (closestObject.distance <= IN_RANGE_DISTANCE && closestObject.angle > 90) {
          stopMotors();
          delay(100); // make sure wheels are stopped
          currentState = VERIFY;
        }
      }
      scanner.scanOneStep(0);
      break;
    case SEARCH:
      logger.log("[State] SEARCH");
      logger.log("[PetCount] " + String(petCount));

      break;
    case GRAB:
      logger.log("[State] GRAB");
      logger.log("[PetCount] " + String(petCount));

      break;
    case ESCAPE:
      logger.log("[State] ESCAPE");
      logger.log("[PetCount]" + String(petCount));
      break;
  }
}
