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

Logger logger;
WifiManager wifiMgr;

HallSensor hallSensor(logger);
Drive driver(logger);
Scanner scanner(logger);
Claw claw(logger);

State currentState = DRIVE;
int petAngle = -1;
int petDistance = 9999;
unsigned long lastSeenPetTime = 0;


enum State {
  DRIVE, // line following and scanning until object is in range
  VERIFY, // checking if object is a wall
  SEARCH, // searching for pet magnet
  GRAB, // grabbing pet
  ESCAPE, // time up, escape
  STOP // stop
};

void setup() {
  Serial.begin(115200);

  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }

  wifiMgr.startWifi();
  logger.begin(); // Launch logger task (on second core!)
  driver.initializeDrive();\
  claw.initialize();
  delay(500);
}

void loop() {
  driver.drive();
  hallSensor.sense();
}


void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
}

void loop() {
  switch (currentState) {
    case DRIVE:
      logger.log("[State]" + currentState);

      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.completedScan()) {
        PolarPoint closestObject = scanner.getClosestObject();
        if (closestObject.distance <= IN_RANGE_DISTANCE && closestObject.distance >= REACHABLE_DISTANCE) {
            currentState = VERIFY;
            stopMotors();
            delay(100);
        }
      }
      scanner.scanOneStep(0);
      break;
    case VERIFY:
      logger.log("[State]" + currentState); 
      if (scanner.completedScan()) {
        if (!scanner.closestObjectIsWall()) {
          PolarPoint pet = scanner.honeIn(scanner.getClosestObject().angle);
          scanner.setServoAngle(pet.angle);
          stopMotors();
          delay(3000);
          currentState = DRIVE;
        } else {
          scanner.reset();
          currentState = DRIVE;
        }
      }
      scanner.scanOneStep(100);
      break;
    case SEARCH:
      logger.log("[State]" + currentState);
      break;
    case GRAB:
      logger.log("[State]" + currentState);
      break;
    case ESCAPE:
      logger.log("[State]" + currentState);
      break;
  }
}

