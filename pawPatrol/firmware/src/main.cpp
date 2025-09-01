#include <WiFi.h>
#include <Arduino.h>

#include "secrets.h"
#include "driver/ledc.h"
#include "wifiManager.h"
#include "logger.h"

#include "drive.h"
#include "scanner.h"
#include "claw.h"
#include "lift.h"
#include "constants.h"

const int BASE_SPEED = 1100;
const int SLOW_SPEED = 800;
const int FAST_SPEED = 1400;
const int THRESHOLD = 100;
const float KP = 0.6f;

int petAngle[] = {15, 135, 175, 175, 170, 0, 180};
int inRange[] = {350, 350, 300, 450, 450, 450, 400};
int cooldown[] = {2000, 2000, 2000, 2000, 2000, 1000, 1000};
int minAngle[] = {0, 90, 90, 90, 90, 0, 90};
int maxAngle[] = {90, 180, 180, 180, 180, 90, 180};

Scanner scanner;
Claw claw;
Lift lift;

PolarPoint pet;

enum State {
  PRE_GATE,
  DRIVE,
  APPROACH,
  VERIFY,
  GET_PET,
  RAMP,
  ESCAPE,
  LIFT,
  STOP,
  CELEBRATE
};

State currentState = PRE_GATE;
unsigned long lastSeenPetTime = 0;
int petCount = 0;
unsigned long cutOffTime = 0;
unsigned long endTime = 0;
volatile bool switchPressed = false;
int scootCount = 0;

int speed = BASE_SPEED;

WifiManager wifiMgr;
Logger logger;
bool loggingEnabled = false;

void IRAM_ATTR onSwitchPress() {
  switchPressed = true;
}

void setup() {
  Serial.begin(115200);

  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");

  attachInterrupt(digitalPinToInterrupt(LIFT_LIMIT_PIN), onSwitchPress, RISING);

  initializeDrive();
  claw.initialize();
  lift.setSpeed(1600);

  // Attempt WiFi and logger
  wifiMgr.startWifi();
  if (wifiMgr.isConnected()) {
    logger.begin();
    loggingEnabled = true;
    logger.log("WiFi connected. Logging enabled.");
  }

  lastSeenPetTime = millis();
  pet.angle = -1;
  pet.distance = 9999;

  delay(1000);
}

void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
  pet.angle = -1;
  pet.distance = 9999;
}

void handlePet() {
  scanner.setServoAngle(150);
  if (petCount == 1) {
    unsigned long startRamp = millis();
    while ((millis() - startRamp) <= 7000) lineFollow(BASE_SPEED, 200, KP, loggingEnabled ? &logger : nullptr);
    stopMotors();
    delay(200);
    claw.rampToss();
    currentState = RAMP;
    return;
  } else if (petCount == 2) {
    claw.windowToss();
    speed = 750;
    currentState = DRIVE;
  } else {
    if (petCount == 5) claw.dump();
    else claw.dunk();
    currentState = DRIVE;
  }

  if (petCount == 6) {
    resetAll();
    unsigned long start = millis();
    while ((millis() - start) <= 1000) {
      lineFollow(SLOW_SPEED, THRESHOLD, KP, loggingEnabled ? &logger : nullptr);
    }
    drive(-1 * BASE_SPEED, -1 * BASE_SPEED);
    delay(2700);
    uTurnRight(200);
    currentState = ESCAPE;
    lastSeenPetTime = millis();
    return;
  } else if (petCount == 7) {
    drive(-1000, -1000);
    delay(1000);
    stopMotors();
    delay(500);
    currentState = LIFT;
  }

  lastSeenPetTime = millis();
  resetAll();
}

void logState(State s) {
  if (!loggingEnabled) return;
  String stateStr;
  switch (s) {
    case PRE_GATE: stateStr = "PRE_GATE"; break;
    case DRIVE: stateStr = "DRIVE"; break;
    case APPROACH: stateStr = "APPROACH"; break;
    case VERIFY: stateStr = "VERIFY"; break;
    case GET_PET: stateStr = "GET_PET"; break;
    case RAMP: stateStr = "RAMP"; break;
    case ESCAPE: stateStr = "ESCAPE"; break;
    case LIFT: stateStr = "LIFT"; break;
    case STOP: stateStr = "STOP"; break;
    case CELEBRATE: stateStr = "CELEBRATE"; break;
    default: stateStr = "UNKNOWN"; break;
  }
  logger.log("[State] " + stateStr + " [PetCount] " + String(petCount));
}

void loop() {
  switch (currentState) {
    case PRE_GATE:
      logState(PRE_GATE);
      scanner.setServoAngle(30);
      lineFollow(FAST_SPEED, THRESHOLD, KP, loggingEnabled ? &logger : nullptr);
      if ((scanner.readDistance() < 300)) {
        currentState = DRIVE;
        lastSeenPetTime = millis();
      }
      break;

    case DRIVE:
      logState(DRIVE);
      scanner.setServoAngle(petAngle[petCount]);
      lineFollow(speed, THRESHOLD, KP, loggingEnabled ? &logger : nullptr);
      if ((millis() - lastSeenPetTime) >= cooldown[petCount]) {
        if (scanner.readDistance() <= inRange[petCount]) {
          stopMotors();
          delay(500);
          currentState = APPROACH;
        }
      }
      break;

    case APPROACH:
      logState(APPROACH);
      while (true) {
        if (scootCount > 3) {
          scootCount = 0;
          petCount++;
          handlePet();
          break;
        }
        pet = scanner.getClosestObject(minAngle[petCount], maxAngle[petCount]);
        if (pet.distance > inRange[petCount]) {
          currentState = DRIVE;
          break;
        }
        if (((1.0 * pet.distance * sin(radians(pet.angle))) < 127.0)) {
          stopMotors();
          delay(500);
          scanner.setServoAngle(150);
          currentState = GET_PET;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= (1.0 * pet.distance * sin(radians(pet.angle)) + 300)) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP, loggingEnabled ? &logger : nullptr);
        }
        stopMotors();
        delay(500);
        scootCount++;
      }
      break;

    case VERIFY:
      logState(VERIFY);
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > inRange[petCount]) {
        currentState = DRIVE;
        break;
      }
      if ((1.0 * pet.distance * sin(radians(pet.angle))) >= 127.0) {
        currentState = APPROACH;
        break;
      }
      stopMotors();
      delay(500);
      scanner.setServoAngle(150);
      currentState = GET_PET;
      break;

    case GET_PET:
      logState(GET_PET);
      PolarPoint offsetPet = calculateOffset(pet); // relative to claw
      claw.grabPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1);
      petCount++;
      handlePet();
      break;

    case RAMP:
      logState(RAMP);
      scanner.setServoAngle(180);
      lineFollow(BASE_SPEED, THRESHOLD, 0.7f, loggingEnabled ? &logger : nullptr);
      if ((millis() - lastSeenPetTime) > cooldown[petCount]) {
        if (scanner.readDistance() <= 400) {
          while (scanner.readDistance() <= 400)
            lineFollow(SLOW_SPEED, THRESHOLD, 0.7f, loggingEnabled ? &logger : nullptr);
          stopMotors();
          delay(500);
          currentState = APPROACH;
        }
      }
      break;

    case ESCAPE:
      logState(ESCAPE);
      scanner.setServoAngle(170);
      lineFollow(BASE_SPEED, 200, KP, loggingEnabled ? &logger : nullptr);
      break;

    case LIFT:
      logState(LIFT);
      logger.log("[PetCount] " + String(petCount)); 
      
      lift.raise(1);
      if (switchPressed) { 
      switchPressed = false;
      Serial.println("FIRST SWITCH PRESSED");
      lift.stop();
      delay(3500);
      lift.lower(4000);
      lift.stop();
      currentState = ESCAPE; }

    case STOP:
      stopMotors();
      delay(50);
      break;

    case CELEBRATE:
      stopMotors();
      claw.setElbowServo(120);
      delay(1000);
      claw.setElbowServo(90);
      delay(1000);
      break;
  }
}
