#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"
#include "lift.h"

#include "logger.h"
#include "wifiManager.h"

const int BASE_SPEED = 1000;
const int SLOW_SPEED = 800;
const int FAST_SPEED = 1500;
const int THRESHOLD = 100;
const float KP = 0.7f;

int petAngle[] = {15, 135, 175, 175, 175, 0};
int inRange[] = {300, 300, 250, 400, 350, 480};
int cooldown[] = {2000, 1000, 0, 3000, 6000, 3000}; // 3rd delay is hard-coded slow driving
int minAngle[] = {0, 90, 90, 90, 90, 0};
int maxAngle[] = {90, 180, 180, 180, 180, 90};

Scanner scanner;
Claw claw;
Lift lift;

PolarPoint pet;
Logger logger;
WifiManager wifiMgr;

enum State {
  PRE_GATE,
  DRIVE, // line following and scanning until object is in range
  APPROACH,
  VERIFY, // checking if object is a wall
  GET_PET, // searching for pet magnet
  RAMP,
  ESCAPE, // time up, escape
  LIFT,
  STOP // stop
};

String stateToString(State s) {
  switch (s) {
    case PRE_GATE:        return "PRE_GATE";
    case DRIVE:           return "DRIVE";
    case APPROACH:        return "APPROACH";
    case VERIFY:          return "VERIFY";
    case GET_PET:         return "GET_PET";
    case RAMP:            return "RAMP";
    case LIFT:            return "LIFT";
    case ESCAPE:          return "ESCAPE";
    case STOP:            return "STOP";
    default:              return "UNKNOWN_STATE";
  }
}

State currentState = STOP;
State lastLoggedState = ESCAPE;
unsigned long lastSeenPetTime = 0;
int lastPetCount = -1;
int petCount = 0;
unsigned long cutOffTime = 0;
unsigned long endTime = 0;
volatile bool switchPressed = false;
int speed = BASE_SPEED;

void IRAM_ATTR onSwitchPress() {
  switchPressed = true;
}

void logState(State newState) {
  if (newState != lastLoggedState) {
    logger.log("[State] " + stateToString(newState));
    lastLoggedState = newState;
  }
}
void logPets(int newCount) {
  if (newCount != lastPetCount) {
    logger.log("[PetCount] " + String(newCount));
    lastPetCount = newCount;
  }
}

void setup() {
  Serial.begin(9600);
  /*while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");*/
  
  wifiMgr.startWifi();
  logger.begin();
  initializeDrive();
  claw.initialize();
  lastSeenPetTime = millis();
  // cutOffTime = millis() + 90000;
  // endTime = millis() + 120000;
  pet.angle = -1;
  pet.distance = 9999;
  attachInterrupt(digitalPinToInterrupt(LIFT_LIMIT_PIN), onSwitchPress, RISING);
  delay(1000);
}

void resetAll() {
  scanner.reset();
  claw.moveToIdlePos();
  pet.angle = -1;
  pet.distance = 9999;
}

void loop() {
  // if (millis() >= cutOffTime && currentState != ESCAPE) {
  //   // resetAll();
  //   // claw.setElbowServo(120);
  //   // delay(100);
  //   // uturnRight(THRESHOLD);
  //   // currentState = ESCAPE;
  //   stopMotors();
  //   return;
  // }
  // if (millis() >= endTime) {
  //   currentState = STOP;
  // }
  switch (currentState) {
    case PRE_GATE:
      
    logState(PRE_GATE);
    logPets(petCount);

      scanner.setServoAngle(30);
      lineFollow(FAST_SPEED, THRESHOLD, KP, logger);
      if ((scanner.readDistance() < 250)) {
        currentState = DRIVE;
        lastSeenPetTime = millis();
      }
      break;
    case DRIVE:

    logState(DRIVE);
    logPets(petCount);     
      
      scanner.setServoAngle(petAngle[petCount]);
      lineFollow(speed, THRESHOLD, KP, logger);
        if ((millis() - lastSeenPetTime) >= cooldown[petCount]) {
        if (scanner.readDistance() <= inRange[petCount]) {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
      break;
    case APPROACH:

    logState(APPROACH);
    logPets(petCount);
      
      while (true) {
        pet = scanner.getClosestObject(minAngle[petCount], maxAngle[petCount]);
        if (pet.distance > inRange[petCount]) {
          currentState = DRIVE;
          break;
        }
        if (((1.0 * pet.distance * sin(radians(pet.angle))) < 127.0) && pet.distance <= 400) {
          stopMotors();
          delay(500);
          scanner.setServoAngle(150);
          currentState = GET_PET;
          if (petCount == 3) currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= (1.0 * pet.distance * sin(radians(pet.angle)) + 200)) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP, logger);
        }
        stopMotors();
        delay(500);
      }
      break;
    case VERIFY: {

    logState(VERIFY);
    logPets(petCount);

      pet = scanner.honeIn(pet.angle);
      if (pet.distance > inRange[petCount]) {
        currentState = DRIVE;
        break;
      }
      if ((1.0 * pet.distance * sin(radians(pet.angle))) >= 127.0 || pet.distance > 400) {
        currentState = APPROACH;
        break;
      }
      stopMotors();
      delay(500);
      scanner.setServoAngle(150);
      currentState = GET_PET;
      break;
    }
    case GET_PET: {

    logState(GET_PET);
    logPets(petCount);

      PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw
      claw.grabPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1);
      petCount++;
      if (petCount == 3) petCount = 4;
      // handling pet (ramp-dropping, window-dropping, dumping)
      if (petCount == 1) {
        unsigned long startRamp = millis();
        while ((millis() - startRamp) <= 8000) lineFollow(BASE_SPEED, 200, 0.8f, logger);
        stopMotors();
        scanner.setServoAngle(150);
        delay(200);
        claw.rampToss();
        currentState = RAMP;
        break;
      } else if (petCount == 2) {
        scanner.setServoAngle(150);
        claw.windowToss();
        speed = SLOW_SPEED;
        currentState = DRIVE;
      } else {
        scanner.setServoAngle(150);
        claw.dump();
        currentState = DRIVE;
      }
      // further handling
      if (petCount == 6) {
        resetAll();
        // ----------------------
        unsigned long start = millis();
        while ((millis() - start) <= 800) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP, logger);
        }
        drive(-1 * BASE_SPEED, -1 * BASE_SPEED);
        delay(2750);
        uTurnRight(200);
        currentState = ESCAPE;
        // ----------------------
        // drive(BASE_SPEED, BASE_SPEED);
        // delay(3000);
        // searchLine(200);
        // currentState = DRIVE;
        // ----------------------
        break;
      }
      lastSeenPetTime = millis();
      scanner.setServoAngle(petAngle[petCount]);
      resetAll();
      break;
    }
    case RAMP:
      
      logState(RAMP);
      logPets(0);

      scanner.setServoAngle(180);
      lineFollow(BASE_SPEED, THRESHOLD, KP, logger);
      if ((millis() - lastSeenPetTime) > cooldown[petCount]) {
        if (scanner.readDistance() <= 400) {
          while (scanner.readDistance() <= 400) lineFollow(SLOW_SPEED, THRESHOLD, KP, logger);
        stopMotors();
        delay(500); // make sure wheels are stopped
        currentState = APPROACH;
        }
      }
      break;
    case ESCAPE:
      
      logState(ESCAPE);
      logPets(petCount);
      
      lineFollow(BASE_SPEED, THRESHOLD, KP, logger);
      break;
    case LIFT:

      logState(LIFT);
      logPets(petCount);

      lift.raise(1);
      if (switchPressed) {
        switchPressed = false;
        lift.stop();
        delay(500);
        lift.lower(3000);
        lift.stop();
        currentState = STOP;
      }
      break;
    case STOP:

      logState(STOP);
      logPets(petCount);
      logger.log("[DRIVE] BLAH | BLAHBLAH");
      
      stopMotors();
      delay(50);
      break;
  }
}
