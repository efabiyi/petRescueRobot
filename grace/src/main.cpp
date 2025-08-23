#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"
#include "claw.h"
#include "pins.h"
#include "lift.h"

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
  DRIVE, // line following and scanning until object is in range
  APPROACH,
  VERIFY, // checking if object is a wall
  GET_PET, // searching for pet magnet
  RAMP,
  ESCAPE, // time up, escape
  LIFT,
  STOP, // stop
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

void IRAM_ATTR onSwitchPress() {
  switchPressed = true;
}

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
  // cutOffTime = millis() + 90000;
  // endTime = millis() + 110000;
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

void handlePet() {
  scanner.setServoAngle(150);
  if (petCount == 1) {
    unsigned long startRamp = millis();
    while ((millis() - startRamp) <= 7000) lineFollow(BASE_SPEED, 200, KP);
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
  // further handling
  if (petCount == 6) {
    resetAll();
    unsigned long start = millis();
    while ((millis() - start) <= 1000) {
      lineFollow(SLOW_SPEED, THRESHOLD, KP);
    }
    // ----------------------
    drive(-1 * BASE_SPEED, -1 * BASE_SPEED);
    delay(2700);
    uTurnRight(200);
    currentState = ESCAPE;
    lastSeenPetTime = millis();
    return;
    // ----------------------
    // drive(900, 1100);
    // delay(3000);
    // searchLine(200);
    // currentState = DRIVE;
    // lastSeenPetTime = millis();
    // break;
  // } else if (petCount == 4 || petCount == 5) {
  //   searchLine(200);
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

void loop() {
  switch (currentState) {
    case PRE_GATE:
      scanner.setServoAngle(30);
      lineFollow(FAST_SPEED, THRESHOLD, KP);
      if ((scanner.readDistance() < 300)) {
        currentState = DRIVE;
        lastSeenPetTime = millis();
      }
      break;
    case DRIVE:
      scanner.setServoAngle(petAngle[petCount]);
      lineFollow(speed, THRESHOLD, KP);
      if ((millis() - lastSeenPetTime) >= cooldown[petCount]) {
        if (scanner.readDistance() <= inRange[petCount]) {
          stopMotors();
          delay(500); // make sure wheels are stopped
          // if (petCount != 3) {
            currentState = APPROACH;
          // } else {
          //   petCount = 4;
          // }
        }
      }
      break;
    case APPROACH:
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
          // if (petCount == 3) currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= (1.0 * pet.distance * sin(radians(pet.angle)) + 300)) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP);
        }
        stopMotors();
        delay(500);
        scootCount++;
      }
      break;
    case VERIFY: {
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
    }
    case GET_PET: {
      PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw
      if ((petCount != 5 && offsetPet.distance > 400) || (petCount == 5 && offsetPet.distance > 480)) {
        unsigned long start = millis();
        while ((millis() - start) <= (1.0 * pet.distance * sin(radians(pet.angle)) + 300)) {
          lineFollow(SLOW_SPEED, THRESHOLD, KP);
        }
        stopMotors();
        delay(500);
        scootCount++;
        currentState = APPROACH;
        break;
      }
      claw.grabPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1);
      petCount++;
      handlePet();
      break;
    }
    case RAMP:
      scanner.setServoAngle(180);
      lineFollow(BASE_SPEED, THRESHOLD, 0.7f);
      if ((millis() - lastSeenPetTime) > cooldown[petCount]) {
        if (scanner.readDistance() <= 400) {
          while (scanner.readDistance() <= 400) lineFollow(SLOW_SPEED, THRESHOLD, 0.7f);
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
      break;
    case ESCAPE:
      scanner.setServoAngle(170);
      lineFollow(BASE_SPEED, 200, KP);
      if ((millis() - lastSeenPetTime) > 12000) {
        if (scanner.readDistance() <= 250) {
          // unsigned long start = millis();
          // while ((millis() - start) <= 8000) {
          //   drive(1000, -500);
          //   delay(500);
          //   drive(-500, 1000);
          //   delay(500);
          // }
          drive(-500, -1000);
          delay(1500);
          int l = analogRead(LEFT_SENSOR);
          int r = analogRead(RIGHT_SENSOR);
          while (l < 200 && r < 200) {
            drive(-800, 800);
            l = analogRead(LEFT_SENSOR);
            r = analogRead(RIGHT_SENSOR);
          }
          stopMotors();
          delay(500);
          unsigned long start = millis();
          while ((millis() - start) <= 5000) lineFollow(SLOW_SPEED, THRESHOLD, KP);
          drive(2000, -2000);
          delay(5000);
          resetAll();
          currentState = CELEBRATE;
        }
      }
      break;
    case LIFT:
      lift.raise();
      if (switchPressed) {
        switchPressed = false;
        lift.stop(); 
        drive(1100, 900);
        delay(1000);
        stopMotors();
        lift.lower();
        delay(3000);
        lift.stop();
        currentState = STOP;
      }
      break;
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
