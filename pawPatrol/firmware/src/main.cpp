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
#include "lift.h"
#include "claw.h"
#include "utils.h"

#define COUNT 10

const int IN_RANGE_DISTANCE = 350;
const int IN_RANGE_DISTANCE_4 = 350;
const int IN_RANGE_DISTANCE_5 = 400;
const int IN_RANGE_DISTANCE_6 = 550;


const int REACHABLE_DISTANCE = 300;
const int REACHABLE_DISTANCE_4 = 350;
const int REACHABLE_DISTANCE_5 = 400;
const int REACHABLE_DISTANCE_6 = 550;

const int MAX_ANGLE_4 = 170;
const int MAX_ANGLE_5 = 170;
const int MAX_ANGLE_6 = 100;

const int MIN_ANGLE_4 = 90;
const int MIN_ANGLE_5 = 90;
const int MIN_ANGLE_6 = 10;

const int BASE_SPEED = 1100;
const int RAMP_SPEED = 1300;
const int SLOW_SPEED = 800;
const int FAST_SPEED = 1200;
const int THRESHOLD = 60;
const int COOL_DOWN = 2500;

int inRange = REACHABLE_DISTANCE;
int reachable = REACHABLE_DISTANCE;

volatile bool switchPressed1 = false;
volatile bool switchPressed2 = false;

Logger logger;
WifiManager wifiMgr;
Lift lift(FWD_LIFT_PIN, BWD_LIFT_PIN);
Scanner scanner;
Claw claw;
PolarPoint pet;

enum State
{
  PRE_GATE,
  DRIVE, // line following and scanning until object is in range
  APPROACH,
  VERIFY,  // checking if object is a wall
  GET_PET, // searching for pet magnet
  RAMP,
  RAISE_LIFT, // Raise lift for pet basket
  ESCAPE,     // time up, escape
  STOP        // stop
};

int petCount = 0;
int secCount = 0;
unsigned long cutOffTime = 0;
State currentState = PRE_GATE;
unsigned long lastSeenPetTime = 0;

void IRAM_ATTR onSwitchPress()
{
  switchPressed1 = true;
}
void IRAM_ATTR onSwitchPress2()
{
  switchPressed2 = true;
}

void setup()
{
  Serial.begin(115200);
  gpio_install_isr_service(0);

  while (!scanner.initialize())
  {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");

  attachInterrupt(digitalPinToInterrupt(LIFT_LIMIT_PIN), onSwitchPress, RISING);
  attachInterrupt(digitalPinToInterrupt(CLAW_LIMIT_PIN), onSwitchPress2, RISING);

  // wifiMgr.startWifi();
  // logger.begin();
  initializeDrive();
  claw.initialize();
  lift.setSpeed(1600);

  // currentState = RAISE_LIFT;
  //petCount = 0;
  lastSeenPetTime = millis();
  cutOffTime = millis() + 90000;
  pet.angle = -1;
  pet.distance = 9999;
  delay(1000);
}

void resetAll()
{
  scanner.reset();
  claw.moveToIdlePos();
  pet.angle = -1;
  pet.distance = 9999;
  delay(200);
}

void loop()
{
  switch (currentState)
  {
  case PRE_GATE:
    logger.log("[State] PRE_GATE");
    logger.log("[PetCount] " + String(petCount));

    scanner.setServoAngle(10);
    lineFollow(FAST_SPEED, THRESHOLD, logger);
    if ((scanner.readDistance() <= 200))
    {
      currentState = DRIVE;
      lastSeenPetTime = millis() - (COOL_DOWN - 500);
    }
    break;
  case DRIVE:
    logger.log("[State] DRIVE");
    logger.log("[PetCount] " + String(petCount));
    
    //Serial.println("[State] DRIVE");
    //Serial.println("[PetCount] " + String(petCount));  

    if (petCount == 0){
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE, 100, 10))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
    } else if (petCount == 1){
      lineFollow(RAMP_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE, 180, 90))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
    } else if (petCount == 2){
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE, 180, 90))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }

    } else if (petCount == 3) {
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE_4, MAX_ANGLE_4, MIN_ANGLE_4))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
    } else if (petCount == 4 ){
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE_5, MAX_ANGLE_5, MIN_ANGLE_5))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
     } else if ( petCount == 5){
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE_6, MAX_ANGLE_6, MIN_ANGLE_6))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
     } else {
      lineFollow(BASE_SPEED, THRESHOLD, logger);
      if (scanner.scanOneStep(IN_RANGE_DISTANCE))
      {
        if ((millis() - lastSeenPetTime) >= COOL_DOWN)
        {
          stopMotors();
          delay(500); // make sure wheels are stopped
          currentState = APPROACH;
        }
      }
    }
    break;

  case APPROACH:

    logger.log("[State] APPROACH");
    logger.log("[PetCount] " + String(petCount));

    //Serial.println("[State] APPROACH");
    //Serial.println("[PetCount] " + String(petCount));

    while (true) {

      if (petCount == 0){
        Serial.println("distance:"+ pet.distance);
        pet = scanner.getClosestObject(10, 100);
        if (pet.distance > IN_RANGE_DISTANCE) {
          currentState = DRIVE;
          break;
        }
        if ((1.0 * pet.distance * sin(radians(pet.angle))) < 127) {
          currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= 400){
          lineFollow(SLOW_SPEED, THRESHOLD, logger);
        }
        stopMotors();
        delay(500);
      }

      if (petCount == 2 || petCount == 3 || petCount == 4 || petCount == 5)
      {
        Serial.println("distance:"+ pet.distance);
        pet = scanner.getClosestObject(90, 180);
        if (pet.distance > IN_RANGE_DISTANCE_4) {
          currentState = DRIVE;
          break;
        }
        if ((1.0 * pet.distance * sin(radians(pet.angle))) < 127) {
          currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= 300){
          lineFollow(SLOW_SPEED, THRESHOLD, logger);
        }
        stopMotors();
        delay(500);
      }
      else
      {
        pet = scanner.getClosestObject(0, 180);
        if (pet.distance > IN_RANGE_DISTANCE)
        {
          currentState = DRIVE;
          break;
        }
        if ((1.0 * pet.distance * sin(radians(pet.angle))) < 101.6)
        {
          currentState = VERIFY;
          break;
        }
        unsigned long start = millis();
        while ((millis() - start) <= 500)
        {
          lineFollow(SLOW_SPEED, THRESHOLD, logger);
        }
        stopMotors();
        delay(500);
      }
    }

  case VERIFY:
  {

    logger.log("[State] DRIVE");
    logger.log("[PetCount] " + String(petCount));

    
    Serial.println("[State] DRIVE");
    Serial.println("[PetCount] " + String(petCount));


    if (petCount == 3)
    {
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > IN_RANGE_DISTANCE_4)
      {
        currentState = DRIVE;
        break;
      }
      if (pet.distance > REACHABLE_DISTANCE_4)
      {
        currentState = APPROACH;
        break;
      }
    } else if (petCount == 4)
    {
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > IN_RANGE_DISTANCE_5)
      {
        currentState = DRIVE;
        break;
      }
      if (pet.distance > REACHABLE_DISTANCE_5)
      {
        currentState = APPROACH;
        break;
      }
    } else if (petCount == 5) {
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > IN_RANGE_DISTANCE_6)
      {
        currentState = DRIVE;
        break;
      }
      if (pet.distance > REACHABLE_DISTANCE_6)
      {
        currentState = APPROACH;
        break;
      }
    }

    else
    {
      pet = scanner.honeIn(pet.angle);
      if (pet.distance > IN_RANGE_DISTANCE)
      {
        currentState = DRIVE;
        break;
      }
      if (pet.distance > REACHABLE_DISTANCE)
      {
        currentState = APPROACH;
        break;
      } 
    }

    scanner.setServoAngle(pet.angle);
      stopMotors();
      delay(500);
      scanner.setServoAngle(150);
      currentState = GET_PET;
      break;
  }
  case GET_PET:
  {
    logger.log("[State] GET_PET");
    logger.log("[PetCount] " + String(petCount));

    logger.log("[Laser] Pet Detected: " + String(pet.angle) + "degrees at " + String(pet.distance) + " mm away.");
    PolarPoint offsetPet = calculateOffset(pet); // pet coords relative to claw

    if (claw.searchPet(offsetPet.angle, offsetPet.distance + 30, petCount + 1))
    {
      petCount++;
      if (petCount == 1)
      {
        scanner.setServoAngle(10);
        while (scanner.readDistance() > 300)
        {
          lineFollow(BASE_SPEED, THRESHOLD, logger);
          delay(10);
        }
        unsigned long startRamp = millis();
        while ((millis() - startRamp) <= 4000)
        {
          lineFollow(BASE_SPEED, THRESHOLD, logger);
          delay(10);
        }
        stopMotors();
        claw.rampToss();
        currentState = RAMP;
      }
      else if (petCount == 2)
      {
        scanner.setServoAngle(150);
        delay(1000);
        claw.windowToss();
        lastSeenPetTime = millis();
        currentState = DRIVE;
      }
      else
      {
        scanner.setServoAngle(150);
        delay(100);
        claw.dump();
        lastSeenPetTime = millis();
        currentState = DRIVE;
      }
    }
    else
    {
      currentState = DRIVE;
    }
    if (petCount == 6){
      currentState = ESCAPE;
    }

    if (petCount == 7)
      currentState = ESCAPE;
    resetAll();
    break;
  }

  case RAMP:

    logger.log("[State] RAMP");
    logger.log("[PetCount] " + String(petCount));

    scanner.setServoAngle(180);
    lineFollow(RAMP_SPEED, THRESHOLD, logger);
    if ((scanner.readDistance() <= 400))
    {
      while (scanner.readDistance() <= 400)
      {
        lineFollow(RAMP_SPEED, THRESHOLD, logger);
      }
      stopMotors();
      delay(500); // make sure wheels are stopped
      currentState = VERIFY;
    }

  case RAISE_LIFT:

    // GET IN POSITION

    lift.raise(1);
    if (switchPressed1)
    {
      switchPressed1 = false;
      Serial.println("FIRST SWITCH PRESSED");
      lift.stop();
      delay(20000);
      lift.lower(3000);
      lift.stop();
      currentState = ESCAPE;
    }

    break;

  case ESCAPE:

    if (switchPressed2)
    {
      switchPressed2 = false;
      Serial.println("SECOND PRESSED - LIFT RAISING");
      currentState = RAISE_LIFT;
    }
    else
    {
      Serial.println("SECOND UNPRESSED");
    }

    // drive forward and escape
    break;
  }
}
