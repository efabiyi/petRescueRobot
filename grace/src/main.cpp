#include <Arduino.h>
#include "drive.h"
#include "driver/ledc.h"
#include "scanner.h"

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 100;

Scanner scanner;

bool objectNear = false;;

void setup() {
  Serial.begin(9600);
  while (!scanner.initialize()) {
    Serial.println("Failed to initialize scanner");
    delay(100);
  }
  Serial.println("Scanner initialized");
  initializeDrive();
  objectNear = false;
  delay(50);
}

void loop() {
  drive();
  if (scanner.completedScan()) {
    PolarPoint closestObject = scanner.getClosestObject();
    // Serial.println(closestObject.distance);
    if (closestObject.distance <= IN_RANGE_DISTANCE && closestObject.distance != 0) {
      // Serial.println("stopping motors and doing second scan");
      while (true) { // check again with wheels stopped
        leftDriveForward(0);
        rightDriveForward(0);
        scanner.scanOneStep();
        if (scanner.completedScan()) {
          break;
        }
      }
      // Serial.println("second scan complete");
      if (!scanner.closestObjectIsWall()) {
        // Serial.println("object detected");
        leftDriveForward(0);
        rightDriveForward(0);
        delay(3000);
      }
    }
  }
  scanner.scanOneStep();  
}
