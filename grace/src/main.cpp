#include <Arduino.h>
#include "drive.h"
#include "detect.h"

void setup() {
  initialize_drive();
  Serial.begin(9600);
}

void loop() {
  drive();
  // // Then handle object detection
  // if (scanForPets()) {
  //   // We found an object within MIN_OBJECT_DISTANCE
  //   ScanResult result = getLastScanResult();
    
  //   if (result.valid && result.distance < 300) {  // Same threshold as ref.cpp
  //     Serial.print("Found object at angle: ");
  //     Serial.print(result.angle);
  //     Serial.print(" distance: ");
  //     Serial.println(result.distance);
      
  //     // Stop and try to align with the object
  //     setLeftMotor(0);
  //     setRightMotor(0);
      
  //     if (result.distance < 150) {  // Same threshold as ref.cpp for pickup
  //       if (alignWithMagnet()) {
  //         // Object is aligned, could add pickup logic here
  //         Serial.println("Object aligned!");
  //       }
  //     }
  //   }
  // }
}
