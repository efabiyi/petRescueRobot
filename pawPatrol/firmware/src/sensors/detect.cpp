// #include "detect.h"
// #include <Arduino.h>
// #include <Adafruit_VL53L0X.h>
// #include <ESP32Servo.h>
// #include "sensors.h"

// Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// Servo scannerServo;

// const int SCAN_START_ANGLE = 50;
// const int SCAN_END_ANGLE = 130;
// const int STEP_SIZE = 10;
// const int TOTAL_SCAN_POINTS = (SCAN_END_ANGLE - SCAN_START_ANGLE) / STEP_SIZE + 1;
// const int MIN_OBJECT_DISTANCE = 300;  // Distance in mm to consider an item in range
// const int REACHABLE_DISTANCE = 95;    // Distance in mm to consider an item reachable
// const int MAX_DISTANCE = 9999;
// const int SCAN_INTERVAL = 30;

// static ScanResult scanData[15];
// static int currentScanAngle = SCAN_START_ANGLE;
// static int scanDirection = 1;
// static unsigned long lastScanTime = 0;

// void initiateDetectionSystem() {
//     Serial.println("Initializing VL53L0X sensor");
//     if (!lox.begin()) {
//         Serial.println(F("Failed to boot VL53L0X"));
//         while(1);
//     }
    
//     scannerServo.attach(27);  // Adjust pin number as needed
//     scannerServo.write(SCAN_START_ANGLE);
    
//     clearScanData();
// }

// void clearScanData() {
//     for(int i = 0; i < TOTAL_SCAN_POINTS; i++) {
//         scanData[i].angle = SCAN_START_ANGLE + (i * STEP_SIZE);
//         scanData[i].distance = MAX_DISTANCE;
//         scanData[i].valid = false;
//     }
// }

// int readDistance() {
//     VL53L0X_RangingMeasurementData_t measure;
//     lox.rangingTest(&measure, false);

//     if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//         return measure.RangeMilliMeter;
//     } else {
//         return MAX_DISTANCE;
//     }
// }

// void saveScanData(int angle, int distance, int position) {
//     if (position >= 0 && position < TOTAL_SCAN_POINTS) {
//         scanData[position].angle = angle;
//         scanData[position].distance = distance;
//         scanData[position].valid = true;
//     }
// }

// ScanResult getClosestObject() {
//     int closestDistance = MAX_DISTANCE;
//     ScanResult closest = {90, MAX_DISTANCE, false};  // Default values

//     for(int i = 0; i < TOTAL_SCAN_POINTS; i++) {
//         if (scanData[i].valid && scanData[i].distance > 0 && scanData[i].distance < closestDistance) {
//             closest = scanData[i];
//             closestDistance = scanData[i].distance;
//         }
//     }

//     return closest;
// }

// ScanResult adjustScannerPosition(int targetAngle, int scanStep, int scanRange) {
//     int startAngle = targetAngle - (scanRange * scanStep);
//     int endAngle = targetAngle + (scanRange * scanStep);
    
//     clearScanData();
    
//     // Move scanner to center position
//     scannerServo.write(90);
//     delay(SCAN_INTERVAL * 2);
    
//     // Perform fine scan
//     for(int angle = startAngle; angle <= endAngle; angle += scanStep) {
//         scannerServo.write(angle);
//         delay(SCAN_INTERVAL);
        
//         int distance = readDistance();
//         int position = (angle - startAngle) / scanStep;
//         saveScanData(angle, distance, position);
//     }
    
//     return getClosestObject();
// }

// bool scanForPets() {
//     unsigned long currentTime = millis();
//     if (currentTime - lastScanTime < SCAN_INTERVAL) {
//         return false;
//     }
//     lastScanTime = currentTime;

//     // Update scanner position
//     currentScanAngle += (STEP_SIZE * scanDirection);
//     scannerServo.write(currentScanAngle);
    
//     // Check if we need to change direction
//     if (currentScanAngle >= SCAN_END_ANGLE || currentScanAngle <= SCAN_START_ANGLE) {
//         scanDirection *= -1;
        
//         // Get closest object after completing a sweep
//         ScanResult closest = getClosestObject();
        
//         if (closest.valid && closest.distance < MIN_OBJECT_DISTANCE) {
//             return true;  // Pet detected within range
//         }
        
//         clearScanData();  // Reset scan data for next sweep
//     }
    
//     // Save current scan data
//     int position = (currentScanAngle - SCAN_START_ANGLE) / STEP_SIZE;
//     int distance = readDistance();
//     saveScanData(currentScanAngle, distance, position);
    
//     return false;
// }
