#include "scanner.h"
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include "pins.h"

Scanner::Scanner() {
    servoStep = -1 * SERVO_STEP_SIZE;
    servoAngle = 90;
}

bool Scanner::initialize() {
    ledcSetup(SERVO_CHANNEL, 50, 12);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
    Serial.println("Initializing VL53L0X sensor");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        return false;
    }
    reset();
    return true;
}

void Scanner::reset() {
    servoAngle = 90;
    servoStep = -1 * SERVO_STEP_SIZE;
    setServoAngle(servoAngle);
    clearScanData();
}

int Scanner::readDistance() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        return measure.RangeMilliMeter + 22;
        // return measure.RangeMilliMeter - 30;
    } else {
        return MAX_DISTANCE;
    }
}

// void Scanner::scanOneStep(int scanDelay) {
//     if (completedScan()) {
//         servoStep = -1 * servoStep;
//     }
//     servoAngle = servoAngle + servoStep;
//     setServoAngle(servoAngle);
//     delay(scanDelay);
//     int distance = readDistance();
//     // Serial.println(distance);
//     int index = (servoAngle - MIN_ANGLE) / SERVO_STEP_SIZE;
//     scanData[index].distance = distance;
//     scanData[index].angle = servoAngle;
// }

bool Scanner::scanOneStep(int threshold) {
    if (servoAngle >= MAX_ANGLE || servoAngle <= MIN_ANGLE) {
        servoStep = -1 * servoStep;
    }
    servoAngle = servoAngle + servoStep;
    setServoAngle(servoAngle);
    int distance = readDistance();
    return (distance <= threshold);
}

PolarPoint Scanner::honeIn(int angle) {
    int minAngle = max(angle - 3, MIN_ANGLE);
    int maxAngle = min(angle + 3, MAX_ANGLE);
    PolarPoint obj;
    obj.angle = -1;
    obj.distance = MAX_DISTANCE;
    if (angle < 0) return obj;
    for (int theta = minAngle; theta <= maxAngle; theta += 2) {
        setServoAngle(theta);
        delay(750);
        int distances[10];
        double sum = 0.0;
        for (int i = 0; i < 10; i++) {
            distances[i] = readDistance();
            sum += distances[i];
        }
        double distance = sum / 10.0;
        for (int i = 0; i < 10; i++) {
            // Serial.print(String(distances[i]) + ", ");
        }
        // Serial.println("");
        if (distance < obj.distance) {
            obj.angle = theta;
            obj.distance = distance;
        }
        // Serial.println("angle: " + String(theta) + " | distance: " + String(distance));
    }
    return obj;
}

void Scanner::setServoAngle(int angle) {
    ledcWrite(SERVO_CHANNEL, angleToDutyScanner(angle));
}

PolarPoint Scanner::getClosestObject(int minAngle, int maxAngle) {
    PolarPoint closestObject;
    closestObject.distance = MAX_DISTANCE;
    closestObject.angle = -1;
    setServoAngle(minAngle);
    delay(500);
    for (int i = minAngle; i <= maxAngle; i += 10) {
        setServoAngle(i);
        delay(100);
        int distance = readDistance();
        if (distance < closestObject.distance) {
            closestObject.distance = distance;
            closestObject.angle = i;
        }
    }
    return closestObject;
}

void Scanner::clearScanData() {
    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        scanData[i].distance = MAX_DISTANCE;
        scanData[i].angle = MIN_ANGLE + i * SERVO_STEP_SIZE;
    }
}

void Scanner::printScanData() {
    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        Serial.println("Angle: " + String(scanData[i].angle) + " Distance: " + String(scanData[i].distance));
    }

    CartesianPoint cartesianData[SCAN_DATA_SIZE];

    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        float radiansVal = radians(scanData[i].angle);
        cartesianData[i].x = (scanData[i].distance) * cos(radiansVal);
        cartesianData[i].y = (scanData[i].distance) * sin(radiansVal);
        Serial.println("x: " + String(cartesianData[i].x) + " y: " + String(cartesianData[i].y)); 
    }
}
