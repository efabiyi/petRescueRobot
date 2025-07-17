#include "scanner.h"
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

Scanner::Scanner() {
    servoStep = SERVO_STEP_SIZE;
    servoAngle = 90;
    clearScanData();
}

bool Scanner::initialize() {
    ledcSetup(SERVO_CHANNEL, 50, 12);
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
    setServoAngle(90);
    Serial.println("Initializing VL53L0X sensor");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        return false;
    }
    return true;
}

int Scanner::readDistance() {
    if (servoAngle < MIN_ANGLE || servoAngle > MAX_ANGLE) {
        return MAX_DISTANCE;
    }
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        return measure.RangeMilliMeter;
    } else {
        return MAX_DISTANCE;
    }
}

bool Scanner::completedScan() {
    return servoAngle >= MAX_ANGLE || servoAngle <= MIN_ANGLE;
}

void Scanner::scanOneStep() {
    if (completedScan()) {
        servoStep = -1 * servoStep;
    }
    servoAngle = servoAngle + servoStep;
    setServoAngle(servoAngle);
    delay(10);
    int distance = readDistance();
    // Serial.println(distance);
    int index = (servoAngle - MIN_ANGLE) / SERVO_STEP_SIZE;
    scanData[index].distance = distance;
    scanData[index].angle = servoAngle;
}

void Scanner::setServoAngle(int angle) {
    servoAngle = angle;
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 500, 2500);
    int duty = (pulseWidth * 4095) / 20000;
    ledcWrite(SERVO_CHANNEL, duty);
}

PolarPoint Scanner::getClosestObject() {
    PolarPoint closestObject;
    closestObject.distance = MAX_DISTANCE;
    closestObject.angle = 0;

    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        if (scanData[i].distance < closestObject.distance) {
            closestObject.distance = scanData[i].distance;
            closestObject.angle = scanData[i].angle;
        }
    }

    return closestObject;
}

void Scanner::clearScanData() {
    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        scanData[i].distance = 0;
        scanData[i].angle = 0;
    }
}

void Scanner::printScanData() {
    for (int i = 0; i < SCAN_DATA_SIZE; i++) {
        // Serial.println("Angle: " + String(scanData[i].angle) + " Distance: " + String(scanData[i].distance));
    }
}

bool Scanner::closestObjectIsWall() {
    int objectIndex = (getClosestObject().angle - MIN_ANGLE) / SERVO_STEP_SIZE;
    int startIndex = max(objectIndex - 3, 0);
    int endIndex = min(objectIndex + 3, SCAN_DATA_SIZE - 1);
    // int startIndex = 0;
    // int endIndex = SCAN_DATA_SIZE - 1;
    CartesianPoint cartesianData[SCAN_DATA_SIZE];
    for (int i = startIndex; i <= endIndex; i++) {
        cartesianData[i].x = scanData[i].distance * cos(scanData[i].angle * PI / 180);
        cartesianData[i].y = scanData[i].distance * sin(scanData[i].angle * PI / 180);
        // Serial.print("angle: ");
        // Serial.print(scanData[i].angle);
        // Serial.print(", distance: ");
        // Serial.print(scanData[i].distance);
        // Serial.print(", x: ");
        // Serial.print(cartesianData[i].x);
        // Serial.print(", y: ");
        // Serial.println(cartesianData[i].y);
    }
    float deriv[SCAN_DATA_SIZE - 1];
    float deriv2[SCAN_DATA_SIZE - 2];
    for (int i = startIndex; i < endIndex; i++) {
        float dx = cartesianData[i + 1].x - cartesianData[i].x;
        if (dx != 0) {
            deriv[i] = (cartesianData[i + 1].y - cartesianData[i].y) / dx;
        } else {
            deriv[i] = 0; // or handle specially
        }
        // Serial.print("deriv: ");
        // Serial.println(deriv[i]);
    }
    for (int i = startIndex; i < endIndex - 1; i++) {
        float dx = cartesianData[i + 2].x - cartesianData[i].x;
        if (dx != 0) {
            deriv2[i] = (deriv[i + 1] - deriv[i]) / dx;
        } else {
            deriv2[i] = 0; // or handle specially
        }
        // Serial.print("2nd deriv: ");
        // Serial.println(deriv2[i]);
    }
    for (int i = startIndex; i < endIndex - 1; i++) {
        if (abs(deriv2[i]) >= 0.01) {
            return false;
        }
    }
    return true;
}
  