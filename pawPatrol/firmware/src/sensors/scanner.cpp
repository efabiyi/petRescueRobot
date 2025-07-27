#include "scanner.h"    
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include "constants.h"

Scanner::Scanner (Logger& logger) : logger(logger){
    servoStep = SERVO_STEP_SIZE;
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
    setServoAngle(90);
    clearScanData();
}

int Scanner::readDistance() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        return measure.RangeMilliMeter + 15;
    } else {
        return MAX_DISTANCE;
    }
}

bool Scanner::completedScan() {
    return servoAngle >= MAX_ANGLE || servoAngle <= MIN_ANGLE;
}

void Scanner::scanOneStep(int scanDelay) {
    if (completedScan()) {
        servoStep = -1 * servoStep;
    }
    servoAngle = servoAngle + servoStep;
    setServoAngle(servoAngle);
    delay(scanDelay);
    int distance = readDistance();
    // Serial.println(distance);
    int index = (servoAngle - MIN_ANGLE) / SERVO_STEP_SIZE;
    scanData[index].distance = distance;
    scanData[index].angle = servoAngle;
}

PolarPoint Scanner::honeIn(int angle) {
    int minAngle = max(angle - 20, MIN_ANGLE);
    int maxAngle = min(angle + 20, MAX_ANGLE);
    int ang1, ang2;
    int d1 = MAX_DISTANCE;
    int d2 = MAX_DISTANCE;
    PolarPoint obj;
    obj.angle = -1;
    obj.distance = MAX_DISTANCE;
    for (int theta = minAngle; theta <= maxAngle; theta += 5) {
        setServoAngle(theta);
        delay(200);
        int distance = readDistance();
        if (distance < d1) {
            ang1 = theta;
            d1 = distance;
        }
    }
    for (int theta = maxAngle; theta >= minAngle; theta -= 5) {
        setServoAngle(theta);
        delay(200);
        int distance = readDistance();
        if (distance < d2) {
            ang2 = theta;
            d2 = distance;
        }
    }
    obj.angle = (ang1 + ang2) / 2;
    obj.distance = (d1 + d2) / 2;
    return obj;
}

void Scanner::setServoAngle(int angle) {
    ledcWrite(SERVO_CHANNEL, angleToDutyMG996R(angle));
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

bool Scanner::closestObjectIsWall() {
    PolarPoint obj = getClosestObject();
    if (obj.angle <= 30 || obj.angle >= 150) return true;
    int objectIndex = (obj.angle - MIN_ANGLE) / SERVO_STEP_SIZE;
    int startIndex = objectIndex;
    int endIndex = objectIndex;
    for (int i = objectIndex; i < SCAN_DATA_SIZE; i++) {
        if (scanData[i].distance > 1000) {
            break;
        }
        endIndex = i;
    }
    for (int i = objectIndex; i >= 0; i--) {
        if (scanData[i].distance > 1000) {
            break;
        }
        startIndex = i;
    }

    CartesianPoint cartesianData[SCAN_DATA_SIZE];

    for (int i = startIndex; i <= endIndex; i++) {
        float radiansVal = radians(scanData[i].angle);
        cartesianData[i].x = (scanData[i].distance) * cos(radiansVal);
        cartesianData[i].y = (scanData[i].distance) * sin(radiansVal);
        // Serial.println("x: " + String(cartesianData[i].x) + " y: " + String(cartesianData[i].y)); 
    }

    int width = cartesianData[endIndex].x - cartesianData[startIndex].x;
    // Serial.println("width: " + String(width));

    float secondDeriv[SCAN_DATA_SIZE - 2];

    for (int i = startIndex + 1; i <= endIndex - 1; i++) {
        float prevY = cartesianData[i - 1].y;
        float y = cartesianData[i].y;
        float nextY = cartesianData[i + 1].y;

        float h1 = cartesianData[i].x - cartesianData[i - 1].x;
        float h2 = cartesianData[i + 1].x - cartesianData[i].x;
        float denom1 = h1 * (h1 + h2);
        float denom2 = h1 * h2;
        float denom3 = h2 * (h1 + h2);

        float deriv = 0.0;

        if (denom1 != 0 && denom2 != 0 && denom3 != 0) {
            deriv = 2.0f / denom1 * prevY - 2.0f / denom2 * y + 2.0f / denom3 * nextY;
        } else {
            deriv = 0.0;  // Assign a safe default
        }

        secondDeriv[i] = deriv;
        if (deriv > 0.03) {
            return false;
        }
        // Serial.println("deriv: " + String(deriv));
    }

    return false;
}
