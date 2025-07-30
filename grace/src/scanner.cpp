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
        return measure.RangeMilliMeter + 15;
    } else {
        return MAX_DISTANCE;
    }
}

bool Scanner::completedScan() {
    return servoAngle >= MAX_ANGLE || servoAngle <= MIN_ANGLE;
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

bool Scanner::scanOneStep(int scanDelay) {
    if (completedScan()) {
        servoStep = -1 * servoStep;
    }
    servoAngle = servoAngle + servoStep;
    setServoAngle(servoAngle);
    delay(scanDelay);
    int distance = readDistance();
    return (distance <= IN_RANGE_DISTANCE);
}

PolarPoint Scanner::honeIn(int angle) {
    int minAngle = max(angle - 5, MIN_ANGLE);
    int maxAngle = min(angle + 5, MAX_ANGLE);
    PolarPoint obj;
    obj.angle = -1;
    obj.distance = MAX_DISTANCE;
    if (angle < 0) return obj;
    for (int theta = minAngle; theta <= maxAngle; theta += 2) {
        setServoAngle(theta);
        delay(500);
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

PolarPoint Scanner::getClosestObject() {
    PolarPoint closestObject;
    closestObject.distance = MAX_DISTANCE;
    closestObject.angle = -1;
    for (int i = MIN_ANGLE; i <= MAX_ANGLE; i += 5) {
        setServoAngle(i);
        delay(50);
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

// bool Scanner::isWall(PolarPoint data[], int size) {
//     int threshold = 1000;
//     int startIndex = -1;
//     int endIndex = -1;
//     bool inCluster = false;

//     for (int i = 0; i < size; i++) {
//         if (!inCluster && data[i].distance < threshold) {
//             startIndex = i;
//             inCluster = true;
//         }
//         else if (inCluster && data[i].distance >= threshold) {
//             endIndex = i - 1;
//             break;
//         }
//     }

//     if (inCluster && endIndex == -1) {
//         endIndex = size - 1;
//     }

//     CartesianPoint cartesianData[SCAN_DATA_SIZE];

//     for (int i = startIndex; i <= endIndex; i++) {
//         float radiansVal = radians(scanData[i].angle);
//         cartesianData[i].x = (scanData[i].distance) * cos(radiansVal);
//         cartesianData[i].y = (scanData[i].distance) * sin(radiansVal);
//         // Serial.println("x: " + String(cartesianData[i].x) + " y: " + String(cartesianData[i].y)); 
//     }

//     int width = cartesianData[endIndex].x - cartesianData[startIndex].x;
//     // Serial.println("width: " + String(width));

//     float secondDeriv[SCAN_DATA_SIZE - 2];

//     for (int i = startIndex + 1; i <= endIndex - 1; i++) {
//         float prevY = cartesianData[i - 1].y;
//         float y = cartesianData[i].y;
//         float nextY = cartesianData[i + 1].y;

//         float h1 = cartesianData[i].x - cartesianData[i - 1].x;
//         float h2 = cartesianData[i + 1].x - cartesianData[i].x;
//         float denom1 = h1 * (h1 + h2);
//         float denom2 = h1 * h2;
//         float denom3 = h2 * (h1 + h2);

//         float deriv = 0.0;

//         if (denom1 != 0 && denom2 != 0 && denom3 != 0) {
//             deriv = 2.0f / denom1 * prevY - 2.0f / denom2 * y + 2.0f / denom3 * nextY;
//         } else {
//             deriv = 0.0;  // Assign a safe default
//         }

//         secondDeriv[i] = deriv;
//         if (deriv > 0.03) {
//             return false;
//         }
//         // Serial.println("deriv: " + String(deriv));
//     }

//     return false;
// }
