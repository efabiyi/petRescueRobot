#include <Arduino.h>
#include "claw.h"
#include "pins.h"

Claw::Claw() {
    zAngle = Z_IDLE_ANGLE;
    baseAngle = BASE_IDLE_ANGLE;
    elbowAngle = ELBOW_IDLE_ANGLE;
    gripperAngle = GRIPPER_OPEN_ANGLE;
}

void Claw::initialize() {
    ledcSetup(Z_SERVO_CHANNEL, 50, 12);
    ledcSetup(BASE_SERVO_CHANNEL, 50, 12);
    ledcSetup(ELBOW_SERVO_CHANNEL, 50, 12);
    ledcSetup(GRIPPER_SERVO_CHANNEL, 50, 12);

    ledcAttachPin(Z_SERVO_PIN, Z_SERVO_CHANNEL);
    ledcAttachPin(BASE_SERVO_PIN, BASE_SERVO_CHANNEL);
    ledcAttachPin(ELBOW_SERVO_PIN, ELBOW_SERVO_CHANNEL);
    ledcAttachPin(GRIPPER_SERVO_PIN, GRIPPER_SERVO_CHANNEL);

    pinMode(HALL_PIN, INPUT);

    moveToIdlePos();
}

void Claw::moveToIdlePos() {
    setZAxisServo(Z_IDLE_ANGLE);
    setBaseServo(BASE_IDLE_ANGLE);
    setElbowServo(ELBOW_IDLE_ANGLE);
    openGripper();
}

void Claw::openGripper() {
    gripperAngle = GRIPPER_OPEN_ANGLE;
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(gripperAngle));
}

void Claw::closeGripper() {
    gripperAngle = GRIPPER_CLOSE_ANGLE;
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(gripperAngle));
}

void Claw::halfOpenGripper() {
    gripperAngle = GRIPPER_HALF_ANGLE;
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(gripperAngle));
}

void Claw::setZAxisServo(int angle) {
    angle = constrain(angle, Z_MIN_ANGLE, Z_MAX_ANGLE);
    if (angle > zAngle) {
        for (int i = zAngle; i <= angle; i++) {
            ledcWrite(Z_SERVO_CHANNEL, angleToDutyZ(i));
            delay(10);
        }
    } else {
        for (int i = zAngle; i >= angle; i--) {
            ledcWrite(Z_SERVO_CHANNEL, angleToDutyZ(i));
            delay(10);
        }
    }
    zAngle = angle;
}

void Claw::setBaseServo(int angle) {
    angle = constrain(angle, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
    baseAngle = angle;
    ledcWrite(BASE_SERVO_CHANNEL, angleToDutyBase(angle));
}

void Claw::setElbowServo(int angle) {
    angle = constrain(angle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
    elbowAngle = angle;
    ledcWrite(ELBOW_SERVO_CHANNEL, angleToDutyElbow(angle));
}

float Claw::distanceFromOrigin(float x, float y) {
    return sqrt(x * x + y * y);
}

float Claw::getElbowAngle(float x, float y) {
    float d = distanceFromOrigin(x, y); 
    if (d > DABI + XIAOBI) return -1;

    float cosAngle = ((DABI * DABI) + (XIAOBI * XIAOBI) - d * d) / (2 * DABI * XIAOBI);
    cosAngle = constrain(cosAngle, -1.0, 1.0);
    return degrees(acos(cosAngle));
}
  
float Claw::getBaseAngle(float x, float y) { 
    float d = distanceFromOrigin(x, y); 
    if (d > DABI + XIAOBI) return -1; 

    float cosBeta = ((DABI * DABI) + (d * d) - (XIAOBI * XIAOBI)) / (2 * DABI * d);
    cosBeta = constrain(cosBeta, -1.0, 1.0);
    float beta = degrees(acos(cosBeta));
    float alpha = degrees(atan2(y,x)); 

    float shoulderAngle = beta + alpha; 

    return shoulderAngle; 
}

PolarPoint calculateOffset(PolarPoint scannerPoint) {
    PolarPoint clawPoint;
    float d = scannerPoint.distance;
    float t = scannerPoint.angle;
    if (scannerPoint.angle != 90) {
        clawPoint.angle = degrees(atan2(OFFSET + d * sin(radians(t)), d * cos(radians(t))));
    } else {
        clawPoint.angle = 90;
    }
    clawPoint.distance = sqrt((OFFSET + d * sin(radians(t))) * (OFFSET + d * sin(radians(t))) + (d * cos(radians(t))) * (d * cos(radians(t))));
    return clawPoint;
}

float Claw::readVoltage() { 
  int sensorValue = analogRead(HALL_PIN); 
  float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0); 
  return voltage; 
}

bool Claw::searchPet(int angle, int distance, int petNumber) { 
    Serial.println("claw searching at " + String(angle) + " degrees");
    int step = -1;
    int zDeg = angle;
    int startY = 100;
    int zRange = 8;
    if (petNumber == 3 || petNumber == 4) {
        setBaseServo(110);
        delay(500);
        setElbowServo(150);
        delay(500);
        startY = 300;
    }
    closeGripper();
    setZAxisServo(angle);
    for (float y = startY; y >= 0; y -= 5) { 
        float elbowDeg = getElbowAngle(distance, y); 
        float baseDeg = getBaseAngle(distance, y); 
        // Serial.println("y: " + String(y));
        // Serial.println("elbowDeg: " + String(elbowDeg));
        // Serial.println("baseDeg: " + String(baseDeg));
        // Serial.println("---");

        if (elbowDeg < 0 || baseDeg < 0) continue; 

        setElbowServo(elbowDeg); 
        setBaseServo(baseDeg); 
        delay(200);

        // while (true) {
        //     setZAxisServo(zDeg);
        //     zDeg += step;
        //     if (zDeg >= angle + zRange || zDeg <= angle - zRange) {
        //         step = -1 * step;
        //         break;
        //     }
            float currentVoltage = readVoltage();
            if (currentVoltage > MAGNET_THRESHOLD_VOLTAGE) {
                // Serial.println("found magnet");
                float b = getBaseAngle(distance, y + 50);
                float e = getElbowAngle(distance, y + 50);
                setElbowServo(e);
                delay(500);
                setBaseServo(b);
                delay(500);
                openGripper();
                // if (petNumber == 4) halfOpenGripper();
                delay(500);
                setZAxisServo(zDeg);
                delay(500);
                for (float h = y; h >= y - 150; h -= 10) {
                    b = getBaseAngle(distance, h);
                    e = getElbowAngle(distance, h);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }
                for (float d = distance; d <= distance + 50; d += 10) {
                    b = getBaseAngle(d, y - 150);
                    e = getElbowAngle(d, y - 150);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }
                closeGripper();
                delay(1000);
                setBaseServo(135);
                delay(500);
                setElbowServo(90);
                setZAxisServo(90);
                delay(500);
                return true; 
            } 
        // }
    } 

    return false; 
}

void Claw::grabPet(int angle, int distance, int petNumber) { 
    int h = -150;
    int r = 30;
    int dlay = 50;
    if (petNumber == 6) {
        // distance += 30;
        distance -= 50;
        dlay = 200;
        r = 60;
        // h = -150;
    } else if (petNumber == 7) {
        distance += 30;
        h = -90;
        angle = 170;
    } else if (petNumber == 3 || petNumber == 4) {
        setBaseServo(110);
        setElbowServo(150);
        delay(100);
        h = 60;
        // if (petNumber == 4) angle -= 7;
        r = 100;
    } else {
        setBaseServo(120);
        setElbowServo(45);
        delay(100);
    }
    setZAxisServo(angle);
    openGripper();
    if (petNumber == 4) halfOpenGripper();
    for (float d = distance - r; d <= distance + r; d += 10) {
        float b = getBaseAngle(d, h);
        float e = getElbowAngle(d, h);
        if (b < 0 || e < 0) continue;
        setBaseServo(b);
        setElbowServo(e);
        delay(dlay);
    }
    delay(500);
    closeGripper();
    delay(500);
    if (petNumber == 4) {
        for (float i = h; i <= h + 100; i += 10) {
            float b = getBaseAngle(distance + r, h);
            float e = getElbowAngle(distance + r, h);
            if (b < 0 || e < 0) continue;
            setBaseServo(b);
            setElbowServo(e);
            delay(dlay);
        }
    }
    if (petNumber >= 5) {
        setBaseServo(getBaseAngle(distance, h + 100));
        setElbowServo(getElbowAngle(distance, h + 100));
        delay(500);
    }
    setBaseServo(135);
    if (petNumber >= 5) delay(500);
    setElbowServo(60);
    setZAxisServo(90);
}

void Claw::dump() {
    setBaseServo(110);
    setElbowServo(120);
    delay(200);
    setZAxisServo(300);
    delay(100);
    setElbowServo(100);
    openGripper();
    delay(500);
    setElbowServo(120);
}

void Claw::dunk() {
    setBaseServo(100);
    for (int i = 120; i <= 200; i += 10) {
        setElbowServo(i);
        delay(10);
    }
    delay(1000);
    openGripper();
    delay(500);
}

void Claw::rampToss() {
    float b = getBaseAngle(300, 80);
    float e = getElbowAngle(300, 80);
    setBaseServo(b);
    setElbowServo(e);
    setZAxisServo(170);
    for (int d = 300; d <= 400; d += 10) {
        b = getBaseAngle(d, 80);
        e = getElbowAngle(d, 80);
        setBaseServo(b);
        setElbowServo(e);
        if (b < 0 || e < 0) continue;
        delay(20);
    }
    openGripper();
    delay(500);
    moveToIdlePos();
}

// void Claw::wallToss() {
//     setBaseServo(110);
//     setElbowServo(120);
//     delay(500);
//     setZAxisServo(180);
//     delay(1000);
//     openGripper();
//     delay(1000);
// }

void Claw::windowToss() {
    float b = getBaseAngle(250, 40);
    float e = getElbowAngle(250, 40);
    setBaseServo(b);
    setElbowServo(e);
    setZAxisServo(170);
    for (int d = 250; d <= 400; d += 10) {
        b = getBaseAngle(d, 40);
        e = getElbowAngle(d, 40);
        setBaseServo(b);
        setElbowServo(e);
        if (b < 0 || e < 0) continue;
        delay(20);
    }
    openGripper();
    delay(500);
    closeGripper();
    for (int d = 400; d >= 250; d -= 10) {
        b = getBaseAngle(d, 70);
        e = getElbowAngle(d, 70);
        setBaseServo(b);
        setElbowServo(e);
        if (b < 0 || e < 0) continue;
        delay(20);
    }
    setZAxisServo(90);
}