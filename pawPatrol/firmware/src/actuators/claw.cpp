#include <Arduino.h>
#include "claw.h"
#include "constants.h"
#include "hallsensor.h"
#include "utils.h"
#include "constants.h"
    
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
    pinMode(CLAW_LIMIT_PIN, INPUT_PULLUP);

    moveToIdlePos();
}

void Claw::moveToIdlePos() {
    setZAxisServo(Z_IDLE_ANGLE);
    setBaseServo(BASE_IDLE_ANGLE);
    setElbowServo(ELBOW_IDLE_ANGLE);
    closeGripper();
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
    gripperAngle = 45;
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
        delay(500); // consider reducing or removing
        setElbowServo(120);
        delay(500);
        startY = 275;
    }
    closeGripper();
    setZAxisServo(angle);
    for (float y = startY; y >= 0; y -= 10) { 
        float elbowDeg = getElbowAngle(distance, y); 
        float baseDeg = getBaseAngle(distance, y); 
        if (elbowDeg < 0 || baseDeg < 0) continue; 

        setElbowServo(elbowDeg); 
        setBaseServo(baseDeg); 
        delay(200);

            float currentVoltage = readVoltage();
            if (currentVoltage > MAGNET_THRESHOLD_VOLTAGE) {

                float b = getBaseAngle(distance, y + 50);
                float e = getElbowAngle(distance, y + 50);

                setElbowServo(e);
                delay(100);
                setBaseServo(b);
                delay(100);
                openGripper();

                if (petNumber == 4) halfOpenGripper();
                delay(100);
                if(petNumber == 4){
                    setZAxisServo(zDeg + 10);
                } else{
                    
                 setZAxisServo(zDeg);
                }
                delay(100);


                for (float h = y; h >= y - 160; h -= 10) {
                    b = getBaseAngle(distance - 20, h);
                    e = getElbowAngle(distance - 20, h);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }

                for (float d = distance - 20; d <= distance + 50; d += 10) {
                    b = getBaseAngle(d, y - 120);
                    e = getElbowAngle(d, y - 120);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }
                closeGripper();
                delay(500);
                setBaseServo(135);
                setElbowServo(90);
                delay(100);
                setZAxisServo(90);
                delay(100);
                return true; 
            } 
        // }
    } 

    return false; 
}


bool Claw::searchPetLimitSwitch(int angle, int distance, int petNumber) { //Hall + limit switch detection
    Serial.println("claw searching at " + String(angle) + " degrees");
    int step = -1;
    int zDeg = angle;
    int startY = 80;
    int zRange = 8;
    if (petNumber == 3 || petNumber == 4) {
        setBaseServo(110);
        delay(500); // consider reducing or removing
        setElbowServo(120);
        delay(500);
        startY = 275;
    }
    closeGripper();
    setZAxisServo(angle);
    for (float y = startY; y >= - 60; y -= 10) { 
        float elbowDeg = getElbowAngle(distance, y); 
        float baseDeg = getBaseAngle(distance, y); 
        if (elbowDeg < 0 || baseDeg < 0) continue; 

        setElbowServo(elbowDeg); 
        setBaseServo(baseDeg); 
        delay(200);

            bool isPressed = digitalRead(CLAW_LIMIT_PIN);
            Serial.println(isPressed);

            float currentVoltage = readVoltage();
            if (isPressed || currentVoltage > MAGNET_THRESHOLD_VOLTAGE) {

                float b = getBaseAngle(distance, y + 50);
                float e = getElbowAngle(distance, y + 50);

                setElbowServo(e);
                delay(100);
                setBaseServo(b);
                delay(100);
                openGripper();

                if (petNumber == 4) halfOpenGripper();
                delay(100);
                setZAxisServo(zDeg);
                delay(100);
                for (float h = y; h >= y - 130; h -= 10) {
                    b = getBaseAngle(distance - 20, h);
                    e = getElbowAngle(distance - 20, h);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }
                for (float d = distance - 20; d <= distance + 50; d += 10) {
                    b = getBaseAngle(d, y - 120);
                    e = getElbowAngle(d, y - 120);
                    if (b < 0 || e < 0) continue;
                    setBaseServo(b);
                    setElbowServo(e);
                    delay(100);
                }
                closeGripper();
                delay(500);
                setBaseServo(135);
                setElbowServo(90);
                delay(100);
                setZAxisServo(90);
                delay(100);
                return true; 
            } 
        // }
    } 

    return false; 
}

void Claw::dump() {
    setBaseServo(110);
    setElbowServo(120);
    delay(500);
    setZAxisServo(270);
    delay(1000);
    openGripper();
    delay(1000);
}

void Claw::rampToss() {
    float b = getBaseAngle(300, 80);
    float e = getElbowAngle(300, 80);
    setBaseServo(b);
    delay(500);
    setElbowServo(e);
    delay(500);
    setZAxisServo(160);
    for (int d = 300; d <= 400; d += 10) {
        b = getBaseAngle(d, 80);
        e = getElbowAngle(d, 80);
        setBaseServo(b);
        setElbowServo(e);
        if (b < 0 || e < 0) continue;
        delay(100);
    }
    delay(500);
    openGripper();
    delay(500);
    moveToIdlePos();
    delay(1000);
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
    float b = getBaseAngle(250, 70);
    float e = getElbowAngle(250, 70);
    setBaseServo(b);
    delay(100);
    setElbowServo(e);
    delay(100);
    setZAxisServo(170);
    for (int d = 250; d <= 410; d += 10) {
        b = getBaseAngle(d, 70);
        e = getElbowAngle(d, 70);
        setBaseServo(b);
        setElbowServo(e);
        if (b < 0 || e < 0) continue;
        delay(100);
    }
    delay(100);
    openGripper();
    delay(1000);
    closeGripper();
    delay(100);
    setElbowServo(50);
    setBaseServo(135);
    delay(1000);
    setZAxisServo(90);
    delay(500);
}