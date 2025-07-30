#include <Arduino.h>
#include "claw.h"
#include "constants.h"

Claw::Claw(Logger& logger) : logger(logger) {
    z = Z_IDLE_ANGLE;
    base = BASE_IDLE_ANGLE;
    elbow = ELBOW_IDLE_ANGLE;
    gripper = GRIPPER_OPEN_ANGLE;
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

    ledcWrite(Z_SERVO_CHANNEL, angleToDutyMG996R(Z_IDLE_ANGLE));
    ledcWrite(BASE_SERVO_CHANNEL, angleToDuty25kgcm(BASE_IDLE_ANGLE));
    ledcWrite(ELBOW_SERVO_CHANNEL, angleToDuty25kgcm(ELBOW_IDLE_ANGLE));
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(GRIPPER_OPEN_ANGLE));

    pinMode(HALL_PIN, INPUT);

    moveToIdlePos();
}

void Claw::moveToIdlePos() {
    ledcWrite(Z_SERVO_CHANNEL, angleToDutyMG996R(Z_IDLE_ANGLE));
    ledcWrite(BASE_SERVO_CHANNEL, angleToDuty25kgcm(BASE_IDLE_ANGLE));
    ledcWrite(ELBOW_SERVO_CHANNEL, angleToDuty25kgcm(ELBOW_IDLE_ANGLE));
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(GRIPPER_OPEN_ANGLE));
}

void Claw::openGripper() {
    gripper = GRIPPER_OPEN_ANGLE;
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(GRIPPER_OPEN_ANGLE));
}

void Claw::closeGripper() {
    gripper = GRIPPER_CLOSE_ANGLE;
    ledcWrite(GRIPPER_SERVO_CHANNEL, angleToDutyMG996R(GRIPPER_CLOSE_ANGLE));
}

void Claw::setZAxisServo(int angle) {
    angle = constrain(angle, Z_MIN_ANGLE, Z_MAX_ANGLE);
    z = angle;
    ledcWrite(Z_SERVO_CHANNEL, angleToDutyMG996R(angle));
}

void Claw::setBaseServo(int angle) {
    angle = constrain(angle, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
    base = angle;
    ledcWrite(BASE_SERVO_CHANNEL, angleToDuty25kgcm(angle));
}

void Claw::setElbowServo(int angle) {
    angle = constrain(angle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
    elbow = angle;
    ledcWrite(ELBOW_SERVO_CHANNEL, angleToDuty25kgcm(angle));
}

float Claw::distanceFromOrigin(float x, float y) {
    return sqrt(x * x + y * y);
}

float Claw::getElbowAngle(float x, float y) { 
    float d = distanceFromOrigin(x, y); 
    if (d > DABI + XIAOBI) return -1; 
    return degrees(acos(((DABI * DABI) + (XIAOBI * XIAOBI) - d * d) / (2 * DABI * XIAOBI))); 
} 
  
float Claw::getBaseAngle(float x, float y) { 
    float d = distanceFromOrigin(x, y); 
    if (d > DABI + XIAOBI) return -1; 
        
    float beta = degrees(acos(((DABI * DABI) + (d * d) - (XIAOBI * XIAOBI)) / (2 * DABI * d))); 
    float alpha = degrees(atan2(y,x)); 

    float shoulderAngle = beta + alpha; 

    return shoulderAngle; 
}

PolarPoint calculateOffset(PolarPoint scannerPoint) {
    PolarPoint clawPoint;
    if (scannerPoint.angle != 90) {
        clawPoint.angle = degrees(atan2(OFFSET + scannerPoint.distance * sin(radians(scannerPoint.angle)), scannerPoint.distance * cos(radians(scannerPoint.angle))));
    } else {
        clawPoint.angle = 90;
    }
    clawPoint.distance = sqrt((OFFSET + scannerPoint.distance * sin(radians(scannerPoint.angle))) * (OFFSET + scannerPoint.distance * sin(radians(scannerPoint.angle))) + (scannerPoint.distance * cos(radians(scannerPoint.angle))) * (scannerPoint.distance * cos(radians(scannerPoint.angle)))) - 70;
    return clawPoint;
}

void Claw::searchForPet(int angle, int distance) {
    setZAxisServo(angle);
    int x = distance;
    for (int y = 50; y > 0; y -= 5) {
        int baseAngle = getBaseAngle(x, y);
        int elbowAngle = getElbowAngle(x, y);
        if (baseAngle < 0 || elbowAngle < 0) continue;
        setBaseServo(baseAngle);
        setElbowServo(elbowAngle);
    }
}

void Claw::grabPet(int angle, int distance) {
    int x = distance;
    int y = 30; // height of pet
    int elbowAngle = getElbowAngle(x, y);
    int baseAngle = getBaseAngle(x, y);
    if (elbowAngle < 0 || baseAngle < 0) return;
    openGripper();
    delay(1000);
    setZAxisServo(angle);
    delay(1000);
    setElbowServo(elbowAngle);
    delay(1000);
    setBaseServo(baseAngle);
    delay(1000);
    closeGripper();
    delay(1000);
    setBaseServo(BASE_IDLE_ANGLE);
    setElbowServo(ELBOW_IDLE_ANGLE);
    setZAxisServo(Z_IDLE_ANGLE);
    delay(1000);
}
