#include <Arduino.h>
#include <ESP32Servo.h>
#include "claw.h"

const int SERVO_ROTATE_Z     = 13;
const int SERVO_FORWARD_BACK = 14;
const int SERVO_UP_DOWN      = 18;
const int SERVO_CLAW         = 19;
const int PULLEY_MOTOR      = 25;

Servo servoRotateZ;     // Base rotation servo
Servo servoForwardBack; // Arm extension servo
Servo servoUpDown;      // Arm height servo
Servo servoClaw;        // Claw grip servo

// Claw position constants
const int CLAW_OPEN = 160;
const int CLAW_CLOSED = 60;
const int CLAW_UP = 150;
const int CLAW_DOWN = 30;
const int CLAW_FORWARD = 150;
const int CLAW_BACK = 30;

// Current positions
static int rotateZPosition = 90;
static int forwardBackPosition = 90;
static int upDownPosition = 90;
static int clawPosition = 90;

void initiateClaw() {
    pinMode(SERVO_CLAW, OUTPUT);
    pinMode(SERVO_ROTATE_Z, OUTPUT);
    pinMode(SERVO_FORWARD_BACK, OUTPUT);
    pinMode(SERVO_UP_DOWN, OUTPUT);
    pinMode(PULLEY_MOTOR, OUTPUT);

    servoRotateZ.attach(SERVO_ROTATE_Z);
    servoForwardBack.attach(SERVO_FORWARD_BACK);
    servoUpDown.attach(SERVO_UP_DOWN);
    servoClaw.attach(SERVO_CLAW);

    // Initialize to neutral positions
    setClawPosition(90);
    setRotateZPosition(90);
    setForwardBackPosition(90);
    setUpDownPosition(90);
}

void handleGrabPet() {
    static enum {
        PREPARE,
        APPROACH,
        GRAB,
        LIFT,
        COMPLETE
    } grabState = PREPARE;
    
    static unsigned long stateStartTime = 0;
    const unsigned long MOVEMENT_DELAY = 1000;
    
    switch(grabState) {
        case PREPARE:
            setClawPosition(CLAW_OPEN);
            setUpDownPosition(CLAW_UP);
            setForwardBackPosition(CLAW_BACK);
            stateStartTime = millis();
            grabState = APPROACH;
            break;
            
        case APPROACH:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setForwardBackPosition(CLAW_FORWARD);
                setUpDownPosition(CLAW_DOWN);
                stateStartTime = millis();
                grabState = GRAB;
            }
            break;
            
        case GRAB:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setClawPosition(CLAW_CLOSED);
                stateStartTime = millis();
                grabState = LIFT;
            }
            break;
            
        case LIFT:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setUpDownPosition(CLAW_UP);
                setForwardBackPosition(CLAW_BACK);
                stateStartTime = millis();
                grabState = COMPLETE;
            }
            break;
            
        case COMPLETE:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                grabState = PREPARE;
            }
            break;
    }
}

void handleStorePet() {
    static enum {
        MOVE_TO_STORAGE,
        RELEASE,
        RETRACT,
        COMPLETE
    } storeState = MOVE_TO_STORAGE;
    
    static unsigned long stateStartTime = 0;
    const unsigned long MOVEMENT_DELAY = 1000;
    
    switch(storeState) {
        case MOVE_TO_STORAGE:
            setRotateZPosition(0);  // Rotate to storage position
            stateStartTime = millis();
            storeState = RELEASE;
            break;
            
        case RELEASE:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setClawPosition(CLAW_OPEN);
                stateStartTime = millis();
                storeState = RETRACT;
            }
            break;
            
        case RETRACT:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setUpDownPosition(CLAW_UP);
                setForwardBackPosition(CLAW_BACK);
                stateStartTime = millis();
                storeState = COMPLETE;
            }
            break;
            
        case COMPLETE:
            if (millis() - stateStartTime > MOVEMENT_DELAY) {
                setRotateZPosition(90);  // Return to center position
                storeState = MOVE_TO_STORAGE;
            }
            break;
    }
}

void setClawPosition(int position) {
    if (position >= 0 && position <= 180) {
        clawPosition = position;
        servoClaw.write(position);
    }
}

void setRotateZPosition(int position) {
    if (position >= 0 && position <= 180) {
        rotateZPosition = position;
        servoRotateZ.write(position);
    }
}

void setForwardBackPosition(int position) {
    if (position >= 0 && position <= 180) {
        forwardBackPosition = position;
        servoForwardBack.write(position);
    }
}

void setUpDownPosition(int position) {
    if (position >= 0 && position <= 180) {
        upDownPosition = position;
        servoUpDown.write(position);
    }
} 