#ifndef SCANNER_H
#define SCANNER_H

#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <limits.h>

struct PolarPoint {
    int distance;
    int angle;
};

struct CartesianPoint {
    int x;
    int y;
};

class Scanner {
private:
    static const int SCAN_DATA_SIZE = 13;
    static const int SERVO_STEP_SIZE = 10;
    static const int MAX_ANGLE = 150;
    static const int MIN_ANGLE = 30;
    static const int SERVO_CHANNEL = 5;
    static const int SERVO_PIN = 0;
    static const int MAX_DISTANCE = 9999;
    static const int IN_RANGE_DISTANCE = 300;
    
    Adafruit_VL53L0X lox;
    PolarPoint scanData[SCAN_DATA_SIZE];
    int servoStep = SERVO_STEP_SIZE;
    int servoAngle = 90;
public:  
    int angleToDutyCycle(int angle);
    int angleToIndex(int angle);
    void saveScanData(int angle, int distance);
    int readDistance();
    void clearScanData();
    void setServoAngle(int angle);

    Scanner();
    bool initialize();
    int getServoAngle() { return servoAngle; }
    PolarPoint getClosestObject();
    void scanOneStep();
    bool completedScan();
    void printScanData();
    bool closestObjectIsWall();
};

#endif