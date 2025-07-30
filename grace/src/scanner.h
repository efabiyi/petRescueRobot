#ifndef SCANNER_H
#define SCANNER_H

#include <Adafruit_VL53L0X.h>
#include "utils.h"

class Scanner {
private:
    static constexpr int SCAN_DATA_SIZE = 13;
    static constexpr int SERVO_STEP_SIZE = 10;
    static constexpr int MAX_ANGLE = 170;
    static constexpr int MIN_ANGLE = 10;
    static constexpr int MAX_DISTANCE = 9999;
    static constexpr int IN_RANGE_DISTANCE = 300;
    
    Adafruit_VL53L0X lox;
    PolarPoint scanData[SCAN_DATA_SIZE];
    int servoStep = SERVO_STEP_SIZE;
    int servoAngle = 90;
    
public:  
    int readDistance();
    void clearScanData();
    void setServoAngle(int angle);

    Scanner();
    bool initialize();
    void reset();
    int getServoAngle() { return servoAngle; }
    PolarPoint getClosestObject();
    bool scanOneStep(int scanDelay);
    bool completedScan();
    void printScanData();
    bool isWall(PolarPoint data[], int size);
    PolarPoint honeIn(int angle);
};

#endif