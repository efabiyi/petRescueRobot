#ifndef SCANNER_H
#define SCANNER_H

#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <limits.h>
#include "constants.h"
#include "logger.h"

struct PolarPoint
{
    int distance;
    int angle;
};

struct CartesianPoint
{
    int x;
    int y;
};

class Scanner
    {
    private:

        Logger& logger;
        Adafruit_VL53L0X lox;
        PolarPoint scanData[SCAN_DATA_SIZE];
        int servoStep = SERVO_STEP_SIZE;
        int servoAngle = 90;

    public:
        Scanner(Logger& logger);
        int angleToDutyCycle(int angle);
        int angleToIndex(int angle);
        void saveScanData(int angle, int distance);
        int readDistance();
        void clearScanData();
        void setServoAngle(int angle);

        bool initialize();
        int getServoAngle() { return servoAngle; }
        PolarPoint getClosestObject();
        void scanOneStep();
        bool completedScan();
        void printScanData();
        bool closestObjectIsWall();
};

#endif