#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>
#include "constants.h"
#include "logger.h"

class Drive {
    private:
        Logger &logger;

    public:
        Drive(Logger &logger);
        void initializeDrive();
        void drive();
};

void leftDriveForward(int speed);
void leftDriveBackward(int speed);
void rightDriveForward(int speed);
void rightDriveBackward(int speed);
float calculateError(int l, int r);
void testDrive(int leftSpeed, int rightSpeed);
void testDriveBwd(int leftSpeed, int rightSpeed);

#endif
