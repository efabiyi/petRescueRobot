#ifndef DRIVE_H
#define DRIVE_H

extern const int MIN_SPEED;
extern const int MAX_SPEED;
extern const float KP;
extern const float KD;

void initializeDrive();
void leftDrive(int speed);
void rightDrive(int speed);
void lineFollow(int baseSpeed, int threshold);
void testDrive(int leftSpeed, int rightSpeed);
void stopMotors();

#endif