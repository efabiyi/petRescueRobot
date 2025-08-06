#ifndef DRIVE_H
#define DRIVE_H

extern const int MIN_SPEED;
extern const int MAX_SPEED;
extern const float KP;
extern const float KD;

void initializeDrive();
void leftDrive(int speed);
void rightDrive(int speed);
void lineFollow(int baseSpeed, int threshold, float KP);
void drive(int leftSpeed, int rightSpeed);
void stopMotors();
void uTurnRight(int threshold);
void uTurnLeft(int threshold);
void searchLine(int threshold);

#endif