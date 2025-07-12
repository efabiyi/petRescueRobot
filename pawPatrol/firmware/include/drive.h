#ifndef DRIVE_H
#define DRIVE_H

// Constants
extern const int BASE_SPEED;
extern const int MIN_SPEED;
extern const int MAX_SPEED;
extern const float KP;
extern const float KD;

// Pin definitions
extern const int LEFT_SENSOR;
extern const int MIDDLE_SENSOR;
extern const int RIGHT_SENSOR;
extern const int FWD_LEFT_PWM;
extern const int BWD_LEFT_PWM;
extern const int FWD_RIGHT_PWM;
extern const int BWD_RIGHT_PWM;
extern const int FWD_LEFT_CHAN;
extern const int BWD_LEFT_CHAN;
extern const int FWD_RIGHT_CHAN;
extern const int BWD_RIGHT_CHAN;

// Function declarations
void initializeDrive();
void leftDriveForward(int speed);
void leftDriveBackward(int speed);
void rightDriveForward(int speed);
void rightDriveBackward(int speed);
float calculateError();
String drive();
void testDrive(int leftSpeed, int rightSpeed);
void testDriveBwd(int leftSpeed, int rightSpeed);

#endif
