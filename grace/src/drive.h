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
void initialize_drive();
void left_drive_forward(int speed);
void left_drive_backward(int speed);
void right_drive_forward(int speed);
void right_drive_backward(int speed);
float calculate_error();
void drive();
void test_drive(int left_speed, int right_speed);
void test_drive_bwd(int left_speed, int right_speed);

#endif