#ifndef CONSTANTS_H
#define CONSTANTS_H

//INIT CONSTANTS



//PIN DEFINITIONS

//Drive and Reflectance
constexpr int LEFT_SENSOR = 38;
constexpr int RIGHT_SENSOR = 37;

constexpr int FWD_LEFT_PWM = 33;
constexpr int BWD_LEFT_PWM = 32;
constexpr int FWD_RIGHT_PWM = 14;
constexpr int BWD_RIGHT_PWM = 13;

constexpr int FWD_LEFT_CHAN = 0;
constexpr int BWD_LEFT_CHAN = 1;
constexpr int FWD_RIGHT_CHAN = 2;
constexpr int BWD_RIGHT_CHAN = 3;

//Distance Sensor
constexpr int SERVO_CHANNEL = 5;
constexpr int SERVO_PIN = 0;

//Claw
constexpr int Z_SERVO_PIN = 20;
constexpr int BASE_SERVO_PIN = 15;
constexpr int ELBOW_SERVO_PIN = 19;
constexpr int GRIPPER_SERVO_PIN = 27;

constexpr int Z_SERVO_CHANNEL = 6;
constexpr int BASE_SERVO_CHANNEL = 7;
constexpr int ELBOW_SERVO_CHANNEL = 8;
constexpr int GRIPPER_SERVO_CHANNEL = 9;

//Hall Sensor
constexpr int HALL_PIN = 36;

//Lift Pins
constexpr int BWD_LIFT_PIN = 26;
constexpr int FWD_LIFT_PIN = 25;
constexpr int BWD_PWM = 8;
constexpr int FWD_PWM = 9;

// Limit Switches

constexpr int CLAW_LIMIT_PIN = 35;
constexpr int LIFT_LIMIT_PIN = 34;

// CONSTANTS

//Drive and Reflectance Constants
extern const int BASE_SPEED;
extern const int MIN_SPEED;
extern const int MAX_SPEED;
extern const int THRESHOLD;
extern const float KP;
extern const float KD;
extern const int OFF_LINE_THRESHOLD;

//Claw Constants

constexpr int CLAW_OPEN_ANGLE = 0; 
constexpr int CLAW_CLOSE_ANGLE = 110; 

constexpr int BITS = 16; 
constexpr int FREQ = 50; 

constexpr int MG996R_MIN_US = 500; 
constexpr int MG996R_MAX_US = 2500; 

constexpr int _25KGCM_MIN_US = 750; 
constexpr int _25KGCM_MAX_US = 2250; 

constexpr int Z_MIN_ANGLE = 0; 
constexpr int Z_MAX_ANGLE = 180; 
constexpr int Z_STEP_SEARCH_ANGLE = 2; 
constexpr int Z_IDLE_ANGLE = 90; 

constexpr int Y_MIN_SEARCH_ANGLE = 30; 
constexpr int Y_MAX_SEARCH_ANGLE = 110; 
constexpr int Y_STEP_SEARCH = 5; 
constexpr int Y_IDLE_ANGLE = 110; 

constexpr int X_MIN_SEARCH_ANGLE = 90; 
constexpr int X_MAX_SEARCH_ANGLE = 140; 
constexpr int X_STEP_SEARCH = 5; 
constexpr int X_IDLE_ANGLE = 90; 

//Hall Sensor Constants
const float HALL_VOLTAGE_REF = 4.5; 
const float MAGNET_THRESHOLD_VOLTAGE = 2.25; 

//Distance Sensor Constants

constexpr int SCAN_DATA_SIZE = 13;
constexpr int SERVO_STEP_SIZE = 10;

extern const int SCAN_DATA_SIZE;
extern const int SERVO_STEP_SIZE;
extern const int MAX_ANGLE;
extern const int MIN_ANGLE;
extern const int MAX_DISTANCE;
extern const int IN_RANGE_DISTANCE;

extern const int IN_RANGE_DISTANCE;
extern const int REACHABLE_DISTANCE;

// Lift Constants
constexpr int LIFT_PWM_FREQUENCY = 250;
constexpr int MAX_LIFT_SPEED = 1600;
constexpr int MIN_LIFT_SPEED = 0;

#endif
