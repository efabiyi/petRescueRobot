#ifndef PINS_H
#define PINS_H

// Motor pins
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

// Scanner pins (laser also uses 22 and 21 for I2C)
constexpr int SERVO_CHANNEL = 5;
constexpr int SERVO_PIN = 0;

// Claw pins
constexpr int Z_SERVO_PIN = 20;
constexpr int BASE_SERVO_PIN = 15;
constexpr int ELBOW_SERVO_PIN = 19;
constexpr int GRIPPER_SERVO_PIN = 27;
constexpr int HALL_PIN = 36;

constexpr int Z_SERVO_CHANNEL = 6;
constexpr int BASE_SERVO_CHANNEL = 7;
constexpr int ELBOW_SERVO_CHANNEL = 8;
constexpr int GRIPPER_SERVO_CHANNEL = 9;

#endif