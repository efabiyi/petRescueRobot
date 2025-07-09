#include <Arduino.h>
#include "driver/ledc.h"
#include "drive.h"

// Constants
const int BASE_SPEED = 1200;
const int MIN_SPEED = 0;
const int MAX_SPEED = 1600;
const int THRESHOLD = 50;
const float KP = 15.0f;
const float KD = 0.5f;

// Pin definitions
const int LEFT_SENSOR = 36;
const int MIDDLE_SENSOR = 35;
const int RIGHT_SENSOR = 34;
const int FWD_LEFT_PWM = 14;
const int BWD_LEFT_PWM = 4;
const int FWD_RIGHT_PWM = 13;
const int BWD_RIGHT_PWM = 5;
const int FWD_LEFT_CHAN = 0;
const int BWD_LEFT_CHAN = 1;
const int FWD_RIGHT_CHAN = 2;
const int BWD_RIGHT_CHAN = 3;

// Global variables
float last_error = 0.0;
unsigned long last_time = 0;
float last_correction = 0.0;

const int OFF_LINE_THRESHOLD = 100;
int off_line_counter = OFF_LINE_THRESHOLD;

void debug_print(const String &message) {
  // Serial.print(message);     // Comment out to improve performance
}

void debug_println(const String &message) {
  debug_print(message);
  Serial.println();
}

void initialize_drive() {
  ledcSetup(FWD_LEFT_CHAN, 250, 12);
  ledcSetup(BWD_LEFT_CHAN, 250, 12);
  ledcSetup(FWD_RIGHT_CHAN, 250, 12);
  ledcSetup(BWD_RIGHT_CHAN, 250, 12);

  ledcAttachPin(FWD_LEFT_PWM, FWD_LEFT_CHAN);
  ledcAttachPin(BWD_LEFT_PWM, BWD_LEFT_CHAN);
  ledcAttachPin(FWD_RIGHT_PWM, FWD_RIGHT_CHAN);
  ledcAttachPin(BWD_RIGHT_PWM, BWD_RIGHT_CHAN);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(MIDDLE_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  last_time = millis();
}

void left_drive_forward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(BWD_LEFT_CHAN, 0);
  ledcWrite(FWD_LEFT_CHAN, speed);
} 

void left_drive_backward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(FWD_LEFT_CHAN, 0);
  ledcWrite(BWD_LEFT_CHAN, speed);  
}

void right_drive_forward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(BWD_RIGHT_CHAN, 0);
  ledcWrite(FWD_RIGHT_CHAN, speed);
}

void right_drive_backward(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  ledcWrite(FWD_RIGHT_CHAN, 0);
  ledcWrite(BWD_RIGHT_CHAN, speed);  
}

void left_drive(int speed) {
  if (speed >= 0) {
    left_drive_forward(speed);
  } else {
    left_drive_backward(-1 * speed);
  }
}

void right_drive(int speed) {
  if (speed >= 0) {
    right_drive_forward(speed);
  } else {
    right_drive_backward(-1 * speed);
  }
}

float calculate_error(int l, int m, int r) {
  return (l * -1.0 + r * 1.0) / (l + m + r + 0.001);
}

bool is_off_line(int l, int m, int r) {
  return (l < THRESHOLD && m < THRESHOLD && r < THRESHOLD);
}

void drive() {
  unsigned long now = millis();
  float delta_time = (now - last_time) / 1000.0;
  last_time = now;

  int left_reading = analogRead(LEFT_SENSOR);
  int middle_reading = analogRead(MIDDLE_SENSOR);
  int right_reading = analogRead(RIGHT_SENSOR);
  debug_print("Left: ");
  debug_print(String(left_reading));
  debug_print(" - Middle: ");
  debug_print(String(middle_reading));
  debug_print(" - Right: ");
  debug_print(String(right_reading));
  bool off_line = is_off_line(left_reading, middle_reading, right_reading);

  float error;
  float derivative;
  float correction;
  int left_speed;
  int right_speed;

  if (off_line) {
    if (last_correction > 0) {
      left_speed = BASE_SPEED + 1.5 * last_correction;
      right_speed = BASE_SPEED + 1.5 * last_correction;
      left_drive_backward(left_speed);
      right_drive_forward(right_speed);
    } else {
      left_speed = BASE_SPEED + 1.5 * last_correction;
      right_speed = BASE_SPEED + 1.5 * last_correction;
      left_drive_forward(left_speed);
      right_drive_backward(right_speed);
    }

    // off_line_counter++;
    // left_drive_forward(0);
    // right_drive_forward(0);
    // return;
  } else { // If on line, calculate error and correction
    // if (off_line_counter > 0) {
    //   left_drive_forward(0);
    //   right_drive_forward(0);
    //   off_line_counter = 0;
    //   delay(200);
    // }
    error = calculate_error(left_reading, middle_reading, right_reading);
    derivative = (error - last_error) / delta_time;
    correction = (KP * error) + (KD * derivative);

    left_speed = BASE_SPEED - (correction * BASE_SPEED/2);
    right_speed = BASE_SPEED + (correction * BASE_SPEED/2);
  
    left_drive_forward(left_speed);
    right_drive_forward(right_speed);
    
    last_error = error;
    last_correction = correction;
  }

  debug_print(" - Error: ");
  debug_print(String(error));
  debug_print(" - Derivative: ");
  debug_print(String(derivative));

  debug_print(" - Correction: ");
  debug_print(String(correction));

  debug_print(" - L wheel: ");
  debug_print(String(constrain(left_speed, MIN_SPEED, MAX_SPEED)));
  debug_print(" - R wheel: ");
  debug_println(String(constrain(right_speed, MIN_SPEED, MAX_SPEED)));
  delay(10);
}

void test_drive(int left_speed, int right_speed) {
  left_drive_forward(left_speed);
  right_drive_forward(right_speed);
  delay(10);
}

void test_drive_bwd(int left_speed, int right_speed) {
  left_drive_backward(left_speed);
  right_drive_backward(right_speed);
  delay(10);
}