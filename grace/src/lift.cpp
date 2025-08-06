#include <Arduino.h>
#include "lift.h"

Lift::Lift() {
  ledcSetup(LIFT_FWD_PWM, LIFT_PWM_FREQUENCY, 12);
  ledcSetup(LIFT_BWD_PWM, LIFT_PWM_FREQUENCY, 12);

  ledcAttachPin(LIFT_FWD_PIN, LIFT_FWD_PWM);
  ledcAttachPin(LIFT_BWD_PIN, LIFT_BWD_PWM);
  
  pinMode(LIFT_LIMIT_PIN, INPUT_PULLUP);
}

void Lift::setSpeed(int newSpeed) {
  speed = constrain(newSpeed, MIN_LIFT_SPEED, MAX_LIFT_SPEED);
}

void Lift::raise(int timeMs) {
  ledcWrite(LIFT_FWD_PWM, speed);
  ledcWrite(LIFT_BWD_PWM, 0);
}

void Lift::lower(int timeMs) {
  ledcWrite(LIFT_BWD_PWM, speed);
  ledcWrite(LIFT_FWD_PWM, 0);
  delay(timeMs);
  stop();
}

void Lift::stop() {
    ledcWrite(LIFT_FWD_PWM, 0);
    ledcWrite(LIFT_BWD_PWM, 0);
}

