#include <Arduino.h>
#include "lift.h"
#include "constants.h"



Lift::Lift(int fwdPin, int bwdPin) {
    
  ledcSetup(FWD_PWM, LIFT_PWM_FREQUENCY, 12);
  ledcSetup(BWD_PWM, LIFT_PWM_FREQUENCY, 12);

  ledcAttachPin(FWD_LIFT_PIN, FWD_PWM);
  ledcAttachPin(BWD_LIFT_PIN, BWD_PWM);
  
  pinMode(LIFT_LIMIT_PIN, INPUT_PULLUP);
  
}

void Lift::setSpeed(int newSpeed) {
  // Ensure speed is within valid range
  speed = constrain(newSpeed, MIN_LIFT_SPEED, MAX_LIFT_SPEED);
}

void Lift::raise(int timeMs) {
  ledcWrite(FWD_PWM, speed);
  ledcWrite(BWD_PWM, 0);
}

void Lift::lower(int timeMs) {
  ledcWrite(BWD_PWM, speed);
  ledcWrite(FWD_PWM, 0);
  delay(timeMs);
  stop();
}

void Lift::stop() {
    ledcWrite(FWD_PWM, 0);
    ledcWrite(BWD_PWM, 0);
}

