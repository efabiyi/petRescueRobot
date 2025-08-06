#ifndef LIFT_H
#define LIFT_H
#include "pins.h"

class Lift {
  const int LIFT_PWM_FREQUENCY = 250;
  const int MAX_LIFT_SPEED = 1600;
  const int MIN_LIFT_SPEED = 0;
  private:
    int speed = 1000;
  public:
    Lift();
    void setSpeed(int newSpeed);
    void raise(int timeMs);
    void lower(int timeMs);
    void stop();
};

void IRAM_ATTR onSwitchPress();
#endif