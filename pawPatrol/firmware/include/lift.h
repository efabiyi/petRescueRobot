#ifndef LIFT_H
#define LIFT_H
#include "constants.h"

class Lift {
  private:
    int speed = 1000; // Default speed  
  public:
    Lift(int fwdMotorPin, int bwdMotorPin);
    void setSpeed(int newSpeed);
    void raise(int timeMs);
    void lower(int timeMs);
    void stop();
};

#endif