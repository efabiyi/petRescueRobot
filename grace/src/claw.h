#ifndef CLAW_H
#define CLAW_H

#include "utils.h"

const int OFFSET = 85;

PolarPoint calculateOffset(PolarPoint scannerPoint);

class Claw {
  private:
    static constexpr int GRIPPER_OPEN_ANGLE = 0; 
    static constexpr int GRIPPER_CLOSE_ANGLE = 110; 

    static constexpr int Z_MIN_ANGLE = 0; 
    static constexpr int Z_MAX_ANGLE = 180; 
    static constexpr int Z_STEP_ANGLE = 2; 
    static constexpr int Z_IDLE_ANGLE = 90; 

    static constexpr int ELBOW_MIN_ANGLE = 30; 
    static constexpr int ELBOW_MAX_ANGLE = 110; 
    static constexpr int ELBOW_STEP_SEARCH = 5; 
    static constexpr int ELBOW_IDLE_ANGLE = 60; 

    static constexpr int BASE_MIN_ANGLE = 40; 
    static constexpr int BASE_MAX_ANGLE = 120; 
    static constexpr int BASE_STEP_SEARCH = 5; 
    static constexpr int BASE_IDLE_ANGLE = 110; 

    static constexpr int DABI = 127;
    static constexpr int XIAOBI = 330;

    // current positions
    int z;
    int base;
    int elbow;
    int gripper;

  public:
    Claw();
    void initialize();
    void moveToIdlePos();
    void openGripper();
    void closeGripper();
    void setZAxisServo(int angle);
    void setBaseServo(int angle);
    void setElbowServo(int angle);
    float distanceFromOrigin(float x, float y);
    float getElbowAngle(float x, float y);
    float getBaseAngle(float x, float y);
    void searchForPet(int angle, int distance);
    void grabPet(int angle, int distance);
};

#endif 