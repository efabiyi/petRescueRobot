#ifndef CLAW_H
#define CLAW_H

#include "utils.h"

const int OFFSET = 86;

PolarPoint calculateOffset(PolarPoint scannerPoint);

class Claw {
  private:
    static constexpr int GRIPPER_OPEN_ANGLE = 0;
    static constexpr int GRIPPER_CLOSE_ANGLE = 110; 

    static constexpr int Z_MIN_ANGLE = 0; 
    static constexpr int Z_MAX_ANGLE = 180;
    static constexpr int Z_IDLE_ANGLE = 90; 

    static constexpr int ELBOW_MIN_ANGLE = 0; 
    static constexpr int ELBOW_MAX_ANGLE = 180; 
    static constexpr int ELBOW_IDLE_ANGLE = 70; 

    static constexpr int BASE_MIN_ANGLE = 0; 
    static constexpr int BASE_MAX_ANGLE = 180; 
    static constexpr int BASE_IDLE_ANGLE = 110; 

    static constexpr int DABI = 127;
    static constexpr int XIAOBI = 381;
    
    static constexpr float HALL_VOLTAGE_REF = 3.3;
    static constexpr float MAGNET_THRESHOLD_VOLTAGE = 1; 

    // current positions
    int zAngle;
    int baseAngle;
    int elbowAngle;
    int gripperAngle;

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
    float readVoltage();
    bool searchPet(int angle, int distance);
};

#endif 