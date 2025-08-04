#ifndef CLAW_H
#define CLAW_H

#include "utils.h"
#include "logger.h"

const int OFFSET = 90;

PolarPoint calculateOffset(PolarPoint scannerPoint);

class Claw {
  private:
    static constexpr int GRIPPER_OPEN_ANGLE = 0;
    static constexpr int GRIPPER_CLOSE_ANGLE = 100; 

    static constexpr int Z_MIN_ANGLE = 0; 
    static constexpr int Z_MAX_ANGLE = 360;
    static constexpr int Z_IDLE_ANGLE = 90; 

    static constexpr int ELBOW_MIN_ANGLE = 0; 
    static constexpr int ELBOW_MAX_ANGLE = 180; 
    static constexpr int ELBOW_IDLE_ANGLE = 45; 

    static constexpr int BASE_MIN_ANGLE = 0; 
    static constexpr int BASE_MAX_ANGLE = 180; 
    static constexpr int BASE_IDLE_ANGLE = 135; 

    static constexpr int DABI = 127;
    static constexpr int XIAOBI = 330;
    
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
    void halfOpenGripper();
    void setZAxisServo(int angle);
    void setBaseServo(int angle);
    void setElbowServo(int angle);
    float distanceFromOrigin(float x, float y);
    float getElbowAngle(float x, float y);
    float getBaseAngle(float x, float y);
    float readVoltage();
    bool searchPet(int angle, int distance, int petNumber);
    bool searchPetLimitSwitch(int angle, int distance, int petNumber);
    void dump();
    void rampToss();
    void wallToss();
    void windowToss();
};

#endif 