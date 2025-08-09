// motion_controller.h
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"

class MotionController {
public:
    MotionController();
    void begin();
    void moveForward(int cells);
    void moveBackward(int cells);
    void turnLeft();
    void turnRight();
    void stop();
    
    // PID Tuning Functions
    void setStraightPID(float kp, float ki, float kd);
    void setTurnPID(float kp, float ki, float kd);

private:
    PID straightPID;
    PID turnPID;
    bool pidEnabled = true;
    
    void move(int cells, bool forward);
    void updatePosition(bool forward);
};

#endif