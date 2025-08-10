/*
// motion_controller.h
#pragma once
#include <Arduino.h>
#include "encoder.h"
#include "motor.h"
#include "imu.h"

class MotionController {
public:
    MotionController();
    void begin();
    void moveForward(float distance_cm);
    void moveBackward(float distance_cm);
    void turnLeft();
    void turnRight();
    void stop();
    void setPID(float kp, float ki, float kd, bool straight);
    
private:
    struct PID {
        float kp = 0.8, ki = 0.05, kd = 0.2;
        float integral = 0;
        float prev_error = 0;
        void reset() { integral = 0; prev_error = 0; }
    } straightPID, turnPID;
    
    void moveLinear(float distance_cm, bool forward);
    void turnToAngle(float target_deg);
    float normalizeAngle(float angle);
};
*/