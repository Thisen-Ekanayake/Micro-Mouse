#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prev_error, integral;
} PID;

float pid_compute(PID *pid, float setpoint, float current);

#endif