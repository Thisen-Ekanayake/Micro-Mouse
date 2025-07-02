#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float prev_error, integral;
} PID;

void pid_reset(PID *pid);
float pid_compute(PID *pid, float setpoint, float measurement);

#endif