#include "pid.h"

void pid_reset(PID *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

float pid_compute(PID *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
