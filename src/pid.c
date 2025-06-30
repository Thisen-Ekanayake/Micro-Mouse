#include "pid.h"

float pid_compute(PID *pid, float setpoint, float current) {
    float error = setpoint - current;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
