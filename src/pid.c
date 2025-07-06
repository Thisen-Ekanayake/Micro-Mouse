#include "pid.h"
#include "config.h"

void pid_reset(PID *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

float pid_compute(PID *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error;

    // clamp integral term to avoid windup
    if (pid->integral > PID_INTEGRAL_MAX) pid->integral = PID_INTEGRAL_MAX;
    if (pid->integral < PID_INTEGRAL_MIN) pid->integral = PID_INTEGRAL_MIN;
    
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
