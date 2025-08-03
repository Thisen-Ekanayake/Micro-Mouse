#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_init();
void motor_set_speed(int left_pwm, int right_pwm);
void motor_stop();
void motor_set_speed_ramp(int left_target, int right_target, int ramp_step, int delay_ms);

#ifdef __cplusplus
}
#endif

#endif