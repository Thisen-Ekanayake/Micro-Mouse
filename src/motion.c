#include <Arduino.h>
#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "config.h"

// turning speed parameters
const int TURN_FAST = 150;
const int TURN_SLOW = 60;
const float TOLERANCE = 1.0;  // degrees for final stop
const float SLOW_DIST = 15.0;  // start slowing down when within 15°

PID straight_pid = {1.0, 0, 0, 0, 0}; // tune this later

void move_forward_cm(float cm) {
    int target_ticks = cm * TICKS_PER_CM;
    reset_ticks();
    pid_reset(&straight_pid);

    while (abs(get_avg_ticks()) < target_ticks) {
        float error = get_left_ticks() - get_right_ticks();
        float correction = pid_compute(&straight_pid, 0, error);

        int base_speed = 150;
        int left_pwm = base_speed - correction;
        int right_pwm = base_speed + correction;

        // allow negative values for reverse (not constrained to 0-255)
        motor_set_speed(left_pwm, right_pwm);
        delay(10);
    }

    motor_stop();
}

void move_backward_cm(float cm) {
    int target_ticks = cm * TICKS_PER_CM;
    reset_ticks();
    pid_reset(&straight_pid);

    while (abs(get_avg_ticks()) < target_ticks) {
        float error = get_left_ticks() - get_right_ticks();
        float correction = pid_compute(&straight_pid, 0, error);

        int base_speed = -150; // negative for reverse
        int left_pwm = base_speed - correction;
        int right_pwm = base_speed + correction;

        motor_set_speed(left_pwm, right_pwm);
        delay(10);
    }

    motor_stop();
}

// placeholder versions - implement this later

float angle_diff(float target, float current) {
    float diff = target - current;
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;
    return diff;
}

void rotate_90_left() {
    // use imu.get_heading() here later
    motor_set_speed(-150,150);
    delay(300);  // placeholder
    motor_stop();
}

void rotate_90_right() {
    motor_set_speed(150,-150);
    delay(300);  // placeholder
    motor_stop();
}