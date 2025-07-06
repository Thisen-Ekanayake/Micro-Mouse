#include <Arduino.h>
#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "config.h"

PID straight_pid = {0.5, 0.01, 0.1, 0, 0}; // tune this later

void move_forward_cm(float cm) {
    int target_ticks = cm * TICKS_PER_CM;
    reset_ticks();
    pid_reset(&straight_pid);

    while (get_avg_ticks() < target_ticks) {
        float error = get_left_ticks() - get_right_ticks();
        float correction = pid_compute(&straight_pid, 0, error);

        int base_speed = 150;
        int left_pwm = constrain(base_speed - correction, 0, 255);
        int right_pwm = constrain(base_speed + correction, 0, 255);

        motor_set_speed(left_pwm, right_pwm);
        delay(10);
    }

    motor_stop();
}

// placeholder versions - implement this later

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