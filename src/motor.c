#include <Arduino.h>
#include "motor.h"
#include "config.h"

void motor_init() {
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

void motor_set_speed(int left, int right) {
    digitalWrite(LEFT_MOTOR_DIR, left >= 0);
    digitalWrite(RIGHT_MOTOR_DIR, right >= 0);
    analogWrite(LEFT_MOTOR_PWM, abs(left));
    analogWrite(RIGHT_MOTOR_PWM, abs(right));
}

void motor_stop() {
    motor_set_speed(0, 0);
}
