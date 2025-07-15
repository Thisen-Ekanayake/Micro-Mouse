#include <Arduino.h>
#include "motor.h"
#include "config.h"

/*
====================================================================================================================

Upgrade 1: PWM Clamping

Ensures PWM values don’t go outside the allowed range (0–255)
Prevents bugs if you accidentally pass values like 300 or -500
nothing to change in motor.h
add below code after includes

int clamp_pwm(int val) {
    if (val < 0) return 0;
    if (val > 255) return 255;
    return val;
}

then update analogWrite() calls like below

analogWrite(LEFT_MOTOR_PWM, clamp_pwm(abs(left)));
analogWrite(RIGHT_MOTOR_PWM, clamp_pwm(abs(right)));

====================================================================================================================

Upgrade 2: Inverted Motor Option
Lets you easily flip motor direction in software if one wheel spins backward.
If you accidentally wire a motor reversed, you don’t need to re-solder — just flip a flag.

Update config.h:

#define INVERT_LEFT_MOTOR   0  // change to 1 if direction is reversed
#define INVERT_RIGHT_MOTOR  0

Update motor.c:

#include "config.h"

Modify direction logic:

bool left_dir = (left >= 0);
bool right_dir = (right >= 0);

if (INVERT_LEFT_MOTOR)  left_dir  = !left_dir;
if (INVERT_RIGHT_MOTOR) right_dir = !right_dir;

digitalWrite(LEFT_MOTOR_DIR, left_dir);
digitalWrite(RIGHT_MOTOR_DIR, right_dir);

====================================================================================================================

Upgrade 3: Soft Start / Speed Ramping
This gives smoother acceleration and reduces jerk.

What it solves:
Prevents sudden wheel jerks
Helps with mechanical stability (especially when using gear motors)

Update motor.h:
Add this function declaration:
void motor_set_speed_ramp(int left_target, int right_target, int ramp_step, int delay_ms);


Update motor.c:

Add this function implementation:

void motor_set_speed_ramp(int left_target, int right_target, int ramp_step, int delay_ms) {
    static int left_current = 0;
    static int right_current = 0;

    while (left_current != left_target || right_current != right_target) {
        if (left_current < left_target) left_current += ramp_step;
        else if (left_current > left_target) left_current -= ramp_step;

        if (right_current < right_target) right_current += ramp_step;
        else if (right_current > right_target) right_current -= ramp_step;

        motor_set_speed(left_current, right_current);
        delay(delay_ms);
    }
}

Example usage:

motor_set_speed_ramp(0, 0, 10, 5);         // slow stop
motor_set_speed_ramp(150, 150, 10, 5);     // soft start

You can tune ramp_step (e.g. 10) and delay_ms (e.g. 5 ms) to control ramp smoothness.

====================================================================================================================

Bonus: Optional PWM Precision with ledcWrite() (ESP32-Specific)
Only if you ever need higher PWM resolution (not needed now, but here’s how):

In motor_init():

ledcSetup(0, 20000, 8); // Channel 0, 20kHz, 8-bit
ledcAttachPin(LEFT_MOTOR_PWM, 0);

ledcSetup(1, 20000, 8);
ledcAttachPin(RIGHT_MOTOR_PWM, 1);

In motor_set_speed():
ledcWrite(0, clamp_pwm(abs(left)));
ledcWrite(1, clamp_pwm(abs(right)));

====================================================================================================================
*/

// === clamp pwm to [0,255] ===
int clamp_pwm(int val) {
    if (val < 0) return 0;
    if (val > 255) return 255;
    return val;
}

void motor_init() {
    // setup motor control pins
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);

    // setup pwm on IN1 pins only (IN2 will be digital high/low)
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTOIN);
    ledcAttachPin(LEFT_MOTOR_IN1, PWM_CHANNEL_LEFT);

    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTOIN);
    ledcAttachPin(RIGHT_MOTOR_IN1, PWM_CHANNEL_RIGHT);
}

// sets signed speed: -255 (reverse) to +255 (forward)
void motor_set_speed(int left, int right) {

    // left motor
    if (left >= 0) {
        digitalWrite(LEFT_MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL_LEFT, clamp_pwm(left));
    } else {
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_LEFT, clamp_pwm(-left));
    }

    // right motor
    if (right >= 0) {
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL_RIGHT, clamp_pwm(right));
    } else {
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_RIGHT, clamp_pwm(-right));
    }
}

void motor_stop() {
    motor_set_speed(0, 0);
}

// === smooth ramping ===
void motor_set_speed_ramp(int left_target, int right_target, int ramp_step, int delay_ms) {
    static int left_current = 0;
    static int right_current = 0;

    while (left_current != left_target || right_current != right_target) {
        if (left_current < left_target) left_current += ramp_step;
        else if (left_current > left_target) left_current -= ramp_step;

        if (right_current < right_target) right_current += ramp_step;
        else if (right_current > right_target) right_current -= ramp_step;

        motor_set_speed(left_current, right_current);
        delay(delay_ms);
    }
}
