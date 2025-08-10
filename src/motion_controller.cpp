/*
// motion_controller.cpp
#include "motion_controller.h"
#include "config.h"

MotionController::MotionController() {
    // Initial PID values (tune these experimentally)
    straightPID = {0.8, 0.05, 0.2};
    turnPID = {4.0, 0.1, 1.0};
}

void MotionController::begin() {
    encoder_init();
    motor_init();
    imu_init();
    reset_ticks();
}

void MotionController::moveForward(float distance_cm) {
    moveLinear(distance_cm, true);
}

void MotionController::moveBackward(float distance_cm) {
    moveLinear(distance_cm, false);
}

void MotionController::moveLinear(float distance_cm, bool forward) {
    reset_ticks();
    const int32_t target_ticks = distance_cm * TICKS_PER_CM;
    const int base_speed = 150;
    float initial_heading = imu_get_heading();
    straightPID.reset();

    while (abs(get_avg_ticks()) < target_ticks) {
        // Heading correction PID
        float current_heading = imu_get_heading();
        float error = normalizeAngle(current_heading - initial_heading);
        
        // PID calculation
        float p = straightPID.kp * error;
        straightPID.integral += straightPID.ki * error;
        straightPID.integral = constrain(straightPID.integral, -200, 200);
        float d = straightPID.kd * (error - straightPID.prev_error);
        straightPID.prev_error = error;
        
        float correction = p + straightPID.integral + d;
        int left_speed = base_speed + correction;
        int right_speed = base_speed - correction;
        
        // Apply direction
        if (!forward) {
            left_speed = -left_speed;
            right_speed = -right_speed;
        }
        
        motor_set_speed(left_speed, right_speed);
        delay(10);
    }
    motor_stop();
}

void MotionController::turnLeft() {
    turnToAngle(imu_get_heading() - 90);
}

void MotionController::turnRight() {
    turnToAngle(imu_get_heading() + 90);
}

void MotionController::turnToAngle(float target_deg) {
    target_deg = normalizeAngle(target_deg);
    turnPID.reset();
    const float tolerance = 1.0f;
    
    while (true) {
        imu_update();
        float current = imu_get_heading();
        float error = normalizeAngle(target_deg - current);
        
        // Handle shortest rotation direction
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        if (fabs(error) < tolerance) break;
        
        // PID calculation for turn
        float p = turnPID.kp * error;
        turnPID.integral += turnPID.ki * error;
        turnPID.integral = constrain(turnPID.integral, -100, 100);
        float d = turnPID.kd * (error - turnPID.prev_error);
        turnPID.prev_error = error;
        
        int speed = constrain(abs(p + turnPID.integral + d), 50, 150);
        
        // Apply rotation direction
        if (error < 0) {
            motor_set_speed(-speed, speed); // Left turn
        } else {
            motor_set_speed(speed, -speed); // Right turn
        }
        delay(10);
    }
    motor_stop();
}

float MotionController::normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

void MotionController::setPID(float kp, float ki, float kd, bool straight) {
    if (straight) {
        straightPID = {kp, ki, kd};
    } else {
        turnPID = {kp, ki, kd};
    }
}
*/