#include <Wire.h>
#include <MPU6050_light.h>
#include "config.h"
#include "imu.h"

MPU6050 mpu(Wire);

// === optional angle normalization helper ===
float normalize_angle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

void imu_init() {
    Wire.begin(MPU_SDA, MPU_SCL);   // I2C pins defined in config.h
    
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 init failed Code: ");
        Serial.println(status);
        while (1) {
            Serial.println("Halting due to IMU failure...");
            delay(1000);
        }  // stop everything
    }

    delay(1000);    // give time to stabilize

    mpu.calcOffsets();  // calibrate on flat surface
    Serial.println("IMU calibrated.");
}

float imu_get_heading() {
    return normalize_angle(mpu.getAngleZ());  // z-axis = yaw/heading in degrees
}

void imu_update() {
    mpu.update();   // must be called regularly to track angle correctly
}

float imu_get_raw_angle() {
    return mpu.getAngleZ();   // use this if want to unnormalize values
}


/*

Test Code for Setup

void loop() {
    Serial.print("Heading: ");
    Serial.println(get_heading());
    delay(200);
}

Rotate the bot on the table slowly — it should show positive or negative degrees
90° right turn ≈ +90°
90° left turn ≈ -90° (or +270° depending on wraparound)

*/