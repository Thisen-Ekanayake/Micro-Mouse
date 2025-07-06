#include <Wire.h>
#include <MPU6050_light.h>
#include "config.h"
#include "imu.h"

MPU6050 mpu(Wire);

// === Optional angle normalization helper ===
float normalize_angle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

void imu_init() {
    Wire.begin(MPU_SDA, MPU_SCL);   // I2C pins defined in config.h

    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 init failed! Code: ");
        Serial.println(status);
        while (1) {
            Serial.println("Halting due to IMU failure...");
            delay(1000);
        }
    }

    delay(1000);            // Give time to stabilize
    mpu.calcOffsets();      // Calibrate on flat surface
    Serial.println("IMU calibrated.");
}

void imu_update() {
    mpu.update();  // Must be called regularly to track angle correctly
}

float imu_get_heading() {
    return normalize_angle(mpu.getAngleZ());
}

float imu_get_raw_angle() {
    return mpu.getAngleZ();  // Use this if you want unnormalized values
}

/*
test code for main

void loop() {
    imu_update();  // Call this as often as possible (every ~5–10ms)
    float heading = imu_get_heading();
    Serial.print("Heading: ");
    Serial.println(heading);
    delay(50);
}
*/