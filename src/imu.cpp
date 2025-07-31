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

/*
void auto_tune_gyro_z_offset() {
    Serial.println("Starting extended auto-tuning for gyro Z offset...");

    float best_drift = 10000.0f;
    int best_z_offset = 0;

    for (int z_offset = -200; z_offset <= 200; z_offset += 1) {
        mpu.setGyroOffsets(0, 0, z_offset);
        delay(1000);  // Stabilize

        mpu.update();
        float initial = mpu.getAngleZ();

        unsigned long start_time = millis();
        while (millis() - start_time < 5000) {
            mpu.update();
            delay(10);
        }

        float final = mpu.getAngleZ();
        float drift = abs(final - initial);
        float rate = (final - initial) / 5.0;  // degrees/sec drift

        Serial.print("Z offset: "); Serial.print(z_offset);
        Serial.print(" | Drift: "); Serial.print(drift);
        Serial.print(" | Rate: "); Serial.println(rate);

        if (drift < best_drift) {
            best_drift = drift;
            best_z_offset = z_offset;
        }
    }

    Serial.println("==== Auto-tuning complete ====");
    Serial.print("Best Z offset: ");
    Serial.print(best_z_offset);
    Serial.print(" | Drift: ");
    Serial.println(best_drift);

    mpu.setGyroOffsets(0, 0, best_z_offset);
}
*/

void imu_init() {
    Wire.begin(MPU_SDA, MPU_SCL);
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 init failed! Code: ");
        Serial.println(status);
        while (1) delay(1000);
    }

    Serial.println("Stabilizing MPU6050...");
    delay(2000);  // Let it warm up

    Serial.println("Calibrating...");
    mpu.calcOffsets(true, true);
    Serial.println("Calibration done.");

    Serial.print("Gyro Offsets: ");
    Serial.print(mpu.getGyroXoffset()); Serial.print(", ");
    Serial.print(mpu.getGyroYoffset()); Serial.print(", ");
    Serial.println(mpu.getGyroZoffset());
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