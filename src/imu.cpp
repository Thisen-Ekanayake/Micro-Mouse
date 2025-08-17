#include <Wire.h>
#include <MPU6050_light.h>
#include "config.h"
#include "imu.h"

MPU6050 mpu(Wire);

unsigned long last_update = 0;
float filtered_heading = 0;
float gyro_heading = 0;
float heading_offset = 0.0f;    // stores the offset for relative heading

const float alpha = 0.98f;       // complementary filter factor (high = gyro heavy)
const int smoothing_window = 5;  // moving average window size
float heading_buffer[smoothing_window] = {0};
int buffer_index = 0;

// === Angle normalization helper ===
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

    filtered_heading = 0;
    gyro_heading = 0;
    last_update = millis();
}

void imu_update() {
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0f; // convert ms to seconds
    last_update = now;

    mpu.update();

    float accel_angle = normalize_angle(mpu.getAngleZ() - heading_offset); // accelerometer angle
    float gyro_rate = mpu.getGyroZ(); // degrees/s

    // integrate gyro
    gyro_heading += gyro_rate * dt;
    gyro_heading = normalize_angle(gyro_heading);

    // complementary filter
    float raw_filtered = alpha * gyro_heading + (1 - alpha) * accel_angle;
    raw_filtered = normalize_angle(raw_filtered);

    // simple moving average for extra smoothing
    heading_buffer[buffer_index] = raw_filtered;
    buffer_index = (buffer_index + 1) % smoothing_window;

    float sum = 0;
    for (int i = 0; i < smoothing_window; i++) sum += heading_buffer[i];
    filtered_heading = normalize_angle(sum / smoothing_window);
}

float imu_get_heading() {
    return filtered_heading;
}

float imu_get_raw_angle() {
    return mpu.getAngleZ();
}

void imu_set_heading_offset(float offset) {
    heading_offset = offset;
}