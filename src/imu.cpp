#include <Wire.h>
#include <MPU6050_light.h>
#include <config.h>
#include "imu.h"

MPU6050 mpu(Wire);

void imu_init() {
    Wire.begin(MPU_SDA, MPU_SCL);   // I2C pins defined in config.h
    
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("MPU6050 init failed Code: ");
        Serial.println(status);
        while (1);  // stop everything
    }

    delay(1000);    // give time to stabilize

    mpu.calcOffsets();  // calibrate on flat surface
    Serial.println("IMU calibrated.");
}

float get_heading() {
    mpu.update();
    return mpu.getAngleZ();  // z-axis = yaw/heading in degrees
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