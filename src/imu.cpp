#include <Wire.h>
#include <MPU6050_light.h>
#include <config.h>

MPU6050 mpu(Wire);

void imu_init() {
    Wire.begin(MPU_SDA, MPU_SCL);
    mpu.begin();
    mpu.calcOffsets();
}

float get_heading() {
    mpu.update();
    return mpu.getAngleZ(); // returns heading in degrees
}
