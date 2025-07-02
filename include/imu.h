#ifndef IMU_H
#define IMU_H

void imu_init();            // initialize and calibrate the imu
void imu_update();          // call this regularly (every 5-10ms)
float imu_get_heading();        // returns angle in degrees
float imu_get_raw_angle();  // (optional) for direct use if normalization added

#endif