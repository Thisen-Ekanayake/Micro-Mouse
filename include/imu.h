#ifndef IMU_H
#define IMU_H

void imu_init();             // Initialize and calibrate the IMU
void imu_update();           // Call this regularly (e.g., every 5–10ms)
float imu_get_heading();     // Get Z-axis angle (yaw in degrees)
float imu_get_raw_angle();   // (Optional) for direct use if normalization added
void imu_reset_heading();

#endif