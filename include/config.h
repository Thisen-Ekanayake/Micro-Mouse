#ifndef CONFIG_H
#define CONFIG_H

// ==== Encoder Pins ====
#define ENCODER_LEFT_A      35
#define ENCODER_LEFT_B      34
#define ENCODER_RIGHT_A     32
#define ENCODER_RIGHT_B     33

// ==== Motor Control Pins (DRV8833) ====
// Left Motor (connected to Ain1 and Ain2)
#define LEFT_MOTOR_IN1      18  // Ain1
#define LEFT_MOTOR_IN2      19  // Ain2

// Right Motor (connected to Bin1 and Bin2)
#define RIGHT_MOTOR_IN1     21  // Bin1
#define RIGHT_MOTOR_IN2     22  // Bin2


// ==== IR Sensors ====
#define IR_SENSOR_LEFT      32
#define IR_SENSOR_FRONT     33
#define IR_SENSOR_RIGHT     34
 
#define IR_THRESHOLD_FRONT  400
#define IR_THRESHOLD_LEFT   400
#define IR_THRESHOLD_RIGHT  400

// ==== IMU (MPU6050) ====
#define MPU_SDA             21
#define MPU_SCL             22

// ==== Button ====
#define BUTTON_PIN          4

// ==== Motion Constants ====
#define TICKS_PER_CM        10.0f   // temporary guess - adjust after testing
#define CELL_SIZE_CM        18
#define MAZE_SIZE           16

// ==== Battery (if monitored) ====
#define BATTERY_FULL_VOLTS  12.6
#define BATTERY_LOW_VOLTS   10.5

// ==== PID ====
#define PID_INTEGRAL_MAX    500.0f
#define PID_INTEGRAL_MIN   -500.0f

#endif