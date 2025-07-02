#ifndef CONFIG_H
#define CONFIG_H

// ==== Encoder Pins ====
#define ENCODER_LEFT_PIN    35
#define ENCODER_RIGHT_PIN   36

// ==== Motor Pins ====
#define LEFT_MOTOR_PWM      25
#define RIGHT_MOTOR_PWM     26
#define LEFT_MOTOR_DIR      27
#define RIGHT_MOTOR_DIR     14

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

#endif