#ifndef CONFIG_H
#define CONFIG_H

// ==== Encoder Pins ====
#define ENCODER_LEFT_A      34
#define ENCODER_LEFT_B      35
#define ENCODER_RIGHT_A     33
#define ENCODER_RIGHT_B     32


// ==== Motor Control Pins (DRV8833) ====
// Left Motor (connected to Ain1 and Ain2)
#define LEFT_MOTOR_IN1      19  // Ain1
#define LEFT_MOTOR_IN2      18  // Ain2

// Right Motor (connected to Bin1 and Bin2)
#define RIGHT_MOTOR_IN1     26  // Bin1 26
#define RIGHT_MOTOR_IN2     27  // Bin2 27


// ==== PWM ====
#define PWM_CHANNEL_LEFT    0
#define PWM_CHANNEL_RIGHT   1
#define PWM_FREQ            1000
#define PWM_RESOLUTION      8   // 0 - 255

/*
// ==== IR Sensors ====
#define IR_SENSOR_LEFT      32
#define IR_SENSOR_FRONT     33
#define IR_SENSOR_RIGHT     34
 
#define IR_THRESHOLD_FRONT  400
#define IR_THRESHOLD_LEFT   400
#define IR_THRESHOLD_RIGHT  400
*/

// ==== IMU (MPU6050) ====
#define MPU_SDA             21
#define MPU_SCL             22


// ==== Button ====
#define BUTTON_PIN          4


// ==== Motion Constants ====
#define TICKS_PER_CM        36.418578f   // temporary guess - adjust after testing
#define CELL_SIZE_CM        18
#define MAZE_SIZE           16


// ==== Battery (if monitored) ====
#define BATTERY_FULL_VOLTS  12.6
#define BATTERY_LOW_VOLTS   10.5


// ==== PID ====
#define PID_INTEGRAL_MAX    500.0f
#define PID_INTEGRAL_MIN   -500.0f


// ==== OLED settings ====
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_RESET          -1
#define SCREEN_ADDRESS      0x3C


// ==== I2C pins for ESP32 ====
#define SDA_PIN             21
#define SCL_PIN             22


// ==== ToF XSHUT Pins ====
#define XSHUT1              25
#define XSHUT2              2
#define XSHUT3              4

#endif