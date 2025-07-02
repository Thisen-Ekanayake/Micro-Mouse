#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "button.h"
#include "maze.h"
#include "floodfill.h"
#include "motion.h"

void setup() {
    Serial.begin(115200);
    motor_init();
    ir_sensor_init();
    imu_init();
    button_init();
    init_maze();

    Serial.println("Micromouse Ready");
    while (!button_pressed()) delay(100);
}

void loop() {
    flood_fill();

    /*

    testing
    move_forward_cm(18);   // move 1 cell
    rotate_90_left();

    How to Calibrate it (Once You Have Hardware)
    Do this:

    Move the micromouse exactly 18 cm forward using move_forward_cm(18);
    Print the number of encoder ticks:

    Serial.println(get_left_ticks());

    Compute:
    TICKS_PER_CM = (left_ticks + right_ticks) / 2.0 / 18.0
    
    Then update the config.h value accordingly.

    */


}