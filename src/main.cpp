#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "button.h"
#include "maze.h"
#include "floodfill.h"

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
}