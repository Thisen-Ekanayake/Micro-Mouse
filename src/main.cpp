#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "button.h"
#include "maze.h"
#include "floodfill.h"
#include "motion.h"

/*
void setup() {
    Serial.begin(115200);
    motor_init();
    ir_sensor_init();
    imu_init();
    button_init();
    maze_init();

    Serial.println("Micromouse Ready");
    while (!button_pressed()) delay(100);
}

void loop() {
    floodfill_init();

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

    =========================================================================

    imu_update();  // Call this as often as possible (every ~5–10ms)
    float heading = imu_get_heading();
    Serial.print("Heading: ");
    Serial.println(heading);
    delay(50);

    =========================================================================

    */
//}
//*/



/*
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 2 as an output.
  pinMode(2, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(2, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(2, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second
}
*/

void setup() {
  Serial.begin(115200);
  Serial.println("=== Motor Test Start ===");

  motor_init();
  delay(1000);
}

void loop() {
  Serial.println("Moving Forward...");
  motor_set_speed(150, 150);
  delay(1000);

  Serial.println("Moving Backward...");
  motor_set_speed(-150, -150);
  delay(1000);

  Serial.println("Turning Left...");
  motor_set_speed(-150, 150);
  delay(1000);

  Serial.println("Turning Right...");
  motor_set_speed(150, -150);
  delay(1000);

  Serial.println("Stopping...");
  motor_stop();
  delay(2000);

  // loop forever for now
  while(true);
}