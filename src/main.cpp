#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "button.h"
#include "maze.h"
#include "floodfill.h"
#include "motion.h"
#include "encoder.h"
#include "display.h"
#include "power.h"
#include "tof_sensor.h"

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


/*void setup() {
  Serial.begin(115200);
  Serial.println("=== Motor Test Start ===");

  motor_init();
  delay(1000);
}

void loop() {
  delay(5000);
  Serial.println("Moving Forward...");
  motor_set_speed(100, 100);
  delay(1000);

  /*Serial.println("Moving Backward...");
  motor_set_speed(-100, -100);
  delay(1000);

  Serial.println("Turning Left...");
  motor_set_speed(-100, 100);
  delay(1000);

  Serial.println("Turning Right...");
  motor_set_speed(100, -100);
  delay(1000);*/

  /*Serial.println("Stopping...");
  motor_stop();
  delay(5000);

  // loop forever for now
  // while(true);
}*/

// === Placeholder Constants ===
const float WHEEL_DIAMETER_MM = 43.0; // <-- Set this later
const int TICKS_PER_REVOLUTION = 1440; // depends on your encoder spec

float ticks_to_distance_mm(int ticks) {
  // Circumference = π * diameter
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  return (wheel_circumference * ticks) / TICKS_PER_REVOLUTION;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Motor + Encoder Test Start ===");

  motor_init();
  encoder_init();  // Initialize encoder
  reset_ticks();   // Reset tick count to 0
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  initDisplay();
  if (!initPowerSensor()) {
    Serial.println("INA3221 not found");
    while (true) delay(10);
  }

  initToFSensor();

}

void loop() {

  float v0 = getBusVoltage(0), i0 = getCurrent(0);
  float v1 = getBusVoltage(1), i1 = getCurrent(1);
  bool charging = v1 >= 12.0;

  Serial.printf("OUT: %.2fV, %.2fmA\n", v0, i0 * 1000);
  Serial.printf("BAT: %.2fV, %.2fmA\n", v1, i1 * 1000);
  Serial.println(charging ? "Charging" : "Disconnected");

  uint16_t d1, d2, d3;
  String left, right, front;
  readToF(d1, d2, d3, left, right, front);

  updateDisplay(v0, i0, v1, i1, left, right, front, charging);

  delay(500);
  Serial.println("Moving Forward...");
  reset_ticks(); // Reset before movement
  motor_set_speed(100, 100);
  delay(3000); // Move forward for 1 second

  motor_stop();
  Serial.println("Stopped.");

  // Get ticks and calculate distance
  int left_tick = get_left_ticks();
  int right_tick = get_right_ticks();
  int avg = get_avg_ticks();
  float distance_mm = ticks_to_distance_mm(avg);

  Serial.print("Left Ticks: "); Serial.println(left_tick);
  Serial.print("Right Ticks: "); Serial.println(right_tick);
  Serial.print("Average Ticks: "); Serial.println(avg);
  Serial.print("Distance Traveled (mm): "); Serial.println(distance_mm);

  delay(5000); // Pause before next loop
}
