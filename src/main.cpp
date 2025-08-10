#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "motor.h"
// #include "ir_sensor.h"
#include "imu.h"
#include "button.h"
#include "maze.h"
// #include "floodfill.h"
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

/*
// === Constants ===
const float WHEEL_DIAMETER_MM = 43.0;
const int TICKS_PER_REVOLUTION = 1440;
const unsigned long tofInterval = 100; // ms
const unsigned long motionInterval = 3000; // 3 sec
//float correction_factor = 640.0 / 371.0;

// === Timekeeping ===
unsigned long lastToFRead = 0;
unsigned long movementTimer = 0;

// === Motion State ===
bool isMoving = false;

// === Conversion ===
float ticks_to_distance_mm(int ticks) {
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  return (wheel_circumference * ticks) / TICKS_PER_REVOLUTION; //* correction_factor;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Motor + Encoder Looping Test Start ===");

  motor_init();
  encoder_init();
  reset_ticks();

  Wire.begin(SDA_PIN, SCL_PIN);

  initDisplay();

  if (!initPowerSensor()) {
    Serial.println("INA3221 not found");
    while (true) delay(10);
  }

  initToFSensor();

  // Start in stopped state
  isMoving = false;
  motor_stop();
  movementTimer = millis();
}

void loop() {
  unsigned long now = millis();

  // === Periodic ToF + Display Update ===
  if (now - lastToFRead >= tofInterval) {
    uint16_t d1, d2, d3;
    String left, right, front;
    readToF(d1, d2, d3, left, right, front);

    float v0 = getBusVoltage(0), i0 = getCurrent(0);
    float v1 = getBusVoltage(1), i1 = getCurrent(1);
    bool charging = v1 >= 12.0;

    updateDisplay(v0, i0, v1, i1, left, right, front, charging);

    lastToFRead = now;
  }

  // === Motion State Machine ===
  if (now - movementTimer >= motionInterval) {
    if (isMoving) {
      // Stop the motors
      motor_stop();
      Serial.println("Stopped.");
      
      // Show encoder reading & distance
      int left_tick = get_left_ticks();
      int right_tick = get_right_ticks();
      int avg = get_avg_ticks();
      float distance_mm = ticks_to_distance_mm(avg);

      Serial.print("Left Ticks: "); Serial.println(left_tick);
      Serial.print("Right Ticks: "); Serial.println(right_tick);
      Serial.print("Average Ticks: "); Serial.println(avg);
      Serial.print("Distance Traveled (mm): "); Serial.println(distance_mm);

      reset_ticks();
    } else {
      // Start moving forward
      motor_set_speed(100, 100);
      Serial.println("Moving Forward...");
    }

    // Toggle state and reset timer
    isMoving = !isMoving;
    movementTimer = now;
  }
}

/*
void setup() {
  Serial.begin(115200);
  delay(1000); // give serial time to connect
  Serial.println("Starting IMU Test...");

  imu_init();
}

void loop() {
  imu_update();

  float heading = imu_get_heading();
  Serial.print("Heading: ");
  Serial.println(heading);

  delay(50);
}
*/
/*
unsigned long last_update = 0;
float previous_heading = 0;
const float alpha = 0.98;  // Low-pass filter coefficient

void setup() {
    Serial.begin(115200);
    imu_init();  // Initialize and calibrate MPU6050
    delay(1000); // Give time for everything to stabilize
    Serial.println("IMU test started...");
}

void loop() {
    if (millis() - last_update >= 5) {
        imu_update();

        float raw = imu_get_heading();
        float filtered = alpha * previous_heading + (1 - alpha) * raw;
        previous_heading = filtered;

        // Print raw and filtered values for comparison
        Serial.print("Raw: ");
        Serial.print(raw);
        Serial.print("  |  Filtered: ");
        Serial.println(filtered);

        last_update = millis();
    }
}
*/

using namespace maze;

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // wait for serial on some boards

  Serial.println("Maze Solver Starting...");

  // Initialize motors, encoders, IMU, maze sensors & logic
  motor_init();
  encoder_init();
  imu_init();
  maze::begin();

  delay(1000);  // stabilize sensors
}

void printPath(const std::vector<Pose>& path) {
  Serial.println("Computed shortest path:");
  for (size_t i = 0; i < path.size(); i++) {
    Serial.printf("Step %d: x=%d, y=%d, dir=%d\n", int(i), path[i].x, path[i].y, int(path[i].d));
  }
}

void loop() {
  Serial.println("Starting maze exploration...");
  startExploration();
  Serial.println("Exploration complete.");

  // Example goal cell (adjust as per your maze goal)
  const int goalX = 7;
  const int goalY = 7;

  Serial.printf("Computing shortest path from (0,0) to (%d,%d)...\n", goalX, goalY);
  std::vector<Pose> path = computeShortestPath(0, 0, goalX, goalY);

  if (path.empty()) {
    Serial.println("No path found!");
    while(1) delay(1000);  // halt
  }

  printPath(path);

  Serial.println("Executing shortest path...");
  executePath(path);

  Serial.println("Path execution complete.");

  // Halt after one run
  while(1) delay(1000);
}
