/*
 * Filename: movement.cpp
 * Description: C++ port of movement.py + movement_speedy.py for Arduino IDE.
 *
 * Turn/drive power uses mm.speed. Motor 2 offset and the turn angles are
 * tuned per speed tier. Loops poll for a double-press abort (abort.h) and
 * stop early when one is requested.
 */
#include "movement.h"
#include "abort.h"

const float CORRECTION_GAIN = 4.0f;  // encoder counts per mm of error
const int MAX_CORRECTION_COUNTS = 160;

// Speed-tier tuning: {motor 2 offset, left 90 angle, right 90 angle}
//   0..150   -> -9,  86, 86
//   151..200 -> -17, 82, 80
//   >200     -> -4,  69, 70
static int motor2_offset(int speed) {
  if (speed <= 150) return -9.25;
  if (speed <= 200) return -17;
  return -4;
}

static float left_90_angle(int speed) {
  if (speed <= 150) return 86.0f;
  if (speed <= 200) return 82.0f;
  return 69.0f;
}

static float right_90_angle(int speed) {
  if (speed <= 150) return 86.0f;
  if (speed <= 200) return 80.0f;
  return 70.0f;
}

// 180 turn angles per speed tier (all 220 for now, tune later)
static float left_180_angle(int speed) {
  if (speed <= 150) return 220.0f;
  if (speed <= 200) return 220.0f;
  return 220.0f;
}

static float right_180_angle(int speed) {
  if (speed <= 150) return 220.0f;
  if (speed <= 200) return 220.0f;
  return 220.0f;
}

void move_forward_one_cell(Micromouse &mm) {

  long start_1 = mm.motor_1.encoder_read();
  long start_2 = mm.motor_2.encoder_read();

  unsigned long start_time = millis();

  mm.motor_1.spin_forward(mm.speed);
  mm.motor_2.spin_forward(mm.speed + motor2_offset(mm.speed));

  while (true) {

    abort_poll(mm);
    if (abort_requested()) break;

    long count_1 = abs(mm.motor_1.encoder_read() - start_1);
    long count_2 = abs(mm.motor_2.encoder_read() - start_2);

    if (count_1 >= ENCODER_COUNT_CELL_1 &&
        count_2 >= ENCODER_COUNT_CELL_2) {
      break;
    }

    if (millis() - start_time > MOVE_TIMEOUT_MILLISEC) {
      break;
    }

    mm.motor_1.spin_forward(mm.speed);
    mm.motor_2.spin_forward(mm.speed + motor2_offset(mm.speed));
  }

  mm.drive_stop();
}


void move_x_cells(Micromouse &mm, int cells) {
  for (int i = 0; i < cells; i++) {
    move_forward_one_cell(mm);
  }
}

void turn_left(Micromouse &mm, float angle_deg) {
  long start_1 = mm.motor_1.encoder_read();
  long start_2 = mm.motor_2.encoder_read();

  // Sector of the circle traced by the wheel spacing, for the given angle.
  float turn_distance = PI * WHEEL_SPACING_MM * angle_deg / 360.0f;
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  long turn_1 = (long)(turn_distance / wheel_circumference * ENCODER_COUNT_PER_WHEEL_REV_1);
  long turn_2 = (long)(turn_distance / wheel_circumference * ENCODER_COUNT_PER_WHEEL_REV_2);

  mm.motor_2.spin_backward(mm.speed);
  mm.motor_1.spin_forward(mm.speed);

  while (true) {
    abort_poll(mm);
    if (abort_requested()) break;

    long count_1 = abs(mm.motor_1.encoder_read() - start_1);
    long count_2 = abs(mm.motor_2.encoder_read() - start_2);
    if (count_1 >= turn_1 && count_2 >= turn_2) break;
  }

  mm.drive_stop();
}

void turn_right(Micromouse &mm, float angle_deg) {
  long start_1 = mm.motor_1.encoder_read();
  long start_2 = mm.motor_2.encoder_read();

  float turn_distance = PI * WHEEL_SPACING_MM * angle_deg / 360.0f;
  float wheel_circumference = PI * WHEEL_DIAMETER_MM;
  long turn_1 = (long)(turn_distance / wheel_circumference * ENCODER_COUNT_PER_WHEEL_REV_1);
  long turn_2 = (long)(turn_distance / wheel_circumference * ENCODER_COUNT_PER_WHEEL_REV_2);

  mm.motor_1.spin_backward(mm.speed);
  mm.motor_2.spin_forward(mm.speed);

  while (true) {
    abort_poll(mm);
    if (abort_requested()) break;

    long count_1 = abs(mm.motor_1.encoder_read() - start_1);
    long count_2 = abs(mm.motor_2.encoder_read() - start_2);
    if (count_1 >= turn_1 && count_2 >= turn_2) break;
  }

  mm.drive_stop();
}

void turn_left(Micromouse &mm) {
  turn_left(mm, left_90_angle(mm.speed));
}

void turn_right(Micromouse &mm) {
  turn_right(mm, right_90_angle(mm.speed));
}

void turn_180(Micromouse &mm) {

  const unsigned long MEASURE_TIME = 200;

  unsigned long start_time = millis();

  float left_sum = 0;
  float right_sum = 0;
  int samples = 0;

  // Measure left and right for 0.2 seconds
  while (millis() - start_time < MEASURE_TIME) {

    abort_poll(mm);

    left_sum += mm.get_tof_distance(2);
    right_sum += mm.get_tof_distance(3);

    samples++;

    delay(10);
  }

  if (abort_requested()) return;

  float left_average = left_sum / samples;
  float right_average = right_sum / samples;

  Serial.print("180 turn | Left: ");
  Serial.print(left_average);
  Serial.print(" mm | Right: ");
  Serial.print(right_average);
  Serial.println(" mm");

  // Turn towards the side with more space
  if (left_average < right_average) {

    Serial.println("180 turn: turning LEFT");
    turn_left(mm, left_180_angle(mm.speed));

  } else {

    Serial.println("180 turn: turning RIGHT");
    turn_right(mm, right_180_angle(mm.speed));
  }
}