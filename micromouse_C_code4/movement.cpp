/*
 * Filename: movement.cpp
 * Description: C++ port of movement.py + movement_speedy.py for Arduino IDE.
 *
 * - Turn/drive power uses mm.speed; motor 2 offset and 90 degree turn angles
 *   are tuned per speed tier.
 * - move_forward_one_cell steers with a PD controller on the side ToFs
 *   (two walls: centre between them, one wall: hold WALL_TARGET_MM, no
 *   walls: keep the wheels in step with the encoders).
 * - Loops poll for a double-press abort (abort.h) and stop early.
 * - Optional wide (arc) 90 degree turns: back up, then arc, when the wall
 *   on the far side is too close. Set ENABLE_WIDE_TURNS to false to get
 *   plain pivot turns and the original forward move length.
 * - Optional front back-off: before a 90 degree turn, if the front wall is
 *   too close, back up first, then run the side-clearance check. Set
 *   ENABLE_FRONT_BACKUP to false to disable.
 * - Optional 180 back-off: before a 180 turn, back up even when the front
 *   reading is missing or out of range. ENABLE_180_BACKUP switches it.
 */
#include "movement.h"
#include "abort.h"

// ================================================================
//  WIDE TURN + FRONT BACK-OFF SETTINGS  (everything to tweak is here)
// ================================================================

// ---- On/off switches ----
static const bool ENABLE_WIDE_TURNS   = true;   // false = plain pivot turns, no backing up for side walls
static const bool ENABLE_FRONT_BACKUP = true;   // false = never back off the front wall before a turn
static const bool ENABLE_180_BACKUP   = true;   // false = no back-up before a 180 turn
// (both false = behaves exactly like the original code)

// ---- When a wide turn triggers (side wall) ----
// Reading of the wall the rear swings toward (right turn -> left sensor,
// left turn -> right sensor). Below this = too close = use a wide turn.
static const float TURN_CLEARANCE_MM = 30.0f;   // higher = triggers more often

// ---- Shape of the wide turn ----
static const float WIDE_RADIUS_MM = 60.0f;      // arc radius AND how far it backs up first (mm).
                                                // Bigger = gentler arc, but needs more room behind.
                                                // Must be more than half the wheel spacing.
static const float WIDE_KP = 0.5f;              // keeps the inner wheel on track; lower if the arc wobbles
static const int   WIDE_MIN_INNER_POWER = 50;   // inner wheel power floor; raise if it stalls

// ---- Backing up (used by both the wide turn and the front back-off) ----
static const int BACKUP_POWER = 100;                   // motor power while reversing
static const unsigned long BACKUP_TIMEOUT_MS = 800;    // give up reversing after this long
static const unsigned long WIDE_TIMEOUT_MS = 2000;     // give up on the arc after this long

// ---- Front back-off (before a 90 degree turn) ----
// FRONT_TARGET_MM: front ToF reading when the axle is dead centre in a cell
//                  with a front wall - measure this.
// FRONT_CLEARANCE_MM: reading below this = too close = back up.
static const float FRONT_TARGET_MM = 50.0f;
static const float FRONT_CLEARANCE_MM = 40.0f;
static const float FRONT_BACKUP_MAX_MM = 20.0f; // never back up more than this

// ---- 180 turn back-up ----
// If the front reading is usable and too close, it backs up to the cell
// centre (same as the front back-off above). If the reading is missing or
// out of range, it backs up this fixed distance instead.
static const float U_TURN_BACKUP_MM = 20.0f;

// ---- Extra turn angle when backing up ----
// Added (in degrees) to the normal 90 degree turn angle ONLY when the mouse
// has just backed up (front back-off before a 90, or the 180 back-up).
// Positive = turns more, negative = turns less, 0 = same as normal.
// Not used by the wide (arc) turn, which follows its own radius.
static const float BACKUP_TURN_EXTRA_DEG = 0.0f;

// ================================================================

// ================================================================
//  90 DEGREE TURN ANGLES  (change these to make turns bigger/smaller)
// ================================================================
// The angle the mouse is told to turn for a "90 degree" turn, per speed
// tier. These are tuned values, not literal degrees (wheel slip means the
// real turn differs), so raise a number if the real turn is too small and
// lower it if too big. 180 turns are two 90s, so they change too.
//   Tier 1: speed <= 150   Tier 2: 151..200   Tier 3: > 200
static const float LEFT_90_TIER1  = 86.0f;
static const float LEFT_90_TIER2  = 82.0f;
static const float LEFT_90_TIER3  = 69.0f;

static const float RIGHT_90_TIER1 = 86.0f;
static const float RIGHT_90_TIER2 = 80.0f;
static const float RIGHT_90_TIER3 = 70.0f;

const float CORRECTION_GAIN = 4.0f;  // encoder counts per mm of error
const int MAX_CORRECTION_COUNTS = 160;

// Distance (mm) the next forward move is shortened by, set by a wide turn
// and consumed by move_forward_one_cell. Always 0 when wide turns are off.
static float forward_trim_mm = 0.0f;

// Speed-tier tuning: {motor 2 offset, left 90 angle, right 90 angle}
//   0..150   -> -9,  86, 86
//   151..200 -> -17, 82, 80
//   >200     -> -4,  69, 70
static int motor2_offset(int speed) {
  if (speed <= 150) return -9;
  if (speed <= 200) return -17;
  return -4;
}

static float left_90_angle(int speed) {
  if (speed <= 150) return LEFT_90_TIER1;
  if (speed <= 200) return LEFT_90_TIER2;
  return LEFT_90_TIER3;
}

static float right_90_angle(int speed) {
  if (speed <= 150) return RIGHT_90_TIER1;
  if (speed <= 200) return RIGHT_90_TIER2;
  return RIGHT_90_TIER3;
}

// --- PD steering for move_forward_one_cell ---
//   both walls : error = left - right
//   one wall   : error = 2 * (offset from WALL_TARGET_MM), same scale as above
//   no walls   : falls back to keeping the wheels in step with the encoders
// Positive error = mouse needs to steer left.
// (WALL_MAX_MM comes from movement.h)
static const float WALL_TARGET_MM = 40.0f;  // side reading when centred - measure this
static const float WALL_KP = 0.3f;
static const float WALL_KD = 1.5f;
static const float SYNC_KP = 0.5f;
static const float SYNC_KD = 1.5f;
static const float MOVE_MAX_CORRECTION = 40.0f;
static const unsigned long MOVE_PD_PERIOD_MS = 20;

enum SteerMode { STEER_NONE, STEER_SYNC, STEER_LEFT_WALL, STEER_RIGHT_WALL, STEER_BOTH };

void move_forward_one_cell(Micromouse &mm) {

  long start_1 = mm.motor_1.encoder_read();
  long start_2 = mm.motor_2.encoder_read();

  // A wide turn leaves the mouse forward_trim_mm ahead of where a pivot
  // turn would, so this move is shortened by that much.
  float circ = PI * WHEEL_DIAMETER_MM;
  long trim_1 = (long)(forward_trim_mm / circ * ENCODER_COUNT_PER_WHEEL_REV_1);
  long trim_2 = (long)(forward_trim_mm / circ * ENCODER_COUNT_PER_WHEEL_REV_2);
  forward_trim_mm = 0.0f;
  long cell_1 = max((long)ENCODER_COUNT_CELL_1 - trim_1, 1L);
  long cell_2 = max((long)ENCODER_COUNT_CELL_2 - trim_2, 1L);

  unsigned long start_time = millis();
  unsigned long last_pd_time = start_time;
  float last_error = 0.0f;
  SteerMode last_mode = STEER_NONE;

  // Tier offset stays as feedforward; PD handles what's left over.
  int base_1 = mm.speed;
  int base_2 = mm.speed + motor2_offset(mm.speed);

  mm.motor_1.spin_forward(base_1);
  mm.motor_2.spin_forward(base_2);

  while (true) {

    abort_poll(mm);
    if (abort_requested()) break;

    long count_1 = abs(mm.motor_1.encoder_read() - start_1);
    long count_2 = abs(mm.motor_2.encoder_read() - start_2);

    if (count_1 >= cell_1 && count_2 >= cell_2) {
      break;
    }

    unsigned long now = millis();

    if (now - start_time > MOVE_TIMEOUT_MILLISEC) {
      break;
    }

    if (now - last_pd_time >= MOVE_PD_PERIOD_MS) {
      last_pd_time = now;

      float left  = mm.get_tof_distance(2);
      float right = mm.get_tof_distance(3);
      bool left_wall  = left  > 0 && left  <= WALL_MAX_MM;
      bool right_wall = right > 0 && right <= WALL_MAX_MM;

      SteerMode mode;
      float error, kp, kd;

      if (left_wall && right_wall) {
        mode = STEER_BOTH;
        error = left - right;
        kp = WALL_KP; kd = WALL_KD;
      } else if (left_wall) {
        mode = STEER_LEFT_WALL;
        error = 2.0f * (left - WALL_TARGET_MM);
        kp = WALL_KP; kd = WALL_KD;
      } else if (right_wall) {
        mode = STEER_RIGHT_WALL;
        error = 2.0f * (WALL_TARGET_MM - right);
        kp = WALL_KP; kd = WALL_KD;
      } else {
        // No walls: keep the wheels in step. Motor 1 ahead = veering
        // left, so error goes negative (steer right).
        mode = STEER_SYNC;
        float progress_1 = (float)count_1 / cell_1;
        float progress_2 = (float)count_2 / cell_2;
        error = (progress_2 - progress_1) * cell_1;
        kp = SYNC_KP; kd = SYNC_KD;
      }

      // No derivative on the first pass after a mode change (avoids a spike)
      float d_error = (mode == last_mode) ? (error - last_error) : 0.0f;
      last_error = error;
      last_mode = mode;

      float correction = kp * error + kd * d_error;
      correction = constrain(correction, -MOVE_MAX_CORRECTION, MOVE_MAX_CORRECTION);

      // Steer left = motor 1 (right wheel) faster, motor 2 slower
      mm.motor_1.spin_forward((int)(base_1 + correction / 2.0f));
      mm.motor_2.spin_forward((int)(base_2 - correction / 2.0f));
    }
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

// ---------- Wide (arc) 90 degree turns ----------
// Used only when ENABLE_WIDE_TURNS is true.

static void back_up(Micromouse &mm, float dist_mm) {
  long start_1 = mm.motor_1.encoder_read();
  long start_2 = mm.motor_2.encoder_read();

  float circ = PI * WHEEL_DIAMETER_MM;
  long target_1 = (long)(dist_mm / circ * ENCODER_COUNT_PER_WHEEL_REV_1);
  long target_2 = (long)(dist_mm / circ * ENCODER_COUNT_PER_WHEEL_REV_2);

  unsigned long t0 = millis();
  mm.motor_1.spin_backward(BACKUP_POWER);
  mm.motor_2.spin_backward(BACKUP_POWER);

  while (true) {
    abort_poll(mm);
    if (abort_requested()) break;

    long c1 = abs(mm.motor_1.encoder_read() - start_1);
    long c2 = abs(mm.motor_2.encoder_read() - start_2);
    if (c1 >= target_1 && c2 >= target_2) break;
    if (millis() - t0 > BACKUP_TIMEOUT_MS) break;
  }

  mm.drive_stop();
  delay(100);
}

// 90 degree arc. Right turn: left wheel (motor 2) is outer.
// Left turn: right wheel (motor 1) is outer.
static void arc_turn(Micromouse &mm, bool right_turn) {
  Motor &outer = right_turn ? mm.motor_2 : mm.motor_1;
  Motor &inner = right_turn ? mm.motor_1 : mm.motor_2;
  float outer_rev = right_turn ? ENCODER_COUNT_PER_WHEEL_REV_2 : ENCODER_COUNT_PER_WHEEL_REV_1;
  float inner_rev = right_turn ? ENCODER_COUNT_PER_WHEEL_REV_1 : ENCODER_COUNT_PER_WHEEL_REV_2;

  float circ = PI * WHEEL_DIAMETER_MM;
  float outer_mm = (PI / 2.0f) * (WIDE_RADIUS_MM + WHEEL_SPACING_MM / 2.0f);
  float inner_mm = (PI / 2.0f) * (WIDE_RADIUS_MM - WHEEL_SPACING_MM / 2.0f);
  long outer_target = (long)(outer_mm / circ * outer_rev);
  long inner_target = (long)(inner_mm / circ * inner_rev);
  float ratio = (float)inner_target / (float)outer_target;  // inner counts per outer count

  long start_o = outer.encoder_read();
  long start_i = inner.encoder_read();

  int base_inner = max((int)(mm.speed * ratio), WIDE_MIN_INNER_POWER);
  unsigned long t0 = millis();

  outer.spin_forward(mm.speed);
  inner.spin_forward(base_inner);

  while (true) {
    abort_poll(mm);
    if (abort_requested()) break;

    long o = abs(outer.encoder_read() - start_o);
    long i = abs(inner.encoder_read() - start_i);

    if (o >= outer_target && i >= inner_target) break;
    if (millis() - t0 > WIDE_TIMEOUT_MS) break;

    // Keep the inner wheel's progress proportional to the outer's
    float lag = o * ratio - i;   // positive = inner wheel behind
    int inner_power = constrain((int)(base_inner + WIDE_KP * lag), 0, 255);
    inner.spin_forward(inner_power);
  }

  mm.drive_stop();
}

static void wide_turn(Micromouse &mm, bool right_turn) {
  Serial.println("Turn: too close to wall, using wide turn");
  back_up(mm, WIDE_RADIUS_MM);
  arc_turn(mm, right_turn);
  forward_trim_mm = WIDE_RADIUS_MM;   // arc ends R ahead of a pivot turn's end point
}

// far_side_sensor: the wall the rear swings toward (right turn -> left = 2,
// left turn -> right = 3)
static bool wide_turn_allowed(Micromouse &mm, uint8_t far_side_sensor) {
  if (!ENABLE_WIDE_TURNS) return false;

  // Start cell has a back wall right behind us - never back up there
  if (mm.x == mm.start_cell.x && mm.y == mm.start_cell.y) return false;
  // Arc radius must be physically valid for this wheel spacing
  if (WIDE_RADIUS_MM <= WHEEL_SPACING_MM / 2.0f + 5.0f) return false;

  float d = mm.get_tof_distance(far_side_sensor);
  return d > 0 && d < TURN_CLEARANCE_MM;
}

// ---------- Front back-off before a 90 degree turn ----------

// Returns true if it actually backed up.
static bool front_backup_if_close(Micromouse &mm) {
  if (!ENABLE_FRONT_BACKUP) return false;

  float d = mm.get_tof_distance(1);
  if (d > 0 && d < FRONT_CLEARANCE_MM) {
    float dist = constrain(FRONT_TARGET_MM - d, 0.0f, FRONT_BACKUP_MAX_MM);
    if (dist > 1.0f) {
      Serial.println("Turn: too close to front wall, backing up");
      back_up(mm, dist);
      return true;
    }
  }
  return false;
}

// Order: back off the front wall first, then check the far side wall
// (which may trigger a wide turn). If it backed up for the front wall and
// then does a normal pivot turn, BACKUP_TURN_EXTRA_DEG is added to the angle.
void turn_left(Micromouse &mm) {
  bool backed_up = front_backup_if_close(mm);
  if (wide_turn_allowed(mm, 3)) wide_turn(mm, false);
  else turn_left(mm, left_90_angle(mm.speed) + (backed_up ? BACKUP_TURN_EXTRA_DEG : 0.0f));
}

void turn_right(Micromouse &mm) {
  bool backed_up = front_backup_if_close(mm);
  if (wide_turn_allowed(mm, 2)) wide_turn(mm, true);
  else turn_right(mm, right_90_angle(mm.speed) + (backed_up ? BACKUP_TURN_EXTRA_DEG : 0.0f));
}

// Back up before a 180 so the front corners clear the wall. Works even when
// the front ToF can't give a usable reading.
// Returns true if it actually backed up.
static bool u_turn_backup(Micromouse &mm) {
  if (!ENABLE_180_BACKUP) return false;

  // The start cell has a wall right behind the mouse - never back up there
  if (mm.x == mm.start_cell.x && mm.y == mm.start_cell.y) return false;

  float d = mm.get_tof_distance(1);
  if (d > 0 && d < FRONT_CLEARANCE_MM) {
    // Usable and too close: return to the cell centre (no trim needed)
    float dist = constrain(FRONT_TARGET_MM - d, 0.0f, FRONT_BACKUP_MAX_MM);
    if (dist > 1.0f) {
      Serial.println("180: too close to front wall, backing up");
      back_up(mm, dist);
      return true;
    }
    return false;
  } else {
    // Missing / out of range: back up a fixed distance anyway. After the
    // 180 this puts the mouse that far ahead of the cell centre, so the
    // next forward move is shortened by the same amount.
    Serial.println("180: no usable front reading, backing up fixed distance");
    back_up(mm, U_TURN_BACKUP_MM);
    forward_trim_mm = U_TURN_BACKUP_MM;
    return true;
  }
}

void turn_180(Micromouse &mm) {

  bool backed_up = u_turn_backup(mm);
  if (abort_requested()) return;
  float extra = backed_up ? BACKUP_TURN_EXTRA_DEG : 0.0f;

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

  // Two plain 90 degree pivot turns in the chosen direction. The angle
  // versions are used so a 180 never triggers a wide turn (the cell behind
  // is a wall in a dead end).
  if (left_average < right_average) {

    Serial.println("180 turn: turning LEFT");
    turn_left(mm, left_90_angle(mm.speed) + extra);
    turn_left(mm, left_90_angle(mm.speed) + extra);

  } else {

    Serial.println("180 turn: turning RIGHT");
    turn_right(mm, right_90_angle(mm.speed) + extra);
    turn_right(mm, right_90_angle(mm.speed) + extra);
  }
}