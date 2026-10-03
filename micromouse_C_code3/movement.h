/*
 * Filename: movement.h
 * Description: C++ port of movement.py + movement_speedy.py for Arduino IDE.
 *
 * movement.py's fixed 90-degree turn_left/turn_right and movement_speedy.py's
 * arbitrary-angle turn_left/turn_right(angle) are the same math with angle
 * hardcoded to 90 - merged here as overloads (turn_left(mm) just calls
 * turn_left(mm, 90)) instead of keeping two near-duplicate implementations.
 */
#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "micromouse.h"

// Wheel/encoder geometry constants (from movement.py / movement_speedy.py)
//To do: turn each encoder and find the counts for first 2 constants.
const int ENCODER_COUNT_PER_WHEEL_REV_1 = 1056;   // motor_1
const int ENCODER_COUNT_PER_WHEEL_REV_2 = 1064;   // motor_2
const float WHEEL_DIAMETER_MM = 44.0f;
const float WHEEL_SPACING_MM  = 74.0f;
const float CELL_SIZE_MM      = 160.0f;

// Encoder counts for one cell for each wheel
const long ENCODER_COUNT_CELL_1 =
    (long)(CELL_SIZE_MM / (PI * WHEEL_DIAMETER_MM) * ENCODER_COUNT_PER_WHEEL_REV_1);
const long ENCODER_COUNT_CELL_2 =
    (long)(CELL_SIZE_MM / (PI * WHEEL_DIAMETER_MM) * ENCODER_COUNT_PER_WHEEL_REV_2);

const unsigned long MOVE_TIMEOUT_MILLISEC = 7000;   // stop moving forward if encoder count hasnt been reached after 7s.

// Centering
const float STEERING_STRENGTH = 1.0f;     // bigger = steers back to the middle harder
const float WALL_MAX_MM       = 120.0f;   // a side reading below this counts as a wall
const float WALL_GAP_MM       = 168.0f;   // clear space between the walls
const float MOUSE_WIDTH_MM    = 107.0f;    // MEASURE!!!!!!!!! TBD
const float TARGET_SIDE_MM    = (WALL_GAP_MM - MOUSE_WIDTH_MM) / 2.0f;   // ideal distance to a side wall
const int MAX_SPEED_CHANGE = 15;


void move_forward_one_cell(Micromouse &mm);
void move_x_cells(Micromouse &mm, int cells);

// Fixed 90-degree turns (delegate to the angle versions below)
void turn_left(Micromouse &mm);
void turn_right(Micromouse &mm);
void turn_180(Micromouse &mm);

// Arbitrary-angle turns, ported from movement_speedy.py
void turn_left(Micromouse &mm, float angle_deg);
void turn_right(Micromouse &mm, float angle_deg);

#endif
