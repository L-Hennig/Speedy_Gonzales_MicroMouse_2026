/*
 * Filename: navigation.h
 * Description: C++ port of navigation.py for Arduino IDE (arduino-pico core).
 *
 * Notable differences from the Python version:
 *   - mm.start_cells (plural, used in fast_run) was a typo for the
 *     singular mm.start_cell that Micromouse actually defines - used
 *     start_cell here, since get_full_path only ever takes one start cell.
 *   - Python's RuntimeError (raised by get_full_path when the goal is
 *     unreachable) is replaced with a bool/int return code, since
 *     exceptions aren't idiomatic for embedded C++.
 *   - The Python version's move_to_next_cell() never updated mm.heading
 *     or mm.x/mm.y after turning and moving - added here so explore and
 *     fast-run actually track where the mouse is (heading is clockwise:
 *     N,E,S,W = 0,1,2,3).
 *   - read_walls() will be a no-op until Micromouse::get_tof_distance()
 *     is wired up to the real driver (currently a stub - see micromouse.h).
 *   - Only one physical button is actually readable in software (see
 *     micromouse.h) - the other is wired to the Pico's RUN pin, which
 *     hard-resets the chip. Since that reboot re-runs setup() and
 *     therefore full_reset() from scratch, "full reset for a new maze"
 *     needs no software handling at all - it's just what RUN already
 *     does. The one real button is instead read as a PRESS COUNT
 *     (single/double/triple/...) to choose between the remaining
 *     actions, instead of the old two-button single/double-press scheme.
 *     run_explore_leg() no longer polls a button to abort mid-explore,
 *     for the same reason - pressing RUN at any time already stops
 *     everything immediately, at the hardware level, regardless of what
 *     the firmware is doing.
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "micromouse.h"
#include "memory.h"
#include "flood_fill.h"

const unsigned long PRESS_WINDOW_MS = 1000;   // max gap between presses in one count
const int EXPLORE_SPEEDS[3] = { 128, 153, 178 };
const int FAST_SPEED_FIRST = 230;
const int FAST_SPEED_DEFAULT = 255;

// How many consecutive presses select each action. Add more here once
// you decide what a triple (or more) press should do.
const uint8_t PRESSES_START_RUN  = 1;
const uint8_t PRESSES_FORCE_FAST = 2;

void read_walls(Micromouse &mm, Memory &memory);
void move_to_next_cell(Micromouse &mm, Direction next_dir);
void explore_step(Micromouse &mm, Memory &memory, const Cell *goal_cells, uint8_t goal_count);

// Returns false if the run got stuck (goal unreachable) - see header note
// above about replacing Python's RuntimeError.
bool fast_run(Micromouse &mm, Memory &memory);

// Runs explore_step until solved - see header note above about why this
// no longer polls a button to abort.
void run_explore_leg(Micromouse &mm, Memory &memory, const Cell *goal_cells, uint8_t goal_count);

// Blocks until the button is pressed at least once, then counts how many
// times it's pressed in a row (gaps under PRESS_WINDOW_MS between
// releases and the next press all count as the same sequence). Returns
// once no further press arrives within the window.
uint8_t wait_for_press_count(Micromouse &mm);

// Main control loop - does not return (matches the Python `while True`).
// Call this from loop(), or just call it once from setup() since it never
// returns on its own.
void run(Micromouse &mm, Memory &memory);

#endif