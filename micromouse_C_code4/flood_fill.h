/*
 * Filename: flood_fill.h
 * Description: C++ port of flood_fill.py for Arduino IDE (arduino-pico core).
 *
 * Uses a fixed-size array as the BFS queue instead of Python's list, since
 * each cell is only ever enqueued once - sized to the maximum maze size,
 * so no dynamic allocation is needed.
 */
#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include "memory.h"

// Recomputes every cell's flood value as its shortest distance (in cells)
// to the nearest goal cell, given the walls currently known.
void update_flood(Memory &memory, const Cell *goal_cells, uint8_t goal_count);

// Returns the direction from (x, y) with the lowest neighbouring flood
// value, or DIR_NONE if no move is possible (walled in on every side).
// Note: ties are resolved by direction order (N, E, S, W) - the Python
// version had a "add preference when tied, if wanted" comment left
// unimplemented, so this carries that over as-is rather than guessing.
Direction get_next_move(Memory &memory, int x, int y);

bool is_solved(int x, int y, const Cell *goal_cells, uint8_t goal_count);

// Moves one cell in the given direction. Same (x, y+1)=N convention as
// the Python version.
Cell step_cell(int x, int y, Direction direction);

// Fills `path` with the direction to take at each step from `start` to
// the nearest reachable goal cell. `path` must have room for at least
// MAX_PATH_LEN entries. Returns the number of steps written, or -1 if
// the goal is unreachable (matches the Python version's RuntimeError).
static const uint16_t MAX_PATH_LEN = Memory::MAZE_WIDTH * Memory::MAZE_HEIGHT;
int get_full_path(Memory &memory, Cell start, const Cell *goal_cells,
                   uint8_t goal_count, Direction *path);

#endif