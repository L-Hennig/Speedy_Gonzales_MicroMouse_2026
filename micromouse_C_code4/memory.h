/*
 * Filename: memory.h
 * Description: C++ port of memory.py for Arduino IDE (arduino-pico core).
 *
 * Changes vs. the Python version:
 *   - Fixed-size maze (9x9, matching the Python default) instead of a
 *     dynamically-sized bytearray - avoids heap allocation on the MCU.
 *     If you need a different maze size later, change MAZE_WIDTH/HEIGHT
 *     below rather than passing a constructor argument.
 *   - Added reset() - navigation.py's full_reset() calls memory.reset(),
 *     but the Python Memory class never actually defined that method.
 *     This re-zeroes both the wall/visited data and the flood values.
 */
#ifndef MEMORY_H
#define MEMORY_H

#include <Arduino.h>

// Directions, matching memory.py's N, E, S, W = 0, 1, 2, 3
enum Direction : uint8_t {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3,
  DIR_NONE = 4   // sentinel - stands in for Python's `None`
};

// Handy for looping over all four directions, same order as the Python
// tuple (N, E, S, W).
static const Direction ALL_DIRECTIONS[4] = { NORTH, EAST, SOUTH, WEST };

// A single maze cell coordinate - shared by Memory, Micromouse (start/goal
// cells), and flood_fill.
struct Cell {
  int x;
  int y;
};

class Memory {
  public:
    static const uint8_t MAZE_WIDTH = 9;
    static const uint8_t MAZE_HEIGHT = 9;

    Memory();

    // Re-zeros walls, visited flags, and flood values - equivalent to
    // constructing a fresh Memory (added; see header comment above).
    void reset();

    void set_wall(uint8_t x, uint8_t y, Direction direction);
    bool has_wall(uint8_t x, uint8_t y, Direction direction) const;

    void mark_visited(uint8_t x, uint8_t y);
    bool is_visited(uint8_t x, uint8_t y) const;

    uint8_t get_flood(uint8_t x, uint8_t y) const;
    void set_flood(uint8_t x, uint8_t y, uint8_t value);

    // Blocks all four walls in every unvisited cell.
    void block_unvisited();

    uint8_t width() const { return MAZE_WIDTH; }
    uint8_t height() const { return MAZE_HEIGHT; }

  private:
    static const uint8_t VISITED = 16;

    // bit layout per cell: 0b000VWSEN (V = visited), same as the Python
    // version's comment.
    uint8_t _cells[MAZE_WIDTH * MAZE_HEIGHT];
    uint8_t _flood[MAZE_WIDTH * MAZE_HEIGHT];

    uint8_t _index(uint8_t x, uint8_t y) const { return y * MAZE_WIDTH + x; }
};

#endif