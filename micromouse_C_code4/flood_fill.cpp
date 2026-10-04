/*
 * Filename: flood_fill.cpp
 * Description: C++ port of flood_fill.py for Arduino IDE (arduino-pico core).
 */
#include "flood_fill.h"

Cell step_cell(int x, int y, Direction direction) {
  switch (direction) {
    case NORTH: return { x, y + 1 };
    case SOUTH: return { x, y - 1 };
    case EAST:  return { x + 1, y };
    case WEST:  return { x - 1, y };
    default:    return { x, y };   // unreachable given a valid direction
  }
}

static bool in_bounds(const Memory &memory, int x, int y) {
  return x >= 0 && x < memory.width() && y >= 0 && y < memory.height();
}

void update_flood(Memory &memory, const Cell *goal_cells, uint8_t goal_count) {
  // Reset flood info to "unknown" distance (255 = not yet reached)
  for (uint8_t y = 0; y < memory.height(); y++) {
    for (uint8_t x = 0; x < memory.width(); x++) {
      memory.set_flood(x, y, 255);
    }
  }

  // Fixed-size FIFO queue - each cell is enqueued at most once, so
  // MAX_PATH_LEN (width*height) is always enough room.
  Cell queue[MAX_PATH_LEN];
  uint16_t head = 0, tail = 0;

  for (uint8_t i = 0; i < goal_count; i++) {
    memory.set_flood(goal_cells[i].x, goal_cells[i].y, 0);
    queue[tail++] = goal_cells[i];
  }

  while (head < tail) {
    Cell c = queue[head++];
    uint8_t current_value = memory.get_flood(c.x, c.y);

    for (uint8_t i = 0; i < 4; i++) {
      Direction direction = ALL_DIRECTIONS[i];
      if (memory.has_wall(c.x, c.y, direction)) continue;

      Cell n = step_cell(c.x, c.y, direction);
      if (!in_bounds(memory, n.x, n.y)) continue;
      if (memory.get_flood(n.x, n.y) != 255) continue;

      memory.set_flood(n.x, n.y, current_value + 1);
      queue[tail++] = n;
    }
  }
}

// Tie-break order: earlier = preferred when flood values are equal
static const Direction MOVE_PRIORITY[4] = {WEST, NORTH, SOUTH, EAST};

Direction get_next_move(Memory &memory, int x, int y) {
  Direction best_direction = DIR_NONE;
  uint16_t best_value = 256;

  for (uint8_t i = 0; i < 4; i++) {
    Direction direction = MOVE_PRIORITY[i];
    if (memory.has_wall(x, y, direction)) continue;

    Cell n = step_cell(x, y, direction);
    if (!in_bounds(memory, n.x, n.y)) continue;

    uint8_t value = memory.get_flood(n.x, n.y);
    if (value < best_value) {
      best_value = value;
      best_direction = direction;
    }
  }

  return best_direction;
}

bool is_solved(int x, int y, const Cell *goal_cells, uint8_t goal_count) {
  for (uint8_t i = 0; i < goal_count; i++) {
    if (x == goal_cells[i].x && y == goal_cells[i].y) {
      return true;
    }
  }

  return false;
}

int get_full_path(Memory &memory, Cell start, const Cell *goal_cells,
                   uint8_t goal_count, Direction *path) {
  int x = start.x, y = start.y;
  uint16_t len = 0;

  while (true) {
    bool at_goal = false;
    for (uint8_t i = 0; i < goal_count; i++) {
      if (goal_cells[i].x == x && goal_cells[i].y == y) { at_goal = true; break; }
    }
    if (at_goal) break;

    Direction direction = get_next_move(memory, x, y);
    if (direction == DIR_NONE) {
      return -1;   // stuck - goal unreachable
    }
    if (len >= MAX_PATH_LEN) {
      return -1;   // safety net against an infinite loop
    }

    path[len++] = direction;
    Cell n = step_cell(x, y, direction);
    x = n.x; y = n.y;
  }

  return len;
}