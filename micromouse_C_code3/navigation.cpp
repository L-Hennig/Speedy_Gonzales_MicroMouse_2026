/*
 * Filename: navigation.cpp
 * Description: C++ port of navigation.py for Arduino IDE (arduino-pico core).
 *
 * run() now does 7 exploratory runs (no separate fast-run mode):
 *   runs 1-2: speed 150
 *   runs 3-5: speed 200
 *   runs 6-7: speed 255, with unvisited cells blocked (known cells only)
 * Runs after the 7th repeat the last config (255, blocked).
 */
#include "navigation.h"
#include "movement.h"
#include "abort.h"

// Run schedule (explore_count is 0-based: 0 = run 1)
static const int TIER_1_RUNS = 2;   // runs 1-2 -> speed 150
static const int TIER_2_END  = 5;   // runs 3-5 -> speed 200, then 255
static const int TIER_1_SPEED = 150;
static const int TIER_2_SPEED = 200;
static const int TIER_3_SPEED = 255;

static void wait_for_release(Micromouse &mm) {
  while (mm.get_button()) {
    delay(10);
  }
}

uint8_t wait_for_press_count(Micromouse &mm) {
  while (!mm.get_button()) {
    delay(10);
  }

  uint8_t count = 0;
  while (true) {
    wait_for_release(mm);
    count++;

    unsigned long start = millis();
    bool pressed_again = false;
    while (millis() - start < PRESS_WINDOW_MS) {
      if (mm.get_button()) {
        pressed_again = true;
        break;
      }
      delay(10);
    }
    if (!pressed_again) break;
  }
  return count;
}

void read_walls(Micromouse &mm, Memory &memory) {

  const unsigned long MEASURE_TIME = 500;  // 0.5 seconds

  unsigned long start_time = millis();

  float front_sum = 0;
  float left_sum = 0;
  float right_sum = 0;

  int samples = 0;

  // Take measurements for 0.5 seconds
  while (millis() - start_time < MEASURE_TIME) {

    abort_poll(mm);

    front_sum += mm.get_tof_distance(1);
    left_sum  += mm.get_tof_distance(2);
    right_sum += mm.get_tof_distance(3);

    samples++;

    delay(10);
  }

  // Calculate averages
  float front = front_sum / samples;
  float left  = left_sum / samples;
  float right = right_sum / samples;

  Serial.print("Average ToF | Front: ");
  Serial.print(front);
  Serial.print(" mm | Left: ");
  Serial.print(left);
  Serial.print(" mm | Right: ");
  Serial.print(right);
  Serial.println(" mm");

  Direction front_dir = ALL_DIRECTIONS[mm.heading % 4];
  Direction left_dir  = ALL_DIRECTIONS[((mm.heading - 1) % 4 + 4) % 4];
  Direction right_dir = ALL_DIRECTIONS[(mm.heading + 1) % 4];

  if (front >= 0 && front <= 140) {
    memory.set_wall(mm.x, mm.y, front_dir);

    Serial.print("WALL FRONT: ");
    Serial.print(front);
    Serial.println(" mm");
  }

  if (left >= 0 && left <= 140) {
    memory.set_wall(mm.x, mm.y, left_dir);

    Serial.print("WALL LEFT: ");
    Serial.print(left);
    Serial.println(" mm");
  }

  if (right >= 0 && right <= 140) {
    memory.set_wall(mm.x, mm.y, right_dir);

    Serial.print("WALL RIGHT: ");
    Serial.print(right);
    Serial.println(" mm");
  }
}


void move_to_next_cell(Micromouse &mm, Direction next_dir) {
  int dif = mm.heading - next_dir;
  if (dif == 0) {
    // already facing the right way
  } else if (dif == 1 || dif == -3) {
    turn_left(mm);
  } else if (dif == 2 || dif == -2) {
    turn_180(mm);
  } else if (dif == 3 || dif == -1) {
    turn_right(mm);
  }
  delay(250);
  // The Python version never updated mm.heading or mm.x/mm.y here - added
  // this so explore/fast-run actually track where the mouse is. Heading
  // is set to the direction just turned to (clockwise N,E,S,W = 0,1,2,3),
  // then x/y step one cell in that direction using the same step_cell()
  // flood_fill.cpp uses internally, so both stay in exact agreement.
  mm.heading = next_dir;
  move_forward_one_cell(mm);

  Cell moved_to = step_cell(mm.x, mm.y, static_cast<Direction>(mm.heading));
  mm.x = moved_to.x;
  mm.y = moved_to.y;
}

void explore_step(Micromouse &mm, Memory &memory, const Cell *goal_cells, uint8_t goal_count) {
  if (abort_requested()) return;

  // Walls are only sensed in cells not yet visited. Once unvisited cells
  // are blocked (last runs) the mouse never enters one, so this is skipped.
  if (!memory.is_visited(mm.x, mm.y)) { 
    delay(10);
    read_walls(mm, memory);
    memory.mark_visited(mm.x, mm.y);

    Serial.print("Current cell: (");
    Serial.print(mm.x);
    Serial.print(", ");
    Serial.print(mm.y);
    Serial.print(") heading=");
    Serial.println((int)mm.heading);

    Serial.print("Visited: ");
    Serial.println(memory.is_visited(mm.x, mm.y) ? "YES" : "NO");

    Serial.print("Flood current: ");
    Serial.println(memory.get_flood(mm.x, mm.y));
  }


  update_flood(memory, goal_cells, goal_count);
  Direction next_dir = get_next_move(memory, mm.x, mm.y);

  Serial.print("next_dir = ");

  Serial.print("Next direction: ");

  switch (next_dir) {
    case NORTH: Serial.println("NORTH"); break;
    case EAST:  Serial.println("EAST");  break;
    case SOUTH: Serial.println("SOUTH"); break;
    case WEST:  Serial.println("WEST");  break;
    case DIR_NONE: Serial.println("NONE"); break;
  }

  
  delay(10);
  

  move_to_next_cell(mm, next_dir);

  delay(10);
  
  Serial.print("Current cell visited: ");
  Serial.println(memory.is_visited(mm.x, mm.y) ? "YES" : "NO");
  Serial.print("\n");

  Serial.println("Walls in memory:");

  for (uint8_t i = 0; i < 4; i++) {
    Direction d = ALL_DIRECTIONS[i];

    Serial.print("  ");
    
    switch (d) {
      case NORTH: Serial.print("NORTH"); break;
      case EAST:  Serial.print("EAST"); break;
      case SOUTH: Serial.print("SOUTH"); break;
      case WEST:  Serial.print("WEST"); break;
      default:    Serial.print("NONE"); break;
    }

    Serial.print(": ");
    Serial.println(memory.has_wall(mm.x, mm.y, d) ? "WALL" : "OPEN");
  }

  
}

bool fast_run(Micromouse &mm, Memory &memory) {
  const Cell *goal_cells = mm.center_cells;
  uint8_t goal_count = mm.center_cells_count;

  memory.block_unvisited();
  update_flood(memory, goal_cells, goal_count);

  Direction path[MAX_PATH_LEN];
  int path_len = get_full_path(memory, mm.start_cell, goal_cells, goal_count, path);
  if (path_len < 0) {
    return false;   // stuck - goal unreachable (Python raised RuntimeError)
  }

  for (int i = 0; i < path_len; i++) {
    move_to_next_cell(mm, path[i]);
  }
  return true;
}

void run_explore_leg(Micromouse &mm, Memory &memory,
  const Cell *goal_cells, uint8_t goal_count) {

  Serial.println("EXPLORE: entered");

  Serial.print("Is solved: ");
  Serial.println(
    is_solved(mm.x, mm.y, goal_cells, goal_count) ? "YES" : "NO"
  );

  while (!is_solved(mm.x, mm.y, goal_cells, goal_count) && !abort_requested()) {

    explore_step(mm, memory, goal_cells, goal_count);

  }
}

static void full_reset(Micromouse &mm, Memory &memory, int &explore_count) {
  memory.reset();
  mm.x = mm.start_cell.x;
  mm.y = mm.start_cell.y;
  mm.heading = NORTH;
  explore_count = 0;
}

// Delay that keeps watching for an abort
static void wait_abortable(Micromouse &mm, unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms && !abort_requested()) {
    abort_poll(mm);
    delay(5);
  }
}

void run(Micromouse &mm, Memory &memory) {

  int explore_count = 0;   // completed runs only
  Memory committed;        // memory saved by the last good run
  Memory checkpoint;       // memory at the moment the centre was reached

  Serial.println("RUN: entered");

  full_reset(mm, memory, explore_count);
  committed = memory;
  Serial.println("RUN: full_reset complete");

  while (true) {

    Serial.println("RUN: waiting for button");

    uint8_t presses = wait_for_press_count(mm);

    Serial.print("RUN: button presses detected = ");
    Serial.println(presses);

    // Only a single press starts a run
    if (presses != PRESSES_START_RUN) {
      Serial.println("RUN: invalid button press count");
      continue;
    }

    // Every run starts from the saved memory, at the start cell
    memory = committed;
    mm.x = mm.start_cell.x;
    mm.y = mm.start_cell.y;
    mm.heading = NORTH;
    abort_reset(mm);

    // Speed by run number
    if (explore_count < TIER_1_RUNS) {
      mm.speed = TIER_1_SPEED;
    } else if (explore_count < TIER_2_END) {
      mm.speed = TIER_2_SPEED;
    } else {
      mm.speed = TIER_3_SPEED;

      // Final runs: only travel through cells already visited
      // (safe to repeat every run)
      memory.block_unvisited();
      Serial.println("RUN: unvisited cells blocked");
    }

    Serial.print("RUN: exploration ");
    Serial.print(explore_count + 1);
    Serial.print(", speed = ");
    Serial.println(mm.speed);

    // ---- Leg 1: start -> centre ----
    run_explore_leg(mm, memory, mm.center_cells, mm.center_cells_count);

    if (abort_requested()) {
      mm.drive_stop();
      memory = committed;   // discard everything from this run
      Serial.println("RUN: ABORTED before centre - run discarded");
      wait_for_release(mm);
      delay(500);
      continue;
    }

    Serial.println("RUN: reached centre - checkpoint saved");
    checkpoint = memory;

    wait_abortable(mm, 2000);

    // ---- Leg 2: centre -> start ----
    if (!abort_requested()) {
      Serial.println("RUN: starting return to start");
      run_explore_leg(mm, memory, &mm.start_cell, 1);
    }

    if (abort_requested()) {
      mm.drive_stop();
      committed = checkpoint;   // keep start->centre info, drop the return
      memory = committed;
      Serial.println("RUN: ABORTED on return - kept info up to centre");
      wait_for_release(mm);
      delay(500);
      continue;
    }

    Serial.println("RUN: returned to start - memory saved");
    committed = memory;
    explore_count++;

    Serial.print("RUN: exploration count = ");
    Serial.println(explore_count);
  }
}