/*
 * Filename: abort.cpp
 * Description: Non-blocking double-press detection used to abort a run.
 */
#include "abort.h"

static bool g_abort = false;
static bool last_pressed = false;
static uint8_t press_count = 0;
static unsigned long first_press_ms = 0;
static unsigned long last_change_ms = 0;

void abort_reset(Micromouse &mm) {
  g_abort = false;
  press_count = 0;
  last_pressed = mm.get_button();
  last_change_ms = millis();
}

void abort_poll(Micromouse &mm) {
  if (g_abort) return;

  unsigned long now = millis();
  bool pressed = mm.get_button();

  // Presses too far apart don't count together
  if (press_count > 0 && now - first_press_ms > ABORT_WINDOW_MS) {
    press_count = 0;
  }

  if (pressed != last_pressed && now - last_change_ms >= ABORT_DEBOUNCE_MS) {
    last_change_ms = now;
    last_pressed = pressed;

    if (pressed) {
      if (press_count == 0) first_press_ms = now;
      press_count++;
      if (press_count >= 2) g_abort = true;
    }
  }
}

bool abort_requested() {
  return g_abort;
}