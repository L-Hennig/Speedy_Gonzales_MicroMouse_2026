/*
 * Filename: abort.h
 * Description: Non-blocking double-press detection used to abort a run.
 * Call abort_poll() often (inside movement/sensing loops); it sets a flag
 * once the button is pressed twice within ABORT_WINDOW_MS.
 */
#pragma once
#include <Arduino.h>
#include "micromouse.h"

#define ABORT_WINDOW_MS   500   // two presses within this = abort
#define ABORT_DEBOUNCE_MS 30    // ignore button changes faster than this

void abort_reset(Micromouse &mm);  // clear flag, call at the start of a run
void abort_poll(Micromouse &mm);   // call frequently while a run is active
bool abort_requested();