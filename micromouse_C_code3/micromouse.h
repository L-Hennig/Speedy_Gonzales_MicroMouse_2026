/*
 * Filename: micromouse.h
 * Description: C++ port of micromouse.py for Arduino IDE (arduino-pico core).
 *
 * Fixes applied vs. the current Python version (as agreed):
 *   - get_button() naming mismatch fixed.
 *   - Only ONE physical button is actually readable in software - the
 *     other is wired to the Pico's RUN pin (physical pin 30), which is a
 *     hardware reset line, not a GPIO. Pressing it reboots the chip
 *     directly, which re-runs setup() and therefore full_reset() from
 *     scratch - so "full reset for a new maze" needs no code at all; it's
 *     inherent in how RUN works. get_button() now takes no index and
 *     just reads the one real button; navigation.cpp counts how many
 *     times it's pressed in a row (single/double/triple/...) to choose
 *     an action instead of using two separate buttons.
 *   - ToF distance reading now uses the real VL53L4CD driver + per-sensor
 *     calibration correction (tof_calibration.h), returning corrected mm.
 *     The three sensors' XSHUT pins are placeholders (see the ToF section
 *     below) - fix those once you know the real wiring; the standard
 *     address-reassignment sequence in begin() will work as-is once they're
 *     correct.
 *
 * Design note: the Python version uses __new__ to force a singleton,
 * since MicroPython code just calls Micromouse() repeatedly to get the
 * same object. In C++/Arduino the natural equivalent is to just create
 * ONE global Micromouse instance in your .ino and pass it (or a pointer
 * /reference to it) into movement/navigation functions - so no singleton
 * enforcement is implemented here. Don't construct more than one.
 */
#ifndef MICROMOUSE_H
#define MICROMOUSE_H

#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "memory.h"   // for the shared Cell struct
#include "vl53l4cd.h"

class Micromouse {
  public:
    Micromouse();

    // Call once from setup() - brings up motors, pin modes, etc.
    void begin();

    // --- LEDs ---
    void led_set(bool red_val, bool green_val);
    void led_green_set(bool value);
    void led_red_set(bool value);
    void led_debug_set(bool value);
    void led_toggle();
    void led_toggle_start(float frequency_hz = 1.0);
    void led_toggle_stop();

    // --- Sensors ---
    // index 0 returns all three as a struct; 1-3 returns just that sensor.
    struct IrReadings { bool ir_1, ir_2, ir_3; };
    IrReadings get_ir_values();
    bool get_ir_value(uint8_t index);

    // Returns the corrected distance in mm for sensor 1 (front), 2 (left),
    // or 3 (right), applying the matching calibration table. Returns -1.0
    // if that sensor failed to initialise (see begin()).
    float get_tof_distance(uint8_t index);

    bool get_button();   // the one real, software-readable button

    // --- Driving ---
    void drive_forward(int power = 255);
    void drive_backward(int power = 255);
    void drive_stop();
    void invert_motor_1();
    void invert_motor_2();

    struct Encoders { long encoder_1, encoder_2; };
    Encoders get_encoders();

    // --- Motors (public, same as Python's mm.motor_1 / mm.motor_2 access) ---
    Motor motor_1;
    Motor motor_2;

    // --- Maze / navigation state (public - accessed directly like the
    // Python mm.x, mm.y, mm.heading etc. in movement.py / navigation.py) ---
    Cell start_cell;
    static const uint8_t MAX_GOAL_CELLS = 4;
    Cell center_cells[MAX_GOAL_CELLS];
    uint8_t center_cells_count;

    int x, y;
    int heading;
    int speed;

  private:
    // Inputs
    uint8_t _button_pin;
    uint8_t _ir_1_pin, _ir_2_pin, _ir_3_pin;
    // Outputs
    uint8_t _green_led_pin, _red_led_pin, _debug_led_pin;

    // LED blink timer - Ticker callbacks are plain function pointers (no
    // captured state), so we go through a single static instance pointer,
    // same trick as Motor's ISR trampoline. Only safe because there's only
    // ever one Micromouse instance (see class comment above).
    static Micromouse* _self;
    static void blink_callback();

    // --- ToF sensors ---
    // All three share the SDA/SCL pins seen in tof_test_drive_v1.py
    // (GP4/GP5). Since they all power up at the same default I2C address
    // (0x29), each one's XSHUT pin is held low to keep it in reset while
    // the others are brought up and reassigned to a unique address - the
    // standard multi-VL53L4CD bring-up sequence (see begin() in the .cpp).
    //
    // XSHUT pin numbers below are placeholders, same treatment as
    // button_2_pin - fix these once you know the real wiring.
    static const uint8_t TOF_XSHUT_1_PIN = 6;   // front
    static const uint8_t TOF_XSHUT_2_PIN = 8;   // left
    static const uint8_t TOF_XSHUT_3_PIN = 7;   // right

    // Distinct addresses assigned during begin() - arbitrary but must be
    // different from the 0x29 power-up default and from each other.
    static const uint8_t TOF_ADDRESS_1 = 0x30;
    static const uint8_t TOF_ADDRESS_2 = 0x31;
    static const uint8_t TOF_ADDRESS_3 = 0x32;

    VL53L4CD _tof_1, _tof_2, _tof_3;
    bool _tof_1_ok, _tof_2_ok, _tof_3_ok;

    // Brings up all three sensors one at a time via XSHUT and assigns
    // each a unique address. Called from begin().
    void tof_begin();
};

#endif