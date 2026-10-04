/*
 * Filename: vl53l4cd.h
 * Description: C++ port of vl53l4cd.py for Arduino IDE (arduino-pico core).
 *
 * vl53l4cd.py is a MicroPython port of Adafruit's CircuitPython
 * VL53L4CD driver, which itself talks to the sensor through a separate
 * i2c_device.py wrapper. That wrapper (checked separately) turned out to
 * be a thin write/read helper with no register-addressing logic of its
 * own - all the actual register read/write framing happens in
 * vl53l4cd.py itself. So here everything talks directly to the Arduino
 * Wire library instead of porting a second wrapper class - Wire already
 * serialises bus access, which is all i2c_device.py was doing.
 */
#ifndef VL53L4CD_H
#define VL53L4CD_H

#include <Arduino.h>
#include <Wire.h>

class VL53L4CD {
  public:
    // address is the sensor's power-up default (0x29 / 41 decimal).
    VL53L4CD(TwoWire &wire = Wire, uint8_t address = 0x29);

    // Checks the model ID, waits for boot, and runs the same init
    // register sequence as the Python driver's __init__ / _sensor_init.
    // Returns false if the sensor doesn't answer correctly or times out.
    bool begin();

    void start_ranging();
    void stop_ranging();
    void clear_interrupt();
    bool data_ready();

    // Distance in mm. Note: the Python driver's `distance` property
    // reports centimeters (raw register value / 10); this returns the
    // raw register value directly as mm instead, since that's the unit
    // actually needed for wall-distance thresholds and calibration.
    float distance_mm();

    // Blocks until a fresh reading is ready, then returns distance_mm().
    float get_distance();

    uint16_t get_timing_budget_ms();
    bool set_timing_budget_ms(uint16_t val);   // false = rejected (see .cpp)

    uint16_t get_inter_measurement_ms();
    bool set_inter_measurement_ms(uint16_t val);

    // Only needed when multiple sensors share one I2C bus: hold every
    // other sensor's XSHUT pin low, call begin() + set_address() on this
    // one, release the next sensor's XSHUT, repeat. See micromouse.cpp
    // for how the three ToF sensors are brought up this way.
    void set_address(uint8_t new_address);

  private:
    TwoWire *_wire;
    uint8_t _address;
    bool _ranging;

    void write_register(uint16_t reg, const uint8_t *data, uint8_t length);
    void read_register(uint16_t reg, uint8_t *data, uint8_t length);

    bool wait_for_boot();
    bool start_vhv();
    uint8_t interrupt_polarity();
};

#endif