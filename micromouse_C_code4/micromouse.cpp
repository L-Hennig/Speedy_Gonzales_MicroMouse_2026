/*
 * Filename: micromouse.cpp
 * Description: C++ port of micromouse.py for Arduino IDE (arduino-pico core).
 */
#include "micromouse.h"
#include "tof_calibration.h"
#include <Ticker.h>

Micromouse* Micromouse::_self = nullptr;
static Ticker blink_timer;

Micromouse::Micromouse()
  : motor_1(21, 20, 19, 22),
    motor_2(17, 18, 15, 16),
    _button_pin(11),   // confirmed: physical pin 15
    _ir_1_pin(12), _ir_2_pin(13), _ir_3_pin(14),
    _green_led_pin(10), _red_led_pin(9), _debug_led_pin(25),
    center_cells_count(0),
    heading(0), speed(200),
    _tof_1_ok(false), _tof_2_ok(false), _tof_3_ok(false)
{
  // Only one Micromouse should ever be constructed (see header comment) -
  // this just always points at the most recently constructed one.
  _self = this;

  start_cell = { 0, 0 };
  center_cells[0] = { 4, 4 };
  center_cells_count = 1;
  x = start_cell.x;
  y = start_cell.y;
}

void Micromouse::begin() {
  pinMode(_button_pin, INPUT);
  pinMode(_ir_1_pin, INPUT);
  pinMode(_ir_2_pin, INPUT);
  pinMode(_ir_3_pin, INPUT);

  pinMode(_green_led_pin, OUTPUT);
  pinMode(_red_led_pin, OUTPUT);
  pinMode(_debug_led_pin, OUTPUT);

  motor_1.begin();
  motor_2.begin();
  motor_2.invert_motor();   // matches the original's motor_2.invert_motor() call

  Wire.setSDA(0);   // GP0
  Wire.setSCL(1);   // GP1
  Wire.begin();
  tof_begin();
}

void Micromouse::tof_begin() {
  // Hold every sensor in reset first.
  pinMode(TOF_XSHUT_1_PIN, OUTPUT);
  pinMode(TOF_XSHUT_2_PIN, OUTPUT);
  pinMode(TOF_XSHUT_3_PIN, OUTPUT);
  digitalWrite(TOF_XSHUT_1_PIN, LOW);
  digitalWrite(TOF_XSHUT_2_PIN, LOW);
  digitalWrite(TOF_XSHUT_3_PIN, LOW);
  delay(10);

  // Bring sensor 1 up alone (still at the 0x29 default address), init it,
  // then move it off to its own address so it stops responding to 0x29.
  digitalWrite(TOF_XSHUT_1_PIN, HIGH);
  delay(10);
  _tof_1 = VL53L4CD(Wire, 0x29);
  _tof_1_ok = _tof_1.begin();
  if (_tof_1_ok) {
    _tof_1.set_address(TOF_ADDRESS_1);
    _tof_1.start_ranging();
  }

  // Same for sensor 2.
  digitalWrite(TOF_XSHUT_2_PIN, HIGH);
  delay(10);
  _tof_2 = VL53L4CD(Wire, 0x29);
  _tof_2_ok = _tof_2.begin();
  if (_tof_2_ok) {
    _tof_2.set_address(TOF_ADDRESS_2);
    _tof_2.start_ranging();
  }

  // Same for sensor 3.
  digitalWrite(TOF_XSHUT_3_PIN, HIGH);
  delay(10);
  _tof_3 = VL53L4CD(Wire, 0x29);
  _tof_3_ok = _tof_3.begin();
  if (_tof_3_ok) {
    _tof_3.set_address(TOF_ADDRESS_3);
    _tof_3.start_ranging();
  }

  // Any sensor that returned false above (wrong ID / no response) is left
  // as "not ok" - get_tof_distance() will return -1.0 for it rather than
  // a bogus reading.
}

// --- LEDs ---------------------------------------------------------------

void Micromouse::led_set(bool red_val, bool green_val) {
  digitalWrite(_green_led_pin, green_val);
  digitalWrite(_red_led_pin, red_val);
}

void Micromouse::led_green_set(bool value) {
  digitalWrite(_green_led_pin, value);
}

void Micromouse::led_red_set(bool value) {
  digitalWrite(_red_led_pin, value);
}

void Micromouse::led_debug_set(bool value) {
  digitalWrite(_debug_led_pin, value);
}

void Micromouse::led_toggle() {
  digitalWrite(_green_led_pin, !digitalRead(_green_led_pin));
  digitalWrite(_red_led_pin, !digitalRead(_red_led_pin));
}

void Micromouse::blink_callback() {
  if (_self != nullptr) {
    _self->led_toggle();
  }
}

void Micromouse::led_toggle_start(float frequency_hz) {
  blink_timer.attach(1.0f / frequency_hz, blink_callback);
}

void Micromouse::led_toggle_stop() {
  blink_timer.detach();
  digitalWrite(_red_led_pin, LOW);
  digitalWrite(_green_led_pin, LOW);
}

// --- Sensors --------------------------------------------------------------

Micromouse::IrReadings Micromouse::get_ir_values() {
  IrReadings r;
  r.ir_1 = digitalRead(_ir_1_pin) == LOW;
  r.ir_2 = digitalRead(_ir_2_pin) == LOW;
  r.ir_3 = digitalRead(_ir_3_pin) == LOW;
  return r;
}

bool Micromouse::get_ir_value(uint8_t index) {
  switch (index) {
    case 1: return digitalRead(_ir_1_pin) == LOW;
    case 2: return digitalRead(_ir_2_pin) == LOW;
    case 3: return digitalRead(_ir_3_pin) == LOW;
    default:
      // Matches the original's IndexError for out-of-range input.
      return false;
  }
}

float Micromouse::get_tof_distance(uint8_t index) {
  VL53L4CD *sensor;
  bool ok;

  switch (index) {
    case 1: sensor = &_tof_1; ok = _tof_1_ok; break;
    case 2: sensor = &_tof_2; ok = _tof_2_ok; break;
    case 3: sensor = &_tof_3; ok = _tof_3_ok; break;
    default: return -1.0f;
  }

  if (!ok) return -1.0f;

  float raw_mm = sensor->get_distance();
  return calibrate(raw_mm, index);
}

bool Micromouse::get_button() {
  return digitalRead(_button_pin) == LOW;
}

// --- Driving ----------------------------------------------------------

void Micromouse::drive_forward(int power) {
  motor_2.spin_forward(power);
  motor_1.spin_forward(power);
}

void Micromouse::drive_backward(int power) {
  motor_2.spin_backward(power);
  motor_1.spin_backward(power);
}

void Micromouse::drive_stop() {
  motor_2.spin_stop();
  motor_1.spin_stop();
}

void Micromouse::invert_motor_1() {
  motor_1.invert_motor();
}

void Micromouse::invert_motor_2() {
  motor_2.invert_motor();
}

Micromouse::Encoders Micromouse::get_encoders() {
  // Named clearly here (encoder_1 = motor_1, encoder_2 = motor_2) -
  // the original Python returned them as a swapped (encoder_2, encoder_1)
  // tuple, which looked like it may have been unintentional.
  Encoders e;
  e.encoder_1 = motor_1.encoder_read();
  e.encoder_2 = motor_2.encoder_read();
  return e;
}