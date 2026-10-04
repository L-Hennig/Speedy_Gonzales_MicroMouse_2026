/*
 * Filename: motor.cpp
 * Description: C++ port of motor.py for Arduino IDE (arduino-pico core).
 */
#include "motor.h"

// --- Quadrature decode table -----------------------------------------
// Ported from rotary.py's _transition_table (full-step mode, the default
// RotaryIRQ used: half_step=False, invert=False, reverse=False).
// Rows = current state (masked to 3 bits), columns = new (CLK<<1 | DT)
// reading. The two high bits of the result carry a direction flag when
// a full detent has been completed; otherwise they're 0 (still debouncing
// partway through a transition, or an illegal jump was ignored).
#define DIR_CW    0x10
#define DIR_CCW   0x20
#define R_START   0x0

static const uint8_t transition_table[7][4] = {
  //   00        01        10        11
  { R_START,   0x4,      0x1,      R_START            }, // R_START
  { 0x2,       R_START,  0x1,      R_START            }, // R_CW_1
  { 0x2,       0x3,      0x1,      R_START            }, // R_CW_2
  { 0x2,       0x3,      R_START,  R_START | DIR_CW   }, // R_CW_3
  { 0x5,       0x4,      R_START,  R_START            }, // R_CCW_1
  { 0x5,       0x4,      0x6,      R_START            }, // R_CCW_2
  { 0x5,       R_START,  0x6,      R_START | DIR_CCW  }  // R_CCW_3
};

Motor* Motor::instances[Motor::MAX_MOTORS] = { nullptr, nullptr };
uint8_t Motor::instance_count = 0;
bool Motor::pwm_freq_set = false;

Motor::Motor(uint8_t m1_pin, uint8_t m2_pin, uint8_t e1_pin, uint8_t e2_pin)
  : _m1_pin(m1_pin), _m2_pin(m2_pin), _e1_pin(e1_pin), _e2_pin(e2_pin),
    _invert(false), _encoder_count(0), _state(R_START)
{
  if (instance_count < MAX_MOTORS) {
    instances[instance_count++] = this;
  }
}

void Motor::begin() {
  pinMode(_m1_pin, OUTPUT);
  pinMode(_m2_pin, OUTPUT);
  pinMode(_e1_pin, INPUT);
  pinMode(_e2_pin, INPUT);

  // Matches the original PWM(freq=2000). This sets frequency for ALL PWM
  // pins on the core, and calling it more than once can disturb pins
  // already configured by an earlier Motor's begin() - so it's now
  // guarded to run exactly once, regardless of how many Motors exist.
  if (!pwm_freq_set) {
    analogWriteFreq(2000);
    pwm_freq_set = true;
  }
  // Default analogWrite range is 0-255, which matches the original
  // 0-255 power scaling - no extra conversion needed.

  // Prime both PWM pins now, at boot, rather than letting the first real
  // analogWrite() happen during an actual drive command. The RP2040 core
  // does one-time setup work (claiming/configuring that pin's PWM slice)
  // on a pin's very first analogWrite() call - if that first call happens
  // mid-movement, it can land at slightly different times for different
  // pins and make the motors appear to start one after another, even
  // though they were commanded at the same instant.
  analogWrite(_m1_pin, 0);
  analogWrite(_m2_pin, 0);

  attachInterruptParam(digitalPinToInterrupt(_e1_pin), isr_trampoline, CHANGE, this);
  attachInterruptParam(digitalPinToInterrupt(_e2_pin), isr_trampoline, CHANGE, this);
}

int Motor::constrain_power(int value, int min_value, int max_value) {
  return min(max_value, max(min_value, value));
}

void Motor::spin_forward(int power) {
  spin_power(constrain_power(power, 0, 255));
}

void Motor::spin_backward(int power) {
  spin_power(-constrain_power(power, 0, 255));
}

void Motor::spin_power(int power) {
  int limited = constrain_power(power, -255, 255);
  int magnitude = abs(limited);

  // Whichever pin ends up active, it always gets the positive magnitude -
  // analogWrite() (and the original duty_u16()) both expect an unsigned
  // value, so the sign of `limited` only decides WHICH pin is active,
  // never what gets written to it. Note this only correctly fixed the
  // false+backward case in an earlier version; _invert = true broke both
  // directions, since the else-branch's `-limited` could still end up
  // negative depending on the XOR outcome. Using abs() here instead
  // covers every combination of direction and invert correctly.
  if ((limited > 0) ^ _invert) {
    analogWrite(_m2_pin, magnitude);
    analogWrite(_m1_pin, 0);
  } else {
    analogWrite(_m1_pin, magnitude);
    analogWrite(_m2_pin, 0);
  }
}

void Motor::spin_stop() {
  analogWrite(_m1_pin, 0);
  analogWrite(_m2_pin, 0);
}

long Motor::encoder_read() {
  // Encoder count is written from an ISR, so briefly disable interrupts
  // while reading it to avoid a torn read (long is wider than one atomic
  // word on this core).
  noInterrupts();
  long value = _encoder_count;
  interrupts();
  return value;
}

void Motor::invert_motor() {
  _invert = !_invert;
}

void Motor::handle_encoder() {
  uint8_t ab = (digitalRead(_e1_pin) << 1) | digitalRead(_e2_pin);
  _state = transition_table[_state & 0x07][ab];
  uint8_t dir = _state & 0x30;
  if (dir == DIR_CW) {
    _encoder_count++;
  } else if (dir == DIR_CCW) {
    _encoder_count--;
  }
}

void Motor::isr_trampoline(void* arg) {
  static_cast<Motor*>(arg)->handle_encoder();
}