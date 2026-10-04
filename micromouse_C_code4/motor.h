/*
 * Filename: motor.h
 * Description: C++ port of motor.py for Arduino IDE (arduino-pico core).
 *   Folds the quadrature encoder decoding (previously rotary.py /
 *   rotary_irq_rp2.py) directly into the Motor class, since we only ever
 *   need a free-running signed count for two fixed encoders - no need for
 *   the original generic Rotary base class's wrap/bounded modes.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    Motor(uint8_t m1_pin, uint8_t m2_pin, uint8_t e1_pin, uint8_t e2_pin);

    // Call once from setup(), after the object exists - sets pin modes,
    // PWM frequency, and attaches the encoder interrupts.
    void begin();

    void spin_forward(int power = 255);
    void spin_backward(int power = 255);
    void spin_power(int power);   // -255..255, negative = backward
    void spin_stop();

    long encoder_read();
    void invert_motor();

  private:
    static const uint8_t MAX_MOTORS = 2;
    static Motor* instances[MAX_MOTORS];
    static uint8_t instance_count;
    static bool pwm_freq_set;   // ensures analogWriteFreq() only ever runs once

    uint8_t _m1_pin, _m2_pin, _e1_pin, _e2_pin;
    bool _invert;
    volatile long _encoder_count;
    volatile uint8_t _state;   // quadrature decode state machine

    int constrain_power(int value, int min_value, int max_value);
    void handle_encoder();    // runs the decode step, called from the ISR

    // attachInterruptParam callback trampoline - arduino-pico core lets us
    // pass an arbitrary void* (here, "this") so one static function can
    // serve every Motor instance's interrupts.
    static void isr_trampoline(void* arg);
};

#endif