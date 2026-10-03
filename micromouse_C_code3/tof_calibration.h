/*
 * Filename: tof_calibration.h
 * Description: C++ port of tof_calibration.py for Arduino IDE.
 */
#ifndef TOF_CALIBRATION_H
#define TOF_CALIBRATION_H

#include <Arduino.h>

// Converts a raw sensor-reported distance (mm) into a corrected
// real-world distance (mm) by linearly interpolating between the
// nearest calibration points for the given sensor (1, 2, or 3).
// Falls back to returning measured_mm unchanged for an unknown sensor
// number (the Python version would have thrown a KeyError instead).
float calibrate(float measured_mm, uint8_t sensor_num);

#endif