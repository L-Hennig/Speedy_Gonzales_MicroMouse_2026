/*
 * Filename: tof_calibration.cpp
 * Description: C++ port of tof_calibration.py for Arduino IDE.
 */
#include "tof_calibration.h"

struct CalPoint {
  float actual_mm;
  float measured_mm;
};

// (actual_mm, measured_mm), sorted by actual distance - transcribed
// directly from tof_calibration.py's three CALIBRATION_TABLE_* lists.
static const CalPoint TABLE_1[] = {
  {10, 15.9f}, {15, 20.7f}, {20, 26.6f}, {25, 32.86f}, {30, 37.26f},
  {35, 43.05f}, {40, 49.77f}, {45, 54.63f}, {50, 60.94f}, {55, 65.06f},
  {60, 71.08f}, {65, 75.81f}, {70, 80.86f}, {75, 86.63f}, {80, 92.19f},
  {85, 97.89f}, {90, 102.85f}, {95, 108.44f}, {100, 113.31f}, {105, 118.31f},
  {110, 124.42f}, {115, 127.62f}, {120, 133.29f}, {125, 138.29f}, {130, 143.96f},
  {135, 149.66f}, {140, 154.33f}, {145, 158.03f}, {150, 164.49f}, {155, 170.41f},
  {160, 175.99f}, {165, 179.54f}, {170, 186.03f}, {175, 188.99f}, {180, 192.29f},
  {185, 198.14f}, {190, 203.63f}, {195, 209.04f}, {200, 213.74f}, {205, 217.44f},
  {210, 221.64f}, {215, 227.08f}, {220, 233.0f}, {225, 237.98f}, {230, 243.27f},
  {235, 247.62f}, {240, 251.49f}, {245, 256.84f}, {250, 261.06f}, {255, 266.24f},
  {260, 270.29f}, {265, 275.76f}, {270, 280.0f}, {275, 284.05f}, {280, 289.56f},
  {285, 294.88f}, {290, 298.77f}, {295, 303.78f}, {300, 309.52f}, {350, 354.68f},
  {400, 402.6f}, {450, 446.96f}, {550, 534.8f}, {600, 577.57f},
};

static const CalPoint TABLE_2[] = {
  {9.50f, 10}, {16.09f, 15}, {23.34f, 20}, {30.77f, 25}, {36.87f, 30},
  {42.39f, 35}, {48.60f, 40}, {54.43f, 45}, {58.76f, 50}, {64.83f, 55},
  {69.98f, 60}, {74.74f, 65}, {79.81f, 70}, {84.78f, 75}, {90.58f, 80},
  {94.86f, 85}, {99.84f, 90}, {105.48f, 95}, {110.78f, 100}, {115.50f, 105},
  {120.90f, 110}, {125.66f, 115}, {130.84f, 120}, {135.49f, 125}, {139.44f, 130},
  {143.04f, 135}, {147.94f, 140}, {153.66f, 145}, {159.34f, 150}, {164.14f, 155},
  {169.56f, 160}, {174.56f, 165}, {179.17f, 170}, {184.11f, 175}, {189.63f, 180},
  {194.76f, 185}, {199.48f, 190}, {204.22f, 195}, {210.02f, 200}, {214.20f, 205},
  {218.55f, 210}, {223.57f, 215}, {228.44f, 220}, {233.63f, 225}, {238.13f, 230},
  {242.68f, 235}, {247.92f, 240}, {252.38f, 245}, {256.45f, 250}, {260.92f, 255},
  {265.87f, 260}, {270.33f, 265}, {274.76f, 270}, {279.32f, 275}, {283.94f, 280},
  {287.98f, 285}, {292.17f, 290}, {296.89f, 295}, {301.62f, 300}, {345.60f, 350},
  {386.48f, 400}, {428.16f, 450}, {463.95f, 500}, {502.57f, 550}, {533.89f, 600},
};

static const CalPoint TABLE_3[] = {
  {12.59f, 10}, {19.30f, 15}, {26.86f, 20}, {32.66f, 25}, {37.98f, 30},
  {42.82f, 35}, {48.38f, 40}, {53.94f, 45}, {59.77f, 50}, {64.86f, 55},
  {69.81f, 60}, {75.14f, 65}, {80.60f, 70}, {85.00f, 75}, {89.74f, 80},
  {94.74f, 85}, {100.21f, 90}, {105.32f, 95}, {110.49f, 100}, {115.46f, 105},
  {120.82f, 110}, {125.41f, 115}, {129.92f, 120}, {135.64f, 125}, {140.43f, 130},
  {146.38f, 135}, {150.51f, 140}, {153.53f, 145}, {158.49f, 150}, {163.20f, 155},
  {168.16f, 160}, {172.55f, 165}, {176.67f, 170}, {181.49f, 175}, {186.39f, 180},
  {190.97f, 185}, {196.25f, 190}, {200.84f, 195}, {205.32f, 200}, {209.80f, 205},
  {214.57f, 210}, {219.26f, 215}, {224.25f, 220}, {228.19f, 225}, {232.35f, 230},
  {236.77f, 235}, {241.82f, 240}, {246.48f, 245}, {251.34f, 250}, {254.58f, 255},
  {258.94f, 260}, {262.33f, 265}, {265.84f, 270}, {266.20f, 275}, {273.96f, 280},
  {278.41f, 285},
};

float calibrate(float measured_mm, uint8_t sensor_num) {
  const CalPoint *table;
  size_t count;

  switch (sensor_num) {
    case 1: table = TABLE_1; count = sizeof(TABLE_1) / sizeof(TABLE_1[0]); break;
    case 2: table = TABLE_2; count = sizeof(TABLE_2) / sizeof(TABLE_2[0]); break;
    case 3: table = TABLE_3; count = sizeof(TABLE_3) / sizeof(TABLE_3[0]); break;
    default: return measured_mm;   // unknown sensor - fall back to raw value
  }

  // Below the lowest calibration point: extrapolate using the first
  // segment's slope.
  if (measured_mm <= table[0].measured_mm) {
    float a0 = table[0].actual_mm, m0 = table[0].measured_mm;
    float a1 = table[1].actual_mm, m1 = table[1].measured_mm;
    float slope = (a1 - a0) / (m1 - m0);
    return a0 + slope * (measured_mm - m0);
  }

  // Above the highest calibration point: extrapolate using the last
  // segment's slope.
  if (measured_mm >= table[count - 1].measured_mm) {
    float a0 = table[count - 2].actual_mm, m0 = table[count - 2].measured_mm;
    float a1 = table[count - 1].actual_mm, m1 = table[count - 1].measured_mm;
    float slope = (a1 - a0) / (m1 - m0);
    return a1 + slope * (measured_mm - m1);
  }

  // Otherwise, find the bracketing pair and interpolate between them.
  for (size_t i = 0; i < count - 1; i++) {
    float m0 = table[i].measured_mm, m1 = table[i + 1].measured_mm;
    if (m0 <= measured_mm && measured_mm <= m1) {
      float a0 = table[i].actual_mm, a1 = table[i + 1].actual_mm;
      float fraction = (measured_mm - m0) / (m1 - m0);
      return a0 + fraction * (a1 - a0);
    }
  }

  return measured_mm;   // shouldn't be reached
}