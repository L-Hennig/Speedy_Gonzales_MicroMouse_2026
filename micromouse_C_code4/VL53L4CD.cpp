/*
 * Filename: vl53l4cd.cpp
 * Description: C++ port of vl53l4cd.py for Arduino IDE (arduino-pico core).
 */
#include "vl53l4cd.h"

// --- Register addresses (matching the _VL53L4CD_* constants in vl53l4cd.py) ---
#define REG_I2C_SLAVE_DEVICE_ADDRESS        0x0001
#define REG_VHV_CONFIG_TIMEOUT_MACROP       0x0008
#define REG_OSC_FREQUENCY                   0x0006
#define REG_GPIO_HV_MUX_CTRL                0x0030
#define REG_GPIO_TIO_HV_STATUS              0x0031
#define REG_RANGE_CONFIG_A                  0x005E
#define REG_RANGE_CONFIG_B                  0x0061
#define REG_INTERMEASUREMENT_MS             0x006C
#define REG_SYSTEM_INTERRUPT_CLEAR          0x0086
#define REG_SYSTEM_START                    0x0087
#define REG_RESULT_DISTANCE                 0x0096
#define REG_RESULT_OSC_CALIBRATE_VAL        0x00DE
#define REG_FIRMWARE_SYSTEM_STATUS          0x00E5
#define REG_IDENTIFICATION_MODEL_ID         0x010F

// 91-byte default configuration sequence, written starting at register
// 0x002D - transcribed directly from vl53l4cd.py's init_seq.
static const uint8_t VL53L4CD_INIT_SEQUENCE[] = {
  0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
  0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0f,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
  0x14, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
  0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
  0xcc, 0x07, 0x01, 0xf1, 0x05, 0x00, 0xa0, 0x00, 0x80, 0x08,
  0x38, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00,
  0x00, 0x02, 0xc7, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00
};

VL53L4CD::VL53L4CD(TwoWire &wire, uint8_t address)
  : _wire(&wire), _address(address), _ranging(false) {}

void VL53L4CD::write_register(uint16_t reg, const uint8_t *data, uint8_t length) {
  _wire->beginTransmission(_address);
  _wire->write((uint8_t)(reg >> 8));
  _wire->write((uint8_t)(reg & 0xFF));
  for (uint8_t i = 0; i < length; i++) {
    _wire->write(data[i]);
  }
  _wire->endTransmission();
}

void VL53L4CD::read_register(uint16_t reg, uint8_t *data, uint8_t length) {
  _wire->beginTransmission(_address);
  _wire->write((uint8_t)(reg >> 8));
  _wire->write((uint8_t)(reg & 0xFF));
  _wire->endTransmission(false);   // repeated start - keep the bus held
  _wire->requestFrom((int)_address, (int)length);
  for (uint8_t i = 0; i < length && _wire->available(); i++) {
    data[i] = _wire->read();
  }
}

bool VL53L4CD::wait_for_boot() {
  for (int i = 0; i < 1000; i++) {
    uint8_t status;
    read_register(REG_FIRMWARE_SYSTEM_STATUS, &status, 1);
    if (status == 0x03) return true;
    delay(1);
  }
  return false;
}

bool VL53L4CD::start_vhv() {
  start_ranging();
  for (int i = 0; i < 1000; i++) {
    if (data_ready()) return true;
    delay(1);
  }
  return false;
}

bool VL53L4CD::begin() {
  uint8_t info[2];
  read_register(REG_IDENTIFICATION_MODEL_ID, info, 2);
  if (info[0] != 0xEB || info[1] != 0xAA) {
    return false;   // wrong sensor ID/type - matches Python's RuntimeError
  }

  _ranging = false;

  if (!wait_for_boot()) return false;

  write_register(0x002D, VL53L4CD_INIT_SEQUENCE, sizeof(VL53L4CD_INIT_SEQUENCE));

  if (!start_vhv()) return false;
  clear_interrupt();
  stop_ranging();

  uint8_t v = 0x09;
  write_register(REG_VHV_CONFIG_TIMEOUT_MACROP, &v, 1);
  v = 0x00;
  write_register(0x000B, &v, 1);
  uint8_t buf2[2] = { 0x05, 0x00 };
  write_register(0x0024, buf2, 2);

  set_inter_measurement_ms(0);
  set_timing_budget_ms(50);

  return true;
}

void VL53L4CD::start_ranging() {
  uint8_t val = (get_inter_measurement_ms() == 0) ? 0x21 : 0x40;
  write_register(REG_SYSTEM_START, &val, 1);

  // Python raises TimeoutError if this doesn't come ready in time; here
  // we just proceed best-effort after the timeout rather than throwing,
  // since exceptions aren't used in this port.
  for (int i = 0; i < 1000; i++) {
    if (data_ready()) break;
    delay(1);
  }
  clear_interrupt();
  _ranging = true;
}

void VL53L4CD::stop_ranging() {
  uint8_t val = 0x00;
  write_register(REG_SYSTEM_START, &val, 1);
  _ranging = false;
}

void VL53L4CD::clear_interrupt() {
  uint8_t val = 0x01;
  write_register(REG_SYSTEM_INTERRUPT_CLEAR, &val, 1);
}

uint8_t VL53L4CD::interrupt_polarity() {
  uint8_t val;
  read_register(REG_GPIO_HV_MUX_CTRL, &val, 1);
  uint8_t int_pol = (val & 0x10) >> 4;
  return int_pol ? 0 : 1;
}

bool VL53L4CD::data_ready() {
  uint8_t status;
  read_register(REG_GPIO_TIO_HV_STATUS, &status, 1);
  return (status & 0x01) == interrupt_polarity();
}

float VL53L4CD::distance_mm() {
  uint8_t buf[2];
  read_register(REG_RESULT_DISTANCE, buf, 2);
  uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
  return (float)raw;
}

float VL53L4CD::get_distance() {
  // The Python driver calls clear_interrupt() twice here with a comment
  // questioning why it's needed - kept as-is since it's cheap and known
  // to work, rather than risk removing something load-bearing.
  clear_interrupt();
  clear_interrupt();
  while (!data_ready()) {
    // busy-wait
  }
  return distance_mm();
}

uint16_t VL53L4CD::get_timing_budget_ms() {
  uint8_t buf[2];
  read_register(REG_OSC_FREQUENCY, buf, 2);
  uint16_t osc_freq = ((uint16_t)buf[0] << 8) | buf[1];
  if (osc_freq == 0) return 0;

  int64_t macro_period_us = ((int64_t)2304 * (1073741824LL / osc_freq)) >> 6;

  read_register(REG_RANGE_CONFIG_A, buf, 2);
  uint16_t macrop_high = ((uint16_t)buf[0] << 8) | buf[1];

  int64_t ls_byte = (int64_t)(macrop_high & 0x00FF) << 4;
  int64_t ms_byte = (macrop_high & 0xFF00) >> 8;
  ms_byte = 0x04 - (ms_byte - 1) - 1;

  int64_t timing_budget_us =
      (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12;

  if (ms_byte < 12 && ms_byte >= 0) {
    timing_budget_us >>= ms_byte;
  }

  if (get_inter_measurement_ms() == 0) {
    timing_budget_us += 2500;
  } else {
    timing_budget_us *= 2;
    timing_budget_us += 4300;
  }

  return (uint16_t)(timing_budget_us / 1000);
}

bool VL53L4CD::set_timing_budget_ms(uint16_t val) {
  if (_ranging) return false;               // "must stop ranging first"
  if (val < 10 || val > 200) return false;  // valid range is 10-200ms

  uint16_t inter_meas = get_inter_measurement_ms();
  if (inter_meas != 0 && val > inter_meas) return false;

  uint8_t buf[2];
  read_register(REG_OSC_FREQUENCY, buf, 2);
  uint16_t osc_freq = ((uint16_t)buf[0] << 8) | buf[1];
  if (osc_freq == 0) return false;

  int64_t timing_budget_us = (int64_t)val * 1000;
  int64_t macro_period_us = ((int64_t)2304 * (1073741824LL / osc_freq)) >> 6;

  if (inter_meas == 0) {
    timing_budget_us -= 2500;
  } else {
    timing_budget_us -= 4300;
  }
  timing_budget_us /= 2;
  timing_budget_us <<= 12;   // shared by both register calculations below

  auto compute_reg = [&](int64_t macro_mult) -> uint16_t {
    int64_t tmp = macro_period_us * macro_mult;
    int64_t ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;
    int64_t ms_byte = 0;
    while (ls_byte & 0xFFFFFF00) {
      ls_byte >>= 1;
      ms_byte += 1;
    }
    return (uint16_t)((ms_byte << 8) + (ls_byte & 0xFF));
  };

  uint16_t config_a = compute_reg(16);
  uint8_t out_a[2] = { (uint8_t)(config_a >> 8), (uint8_t)(config_a & 0xFF) };
  write_register(REG_RANGE_CONFIG_A, out_a, 2);

  uint16_t config_b = compute_reg(12);
  uint8_t out_b[2] = { (uint8_t)(config_b >> 8), (uint8_t)(config_b & 0xFF) };
  write_register(REG_RANGE_CONFIG_B, out_b, 2);

  return true;
}

uint16_t VL53L4CD::get_inter_measurement_ms() {
  uint8_t buf[4];
  read_register(REG_INTERMEASUREMENT_MS, buf, 4);
  uint32_t reg_val = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                      ((uint32_t)buf[2] << 8) | buf[3];

  uint8_t buf2[2];
  read_register(REG_RESULT_OSC_CALIBRATE_VAL, buf2, 2);
  uint16_t clock_pll = (((uint16_t)buf2[0] << 8) | buf2[1]) & 0x3FF;
  clock_pll = (uint16_t)(1.065f * clock_pll);
  if (clock_pll == 0) return 0;

  return (uint16_t)(reg_val / clock_pll);
}

bool VL53L4CD::set_inter_measurement_ms(uint16_t val) {
  if (_ranging) return false;

  uint16_t timing_bud = get_timing_budget_ms();
  if (val != 0 && val < timing_bud) return false;

  uint8_t buf2[2];
  read_register(REG_RESULT_OSC_CALIBRATE_VAL, buf2, 2);
  uint16_t clock_pll = (((uint16_t)buf2[0] << 8) | buf2[1]) & 0x3FF;

  uint32_t int_meas = (uint32_t)(1.055f * val * clock_pll);
  uint8_t out[4] = {
    (uint8_t)((int_meas >> 24) & 0xFF),
    (uint8_t)((int_meas >> 16) & 0xFF),
    (uint8_t)((int_meas >> 8) & 0xFF),
    (uint8_t)(int_meas & 0xFF)
  };
  write_register(REG_INTERMEASUREMENT_MS, out, 4);

  // Re-apply timing budget, since it's derived from inter-measurement.
  set_timing_budget_ms(timing_bud);
  return true;
}

void VL53L4CD::set_address(uint8_t new_address) {
  uint8_t val = new_address;
  write_register(REG_I2C_SLAVE_DEVICE_ADDRESS, &val, 1);
  _address = new_address;
}