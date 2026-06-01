#include "Ist8310.h"

#include <cstring>

namespace
{
constexpr uint32_t I2C_TIMEOUT_MS = 10U;
// Register map from mag_spec.md, with conventional IST8310 addresses cross-checked against PX4.
constexpr uint8_t REG_WIA = 0x00U;
constexpr uint8_t REG_STAT1 = 0x02U;
constexpr uint8_t REG_CNTL1 = 0x0AU;
constexpr uint8_t REG_STR = 0x0CU;
constexpr uint8_t REG_TCCNTL = 0x40U;
constexpr uint8_t REG_AVGCNTL = 0x41U;
constexpr uint8_t REG_PDCNTL = 0x42U;
constexpr uint8_t WIA_VALUE = 0x10U;
constexpr uint8_t STAT1_DRDY = 0x01U;
constexpr uint8_t CNTL1_SINGLE_MEASUREMENT = 0x01U;
constexpr uint8_t STR_SELF_TEST_DISABLED = 0x00U;
constexpr uint8_t TCCNTL_TEMP_COMP_ENABLED = 0x00U;
constexpr uint8_t AVGCNTL_16X_AVERAGE = 0x24U;
constexpr uint8_t PDCNTL_PERFORMANCE = 0xC0U;
constexpr uint32_t STATUS_POLL_PERIOD_US = 1000U;
constexpr float UT_TO_T = 1.0e-6f;
constexpr float COUNTS_PER_UT = 3.3f;
constexpr uint32_t SENSOR_ERROR_MAG = 0x0008U;
constexpr uint32_t SENSOR_ERROR_MAG_ID = 0x0010U;
constexpr uint32_t SENSOR_ERROR_MAG_CONFIG = 0x0020U;
constexpr uint32_t SENSOR_ERROR_MAG_TIMEOUT = 0x0040U;

int16_t read_i16_le(const uint8_t low, const uint8_t high)
{
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}
} // namespace

bool Ist8310::init(const Config & config)
{
  config_ = config;
  initialized_ = false;
  new_sample_ = false;
  measurement_active_ = false;
  init_status_ = SENSOR_ERROR_MAG;
  who_am_i_ = 0;
  last_start_time_us_ = 0;
  next_sample_time_us_ = 0;
  next_status_check_time_us_ = 0;
  latest_sample_ = {};

  if (config_.hi2c == nullptr || config_.clock_micros == nullptr || config_.delay_ms == nullptr) {
    return false;
  }

  delay_ms(50);

  uint8_t who_am_i = 0;
  if (!read_register(REG_WIA, &who_am_i)) { return false; }
  who_am_i_ = who_am_i;
  if (who_am_i != WIA_VALUE) {
    init_status_ = SENSOR_ERROR_MAG_ID;
    return false;
  }

  if (!write_register(REG_CNTL1, 0x00U)
      || !write_register(REG_PDCNTL, PDCNTL_PERFORMANCE)
      || !write_register(REG_AVGCNTL, AVGCNTL_16X_AVERAGE)
      || !write_register(REG_STR, STR_SELF_TEST_DISABLED)
      || !write_register(REG_TCCNTL, TCCNTL_TEMP_COMP_ENABLED)) {
    init_status_ = SENSOR_ERROR_MAG_CONFIG;
    return false;
  }

  initialized_ = true;
  init_status_ = 0;
  if (!start_measurement()) {
    initialized_ = false;
    return false;
  }
  return true;
}

void Ist8310::poll()
{
  if (!initialized_) { return; }

  const uint64_t now = clock_micros();
  if (!measurement_active_) {
    if (now >= next_sample_time_us_) { (void) start_measurement(); }
    return;
  }

  if (now < next_status_check_time_us_) { return; }
  next_status_check_time_us_ = now + STATUS_POLL_PERIOD_US;

  uint8_t status = 0;
  if (!read_register(REG_STAT1, &status)) {
    measurement_active_ = false;
    init_status_ = SENSOR_ERROR_MAG;
    next_sample_time_us_ = now + config_.sample_period_us;
    return;
  }

  if ((status & STAT1_DRDY) != 0U) {
    measurement_active_ = false;
    if (read_sample()) {
      init_status_ = 0;
    } else {
      init_status_ = SENSOR_ERROR_MAG;
    }
    next_sample_time_us_ = last_start_time_us_ + config_.sample_period_us;
    return;
  }

  if ((now - last_start_time_us_) > config_.conversion_timeout_us) {
    measurement_active_ = false;
    init_status_ = SENSOR_ERROR_MAG_TIMEOUT;
    (void) write_register(REG_CNTL1, 0x00U);
    next_sample_time_us_ = now + config_.sample_period_us;
  }
}

bool Ist8310::read(rosflight_firmware::MagStruct * mag)
{
  if (!initialized_ || mag == nullptr) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = new_sample_;
  if (have_sample) {
    *mag = latest_sample_;
    new_sample_ = false;
  }
  if (!primask) { __enable_irq(); }

  return have_sample;
}

bool Ist8310::read_register(uint8_t reg, uint8_t * value)
{
  return read_registers(reg, value, 1U);
}

bool Ist8310::write_register(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  return HAL_I2C_Master_Transmit(config_.hi2c, config_.i2c_address << 1, tx, sizeof(tx),
                                 I2C_TIMEOUT_MS) == HAL_OK;
}

bool Ist8310::read_registers(uint8_t start_reg, uint8_t * dest, size_t len)
{
  if (dest == nullptr || len == 0U) { return false; }
  if (HAL_I2C_Master_Transmit(config_.hi2c, config_.i2c_address << 1, &start_reg, 1U,
                              I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return HAL_I2C_Master_Receive(config_.hi2c, config_.i2c_address << 1, dest,
                                static_cast<uint16_t>(len), I2C_TIMEOUT_MS) == HAL_OK;
}

bool Ist8310::start_measurement()
{
  if (!write_register(REG_CNTL1, CNTL1_SINGLE_MEASUREMENT)) {
    init_status_ = SENSOR_ERROR_MAG;
    return false;
  }

  measurement_active_ = true;
  last_start_time_us_ = clock_micros();
  next_status_check_time_us_ = last_start_time_us_ + STATUS_POLL_PERIOD_US;
  return true;
}

bool Ist8310::read_sample()
{
  uint8_t buffer[8] = {};
  if (!read_registers(REG_STAT1, buffer, sizeof(buffer))) { return false; }
  if ((buffer[0] & STAT1_DRDY) == 0U) { return false; }

  rosflight_firmware::MagStruct sample = {};
  const uint64_t now = clock_micros();
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.header.status = static_cast<uint16_t>((static_cast<uint16_t>(buffer[7]) << 8) | buffer[0]);

  const int16_t raw_x = read_i16_le(buffer[1], buffer[2]);
  const int16_t raw_y = read_i16_le(buffer[3], buffer[4]);
  const int16_t raw_z = read_i16_le(buffer[5], buffer[6]);
  sample.flux[0] = (static_cast<float>(raw_x) / COUNTS_PER_UT) * UT_TO_T;
  sample.flux[1] = (static_cast<float>(raw_y) / COUNTS_PER_UT) * UT_TO_T;
  sample.flux[2] = (static_cast<float>(raw_z) / COUNTS_PER_UT) * UT_TO_T;
  sample.temperature = 0.0f;
  rotate_to_body(sample.flux);

  latest_sample_ = sample;
  new_sample_ = true;
  return true;
}

void Ist8310::rotate_to_body(float * v) const
{
  if (config_.rotation_sensor_to_body == nullptr) { return; }

  const float x = v[0];
  const float y = v[1];
  const float z = v[2];
  const float * r = config_.rotation_sensor_to_body;

  v[0] = r[0] * x + r[1] * y + r[2] * z;
  v[1] = r[3] * x + r[4] * y + r[5] * z;
  v[2] = r[6] * x + r[7] * y + r[8] * z;
}

uint64_t Ist8310::clock_micros() const { return config_.clock_micros(); }

void Ist8310::delay_ms(uint32_t milliseconds) const { config_.delay_ms(milliseconds); }
