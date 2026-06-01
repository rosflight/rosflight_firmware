#include "Ms5611.h"

namespace
{
constexpr uint32_t I2C_TIMEOUT_MS = 10U;
constexpr uint8_t ADDRESS_0 = 0x76U;
constexpr uint8_t ADDRESS_1 = 0x77U;
constexpr uint8_t CMD_RESET = 0x1EU;
constexpr uint8_t CMD_ADC_READ = 0x00U;
constexpr uint8_t CMD_CONVERT_D1 = 0x40U;
constexpr uint8_t CMD_CONVERT_D2 = 0x50U;
constexpr uint8_t CMD_PROM_READ = 0xA0U;
constexpr uint32_t RESET_DELAY_MS = 3U;
constexpr float CENTIDEG_TO_DEG = 0.01f;
constexpr float DEG_C_TO_K = 273.15f;
constexpr uint32_t SENSOR_ERROR_BARO = 0x0100U;
constexpr uint32_t SENSOR_ERROR_BARO_PROM = 0x0200U;
constexpr uint32_t SENSOR_ERROR_BARO_CRC = 0x0400U;
constexpr uint32_t SENSOR_ERROR_BARO_SAMPLE = 0x0800U;

uint16_t ms5611_crc4(const uint16_t * prom)
{
  uint16_t prom_copy[8] = {};
  for (uint8_t i = 0; i < 8U; i++) { prom_copy[i] = prom[i]; }
  prom_copy[7] &= 0xFF00U;

  uint16_t n_rem = 0;
  for (uint8_t cnt = 0; cnt < 16U; cnt++) {
    if ((cnt & 1U) == 0U) {
      n_rem ^= static_cast<uint16_t>(prom_copy[cnt >> 1U] >> 8U);
    } else {
      n_rem ^= static_cast<uint16_t>(prom_copy[cnt >> 1U] & 0x00FFU);
    }

    for (uint8_t bit = 0; bit < 8U; bit++) {
      if ((n_rem & 0x8000U) != 0U) {
        n_rem = static_cast<uint16_t>((n_rem << 1U) ^ 0x3000U);
      } else {
        n_rem = static_cast<uint16_t>(n_rem << 1U);
      }
    }
  }

  return static_cast<uint16_t>((n_rem >> 12U) & 0x000FU);
}
} // namespace

bool Ms5611::init(const Config & config)
{
  config_ = config;
  initialized_ = false;
  new_sample_ = false;
  init_status_ = SENSOR_ERROR_BARO;
  i2c_address_ = 0;
  d1_ = 0;
  d2_ = 0;
  pressure_count_ = 0;
  state_ = State::IDLE;
  conversion_deadline_us_ = 0;
  latest_sample_ = {};
  for (uint8_t i = 0; i < 8U; i++) { prom_[i] = 0; }

  if (config_.hi2c == nullptr || config_.clock_micros == nullptr || config_.delay_ms == nullptr) {
    return false;
  }
  if (config_.temperature_decimation == 0U) { config_.temperature_decimation = 1U; }

  if (!probe_address(ADDRESS_0) && !probe_address(ADDRESS_1)) { return false; }

  if (!take_initial_sample()) {
    initialized_ = false;
    init_status_ = SENSOR_ERROR_BARO_SAMPLE;
    return false;
  }

  initialized_ = true;
  init_status_ = 0;
  if (!start_pressure_conversion()) {
    initialized_ = false;
    return false;
  }
  return true;
}

void Ms5611::poll()
{
  if (!initialized_) { return; }

  const uint64_t now = clock_micros();
  if (state_ == State::IDLE) {
    (void) start_pressure_conversion();
    return;
  }
  if (now < conversion_deadline_us_) { return; }

  uint32_t adc = 0;
  if (!read_adc(&adc) || adc == 0U) {
    state_ = State::IDLE;
    init_status_ = SENSOR_ERROR_BARO_SAMPLE;
    return;
  }

  if (state_ == State::D1_CONVERTING) {
    d1_ = adc;
    pressure_count_++;
    if (pressure_count_ >= config_.temperature_decimation) {
      (void) start_temperature_conversion();
    } else {
      if (compute_sample()) { init_status_ = 0; }
      (void) start_pressure_conversion();
    }
    return;
  }

  if (state_ == State::D2_CONVERTING) {
    d2_ = adc;
    pressure_count_ = 0;
    if (compute_sample()) { init_status_ = 0; }
    (void) start_pressure_conversion();
  }
}

bool Ms5611::read(rosflight_firmware::PressureStruct * baro)
{
  if (!initialized_ || baro == nullptr) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = new_sample_;
  if (have_sample) {
    *baro = latest_sample_;
    new_sample_ = false;
  }
  if (!primask) { __enable_irq(); }

  return have_sample;
}

bool Ms5611::probe_address(uint8_t address)
{
  i2c_address_ = address;
  if (!reset()) { return false; }
  delay_ms(RESET_DELAY_MS);
  if (!read_prom()) {
    init_status_ = SENSOR_ERROR_BARO_PROM;
    return false;
  }
  if (!prom_sane()) {
    init_status_ = SENSOR_ERROR_BARO_PROM;
    return false;
  }
  if (!prom_crc_ok()) {
    init_status_ = SENSOR_ERROR_BARO_CRC;
    return false;
  }
  return true;
}

bool Ms5611::reset()
{
  return command(CMD_RESET);
}

bool Ms5611::read_prom()
{
  for (uint8_t i = 0; i < 8U; i++) {
    if (!read_prom_word(i, &prom_[i])) { return false; }
  }
  return true;
}

bool Ms5611::prom_sane() const
{
  bool all_zero = true;
  bool all_ones = true;
  for (uint8_t i = 1; i <= 6U; i++) {
    all_zero = all_zero && (prom_[i] == 0U);
    all_ones = all_ones && (prom_[i] == 0xFFFFU);
  }
  return !all_zero && !all_ones;
}

bool Ms5611::prom_crc_ok() const
{
  return ms5611_crc4(prom_) == (prom_[7] & 0x000FU);
}

bool Ms5611::command(uint8_t cmd)
{
  return HAL_I2C_Master_Transmit(config_.hi2c, i2c_address_ << 1U, &cmd, 1U,
                                 I2C_TIMEOUT_MS) == HAL_OK;
}

bool Ms5611::read_prom_word(uint8_t index, uint16_t * value)
{
  if (value == nullptr || index >= 8U) { return false; }

  uint8_t cmd = static_cast<uint8_t>(CMD_PROM_READ + (2U * index));
  uint8_t rx[2] = {};
  if (HAL_I2C_Master_Transmit(config_.hi2c, i2c_address_ << 1U, &cmd, 1U,
                              I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Master_Receive(config_.hi2c, i2c_address_ << 1U, rx, sizeof(rx),
                             I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  *value = static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8U) | rx[1]);
  return true;
}

bool Ms5611::read_adc(uint32_t * value)
{
  if (value == nullptr) { return false; }

  uint8_t cmd = CMD_ADC_READ;
  uint8_t rx[3] = {};
  if (HAL_I2C_Master_Transmit(config_.hi2c, i2c_address_ << 1U, &cmd, 1U,
                              I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Master_Receive(config_.hi2c, i2c_address_ << 1U, rx, sizeof(rx),
                             I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  *value = (static_cast<uint32_t>(rx[0]) << 16U) | (static_cast<uint32_t>(rx[1]) << 8U) | rx[2];
  return true;
}

bool Ms5611::start_pressure_conversion()
{
  if (!command(d1_command())) {
    init_status_ = SENSOR_ERROR_BARO;
    state_ = State::IDLE;
    return false;
  }
  state_ = State::D1_CONVERTING;
  conversion_deadline_us_ = clock_micros() + conversion_time_us();
  return true;
}

bool Ms5611::start_temperature_conversion()
{
  if (!command(d2_command())) {
    init_status_ = SENSOR_ERROR_BARO;
    state_ = State::IDLE;
    return false;
  }
  state_ = State::D2_CONVERTING;
  conversion_deadline_us_ = clock_micros() + conversion_time_us();
  return true;
}

bool Ms5611::take_initial_sample()
{
  if (!command(d2_command())) { return false; }
  delay_ms((conversion_time_us() + 999U) / 1000U);
  if (!read_adc(&d2_) || d2_ == 0U) { return false; }

  if (!command(d1_command())) { return false; }
  delay_ms((conversion_time_us() + 999U) / 1000U);
  if (!read_adc(&d1_) || d1_ == 0U) { return false; }

  return compute_sample();
}

bool Ms5611::compute_sample()
{
  if (d1_ == 0U || d2_ == 0U) { return false; }

  const int64_t d_t = static_cast<int64_t>(d2_) - (static_cast<int64_t>(prom_[5]) << 8);
  int64_t temp = 2000 + ((d_t * static_cast<int64_t>(prom_[6])) >> 23);
  int64_t off = (static_cast<int64_t>(prom_[2]) << 16) + ((static_cast<int64_t>(prom_[4]) * d_t) >> 7);
  int64_t sens = (static_cast<int64_t>(prom_[1]) << 15) + ((static_cast<int64_t>(prom_[3]) * d_t) >> 8);

  if (temp < 2000) {
    const int64_t temp_minus_2000 = temp - 2000;
    const int64_t temp_minus_2000_sq = temp_minus_2000 * temp_minus_2000;
    int64_t t2 = (d_t * d_t) >> 31;
    int64_t off2 = (5 * temp_minus_2000_sq) >> 1;
    int64_t sens2 = (5 * temp_minus_2000_sq) >> 2;

    if (temp < -1500) {
      const int64_t temp_plus_1500 = temp + 1500;
      const int64_t temp_plus_1500_sq = temp_plus_1500 * temp_plus_1500;
      off2 += 7 * temp_plus_1500_sq;
      sens2 += (11 * temp_plus_1500_sq) >> 1;
    }

    temp -= t2;
    off -= off2;
    sens -= sens2;
  }

  const int64_t pressure = (((static_cast<int64_t>(d1_) * sens) >> 21) - off) >> 15;
  const float pressure_pa = static_cast<float>(pressure);
  const float temperature_c = static_cast<float>(temp) * CENTIDEG_TO_DEG;

  if (!pressure_plausible(pressure_pa, temperature_c)) {
    init_status_ = SENSOR_ERROR_BARO_SAMPLE;
    return false;
  }

  const uint64_t now = clock_micros();
  rosflight_firmware::PressureStruct sample = {};
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.header.status = static_cast<uint16_t>(i2c_address_);
  sample.pressure = pressure_pa;
  sample.temperature = temperature_c + DEG_C_TO_K;
  sample.altitude = 0.0f;

  latest_sample_ = sample;
  new_sample_ = true;
  return true;
}

bool Ms5611::pressure_plausible(float pressure_pa, float temperature_c) const
{
  return pressure_pa >= 1000.0f && pressure_pa <= 120000.0f
    && temperature_c >= -40.0f && temperature_c <= 85.0f;
}

uint8_t Ms5611::d1_command() const
{
  return static_cast<uint8_t>(CMD_CONVERT_D1 + (static_cast<uint8_t>(config_.osr) * 2U));
}

uint8_t Ms5611::d2_command() const
{
  return static_cast<uint8_t>(CMD_CONVERT_D2 + (static_cast<uint8_t>(config_.osr) * 2U));
}

uint32_t Ms5611::conversion_time_us() const
{
  switch (config_.osr) {
    case Osr::OSR_256:
      return 1000U;
    case Osr::OSR_512:
      return 2000U;
    case Osr::OSR_1024:
      return 3000U;
    case Osr::OSR_2048:
      return 5000U;
    case Osr::OSR_4096:
    default:
      return 10000U;
  }
}

uint64_t Ms5611::clock_micros() const { return config_.clock_micros(); }

void Ms5611::delay_ms(uint32_t milliseconds) const { config_.delay_ms(milliseconds); }
