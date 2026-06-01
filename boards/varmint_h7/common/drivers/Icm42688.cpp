#include "Icm42688.h"

#include <cstring>

namespace
{
constexpr uint32_t SPI_TIMEOUT_MS = 10U;
constexpr uint8_t WHO_AM_I_VALUE = 0x47U;
constexpr uint8_t REG_DEVICE_CONFIG = 0x11U;
constexpr uint8_t REG_INT_CONFIG = 0x14U;
constexpr uint8_t REG_TEMP_DATA1 = 0x1DU;
constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x4BU;
constexpr uint8_t REG_PWR_MGMT0 = 0x4EU;
constexpr uint8_t REG_GYRO_CONFIG0 = 0x4FU;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50U;
constexpr uint8_t REG_INT_CONFIG1 = 0x64U;
constexpr uint8_t REG_INT_SOURCE0 = 0x65U;
constexpr uint8_t REG_WHO_AM_I = 0x75U;
constexpr uint8_t REG_BANK_SEL = 0x76U;
constexpr uint8_t READ = 0x80U;
constexpr uint8_t WRITE = 0x00U;
constexpr uint8_t ODR_1KHZ = 0x06U;
constexpr uint8_t GYRO_FS_500DPS = 0x40U;
constexpr uint8_t ACCEL_FS_8G = 0x20U;
constexpr uint8_t GYRO_CONFIG0_VALUE = GYRO_FS_500DPS | ODR_1KHZ;
constexpr uint8_t ACCEL_CONFIG0_VALUE = ACCEL_FS_8G | ODR_1KHZ;
constexpr uint8_t PWR_MGMT0_LN_GYRO_ACCEL = 0x0FU;
constexpr uint8_t INT_CONFIG_VALUE = 0x1BU;
constexpr uint8_t INT_SOURCE0_DRDY_INT1 = 0x08U;
constexpr uint8_t INT_CONFIG1_ASYNC_RESET_BIT = 0x10U;
constexpr float GRAVITY_MSS = 9.80665f;
constexpr float ACCEL_SCALE = GRAVITY_MSS / 4096.0f; // +/-8g register mode
constexpr float GYRO_SCALE = 0.01745329252f / 65.5f; // +/-500 dps register mode
constexpr uint32_t SENSOR_ERROR_IMU = 0x0001U;
constexpr uint32_t SENSOR_ERROR_IMU_CONFIG = 0x0004U;

int16_t read_i16_be(const uint8_t * bytes)
{
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}
} // namespace

bool Icm42688::init(const Config & config)
{
  config_ = config;
  initialized_ = false;
  dma_busy_ = false;
  new_sample_ = false;
  init_status_ = SENSOR_ERROR_IMU;
  who_am_i_ = 0;
  last_drdy_time_us_ = 0;

  if (config_.hspi == nullptr || config_.cs_port == nullptr || config_.dma_tx_buffer == nullptr
      || config_.dma_rx_buffer == nullptr || config_.dma_buffer_size < BURST_BYTES
      || config_.clock_micros == nullptr || config_.delay_ms == nullptr) {
    return false;
  }

  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_SET);
  delay_ms(3);

  write_register(REG_DEVICE_CONFIG, 0x01U);
  delay_ms(2);
  write_register(REG_BANK_SEL, 0x00U);

  const uint8_t who_am_i = read_register(REG_WHO_AM_I);
  if (who_am_i != WHO_AM_I_VALUE) {
    who_am_i_ = who_am_i;
    return false;
  }

  write_register(REG_PWR_MGMT0, 0x00U);
  delay_ms(1);
  write_register(REG_GYRO_CONFIG0, GYRO_CONFIG0_VALUE);
  write_register(REG_ACCEL_CONFIG0, ACCEL_CONFIG0_VALUE);
  write_register(REG_SIGNAL_PATH_RESET, 0x0AU);
  delay_ms(1);
  update_register(REG_INT_CONFIG1, INT_CONFIG1_ASYNC_RESET_BIT, 0x00U);
  write_register(REG_INT_CONFIG, INT_CONFIG_VALUE);
  write_register(REG_INT_SOURCE0, INT_SOURCE0_DRDY_INT1);
  write_register(REG_PWR_MGMT0, PWR_MGMT0_LN_GYRO_ACCEL);
  delay_ms(50);

  if (!verify_configuration()) {
    init_status_ = SENSOR_ERROR_IMU_CONFIG;
    return false;
  }

  initialized_ = true;
  init_status_ = 0;
  return true;
}

void Icm42688::start_dma()
{
  if (!initialized_ || dma_busy_) { return; }
  last_drdy_time_us_ = clock_micros();

  std::memset(config_.dma_tx_buffer, 0, config_.dma_buffer_size);
  config_.dma_tx_buffer[0] = REG_TEMP_DATA1 | READ;

  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive_DMA(config_.hspi, config_.dma_tx_buffer, config_.dma_rx_buffer,
                                  BURST_BYTES) == HAL_OK) {
    dma_busy_ = true;
  } else {
    HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_SET);
  }
}

void Icm42688::finish_dma()
{
  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_SET);
  if (!dma_busy_) { return; }
  dma_busy_ = false;

  rosflight_firmware::ImuStruct sample = {};
  const uint64_t now = clock_micros();
  sample.header.timestamp = last_drdy_time_us_ == 0 ? now : last_drdy_time_us_;
  sample.header.complete = now;
  sample.header.status = 0;

  const int16_t temp_raw = read_i16_be(&config_.dma_rx_buffer[1]);
  const int16_t accel_x = read_i16_be(&config_.dma_rx_buffer[3]);
  const int16_t accel_y = read_i16_be(&config_.dma_rx_buffer[5]);
  const int16_t accel_z = read_i16_be(&config_.dma_rx_buffer[7]);
  const int16_t gyro_x = read_i16_be(&config_.dma_rx_buffer[9]);
  const int16_t gyro_y = read_i16_be(&config_.dma_rx_buffer[11]);
  const int16_t gyro_z = read_i16_be(&config_.dma_rx_buffer[13]);

  sample.temperature = (static_cast<float>(temp_raw) / 132.48f) + 25.0f + 273.15f;
  sample.accel[0] = static_cast<float>(accel_x) * ACCEL_SCALE;
  sample.accel[1] = static_cast<float>(accel_y) * ACCEL_SCALE;
  sample.accel[2] = static_cast<float>(accel_z) * ACCEL_SCALE;
  sample.gyro[0] = static_cast<float>(gyro_x) * GYRO_SCALE;
  sample.gyro[1] = static_cast<float>(gyro_y) * GYRO_SCALE;
  sample.gyro[2] = static_cast<float>(gyro_z) * GYRO_SCALE;
  rotate_to_body(sample.accel);
  rotate_to_body(sample.gyro);

  latest_sample_ = sample;
  new_sample_ = true;
}

void Icm42688::abort_dma()
{
  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_SET);
  dma_busy_ = false;
}

bool Icm42688::read(rosflight_firmware::ImuStruct * imu)
{
  if (!initialized_ || imu == nullptr) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = new_sample_;
  if (have_sample) {
    *imu = latest_sample_;
    new_sample_ = false;
  }
  if (!primask) { __enable_irq(); }

  return have_sample;
}

uint8_t Icm42688::transfer_byte(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  uint8_t rx[2] = {};
  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_RESET);
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(config_.hspi, tx, rx, sizeof(tx), SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(config_.cs_port, config_.cs_pin, GPIO_PIN_SET);
  return (status == HAL_OK) ? rx[1] : 0U;
}

uint8_t Icm42688::read_register(uint8_t reg) { return transfer_byte(reg | READ, 0U); }

void Icm42688::write_register(uint8_t reg, uint8_t value)
{
  (void) transfer_byte(reg | WRITE, value);
  for (volatile uint32_t i = 0; i < 2000U; i++) {}
}

void Icm42688::update_register(uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
  const uint8_t value = read_register(reg);
  write_register(reg, static_cast<uint8_t>((value & ~clear_mask) | set_mask));
}

bool Icm42688::verify_configuration()
{
  if (read_register(REG_GYRO_CONFIG0) != GYRO_CONFIG0_VALUE) { return false; }
  if (read_register(REG_ACCEL_CONFIG0) != ACCEL_CONFIG0_VALUE) { return false; }
  if (read_register(REG_INT_CONFIG) != INT_CONFIG_VALUE) { return false; }
  if (read_register(REG_INT_SOURCE0) != INT_SOURCE0_DRDY_INT1) { return false; }
  if ((read_register(REG_INT_CONFIG1) & INT_CONFIG1_ASYNC_RESET_BIT) != 0U) { return false; }
  if ((read_register(REG_PWR_MGMT0) & PWR_MGMT0_LN_GYRO_ACCEL) != PWR_MGMT0_LN_GYRO_ACCEL) {
    return false;
  }
  return true;
}

void Icm42688::rotate_to_body(float * v) const
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

uint64_t Icm42688::clock_micros() const { return config_.clock_micros(); }

void Icm42688::delay_ms(uint32_t milliseconds) const { config_.delay_ms(milliseconds); }
