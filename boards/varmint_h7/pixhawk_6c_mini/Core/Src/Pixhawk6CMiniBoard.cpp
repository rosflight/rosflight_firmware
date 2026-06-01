#include "Pixhawk6CMiniBoard.h"

#include "Icm42688.h"
#include "Ist8310.h"
#include "Packets.h"
#include "main.h"
#include "usbd_cdc_acm_if.h"
#include "usb_device.h"

#include <algorithm>
#include <cstring>
#include <cstdio>

namespace
{
constexpr uint32_t TIMER_HZ = 1000000U;
constexpr uint16_t VCP_TX_FIFO_BUFFERS = 256;
constexpr uint16_t VCP_RX_FIFO_BUFFER_BYTES = 8 * 1024;
constexpr uint32_t SD_MAXBLKS = 16U;
constexpr uint32_t SD_BLKSIZE = 512U;
constexpr uint32_t SD_BUFF_SIZE = SD_MAXBLKS * SD_BLKSIZE;
constexpr uint32_t SD_CRC_BYTES = sizeof(uint32_t);
constexpr uint32_t SD_TIMEOUT_MS = 1000U;
constexpr uint64_t VCP_QUEUE_TIMEOUT_US = 5000U;
constexpr uint64_t BOARD_LOOP_PERIOD_US = 2500U;
constexpr uint64_t DIFF_PRESSURE_PERIOD_US = 50000U;
constexpr uint64_t BATTERY_SAMPLE_PERIOD_US = 100000U;
constexpr uint32_t MAG_SAMPLE_PERIOD_US = 10000U;
constexpr uint32_t MAG_TIMEOUT_US = 20000U;
constexpr uint8_t IST8310_I2C_ADDRESS = 0x0CU;
constexpr uint32_t SENSOR_ERROR_BATTERY = 0x0002U;
constexpr float BATTERY_VOLTAGE_SCALE = 12.62f;
constexpr float BATTERY_CURRENT_SCALE = 60.5f;
constexpr float BATTERY_CURRENT_OFFSET = 0.0747f;
constexpr uint32_t BATTERY_ADC_CHANNELS = 4U;

constexpr float IMU_TO_FMU_ROTATION[9] = {
  0.0f, -1.0f, 0.0f,
  -1.0f, 0.0f, 0.0f,
  0.0f, 0.0f, -1.0f,
};

constexpr float MAG_TO_FMU_ROTATION[9] = {
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 1.0f,
};

DTCM_RAM uint8_t vcp_fifo_tx_buffer[VCP_TX_FIFO_BUFFERS * sizeof(SerialTxPacket)];
uint8_t vcp_fifo_rx_buffer[VCP_RX_FIFO_BUFFER_BYTES];
SD_DMA_RAM uint8_t sd_rx_buf[SD_BUFF_SIZE];
SD_DMA_RAM uint8_t sd_tx_buf[SD_BUFF_SIZE];
DMA_RAM uint8_t icm42688_dma_tx_buf[Icm42688::BURST_BYTES];
DMA_RAM uint8_t icm42688_dma_rx_buf[Icm42688::BURST_BYTES];
DMA_RAM uint32_t battery_adc_buf[BATTERY_ADC_CHANNELS];

uint64_t pixhawk_clock_micros_raw()
{
  static uint32_t last_count = 0;
  static uint64_t high_count = 0;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint32_t now_count = TIM2->CNT;
  if (now_count < last_count) { high_count += (1ULL << 32); }
  last_count = now_count;
  const uint64_t now = high_count + now_count;

  if (!primask) { __enable_irq(); }

  return now;
}

void pixhawk_clock_init()
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1) { timer_clock_hz *= 2U; }

  TIM2->CR1 = 0;
  TIM2->PSC = (timer_clock_hz / TIMER_HZ) - 1U;
  TIM2->ARR = 0xFFFFFFFFU;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CNT = 0;
  TIM2->CR1 = TIM_CR1_CEN;
  pixhawk_clock_micros_raw();
}

bool sd_wait_ready(uint32_t timeout_ms)
{
  const uint32_t start_ms = HAL_GetTick();
  while ((HAL_GetTick() - start_ms) < timeout_ms) {
    if (HAL_SD_GetCardState(&hsd2) == HAL_SD_CARD_TRANSFER) { return true; }
  }
  return false;
}

uint32_t sd_blocks_for_len(size_t len)
{
  return static_cast<uint32_t>((len + SD_CRC_BYTES + SD_BLKSIZE - 1U) / SD_BLKSIZE);
}

void pace_board_loop(Vcp & vcp)
{
  static uint64_t next_loop_time = 0;
  uint64_t now = pixhawk_clock_micros_raw();

  if (next_loop_time == 0) {
    next_loop_time = now + BOARD_LOOP_PERIOD_US;
    return;
  }

  while (now < next_loop_time) {
    vcp.poll();
    now = pixhawk_clock_micros_raw();
  }

  next_loop_time += BOARD_LOOP_PERIOD_US;
  if (next_loop_time < now) { next_loop_time = now + BOARD_LOOP_PERIOD_US; }
}

class BatteryMonitor
{
public:
  bool init()
  {
    initialized_ = false;
    dma_busy_ = false;
    have_sample_ = false;
    init_status_ = SENSOR_ERROR_BATTERY;
    voltage_scale_ = BATTERY_VOLTAGE_SCALE;
    current_scale_ = BATTERY_CURRENT_SCALE;
    current_offset_ = BATTERY_CURRENT_OFFSET;
    next_sample_time_ = 0;

    initialized_ = true;
    init_status_ = 0;
    return true;
  }

  void poll()
  {
    if (!initialized_ || dma_busy_) { return; }

    const uint64_t now = pixhawk_clock_micros_raw();
    if (next_sample_time_ != 0U && now < next_sample_time_) { return; }
    next_sample_time_ = now + BATTERY_SAMPLE_PERIOD_US;

    if (HAL_ADC_Start_DMA(&hadc1, battery_adc_buf, BATTERY_ADC_CHANNELS) == HAL_OK) {
      dma_busy_ = true;
    }
  }

  void finish_dma()
  {
    if (!dma_busy_) { return; }
    dma_busy_ = false;

    const uint32_t raw_voltage = battery_adc_buf[0];
    const uint32_t raw_current = battery_adc_buf[1];
    const uint32_t raw_temp = battery_adc_buf[2];
    const uint32_t raw_vref = battery_adc_buf[3];

    if (raw_vref == 0U) { return; }

    const float vdda = (VREFINT_CAL_VREF / 1000.0f) * static_cast<float>(*VREFINT_CAL_ADDR)
      / static_cast<float>(raw_vref);
    const float battery_pin_voltage = static_cast<float>(raw_voltage) / 65535.0f * vdda;
    const float battery_pin_current = static_cast<float>(raw_current) / 65535.0f * vdda;
    const float raw_temperature = static_cast<float>(raw_temp);
    const float temperature_c =
      ((TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
       / (static_cast<float>(*TEMPSENSOR_CAL2_ADDR) - static_cast<float>(*TEMPSENSOR_CAL1_ADDR)))
      * (raw_temperature - static_cast<float>(*TEMPSENSOR_CAL1_ADDR))
      + TEMPSENSOR_CAL1_TEMP;
    const uint64_t now = pixhawk_clock_micros_raw();

    latest_sample_.header.timestamp = now;
    latest_sample_.header.complete = now;
    latest_sample_.header.status = 0;
    latest_sample_.voltage = battery_pin_voltage * voltage_scale_;
    latest_sample_.current = (battery_pin_current - current_offset_) * current_scale_;
    latest_sample_.temperature = temperature_c + 273.15f;
    have_sample_ = true;
  }

  void abort_dma()
  {
    (void) HAL_ADC_Stop_DMA(&hadc1);
    dma_busy_ = false;
  }

  bool read(rosflight_firmware::BatteryStruct * bat)
  {
    if (!initialized_ || bat == nullptr || !have_sample_) { return false; }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool have_sample = have_sample_;
    if (have_sample) {
      *bat = latest_sample_;
      have_sample_ = false;
    }
    if (!primask) { __enable_irq(); }

    return have_sample;
  }

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }

private:
  volatile bool initialized_ = false;
  volatile bool dma_busy_ = false;
  volatile bool have_sample_ = false;
  uint32_t init_status_ = SENSOR_ERROR_BATTERY;
  uint64_t next_sample_time_ = 0;
  float voltage_scale_ = BATTERY_VOLTAGE_SCALE;
  float current_scale_ = BATTERY_CURRENT_SCALE;
  float current_offset_ = BATTERY_CURRENT_OFFSET;
  rosflight_firmware::BatteryStruct latest_sample_ = {};
};

Icm42688 icm42688;
Ist8310 ist8310;
BatteryMonitor battery_monitor;
} // namespace

Pixhawk6CMiniBoard pixhawk_6c_mini_board;
rosflight_firmware::Board & board = pixhawk_6c_mini_board;

uint32_t Vcp::init(uint16_t sample_rate_hz)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Vcp");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  txTimeout_ = 0;
  txDtimeout_ = 250;

  txFifo_.init(VCP_TX_FIFO_BUFFERS, sizeof(SerialTxPacket), vcp_fifo_tx_buffer);
  rxFifo_.init(VCP_RX_FIFO_BUFFER_BYTES, vcp_fifo_rx_buffer);

  txIdle_ = true;
  retry_ = 0;

  return initializationStatus_;
}

uint16_t Vcp::writePacket(SerialTxPacket * p_new)
{
  return txFifo_.write(reinterpret_cast<uint8_t *>(p_new), sizeof(SerialTxPacket));
}

void Vcp::poll(void)
{
  if ((txFifo_.packetCountMax() > 0) && (pixhawk_clock_micros_raw() > txTimeout_)) { txStart(); }
}

void Vcp::txCdcCallback(void) { txStart(); }

void Vcp::txStart(void)
{
  txTimeout_ = pixhawk_clock_micros_raw() + txDtimeout_;

  static SerialTxPacket p;
  uint8_t status = !USBD_OK;

  if (retry_) {
    status = VCP_Transmit(reinterpret_cast<uint8_t *>(p.payload), p.payloadSize);
    if (status == USBD_OK) {
      retry_ = 0;
      txIdle_ = false;
    } else {
      retry_--;
      txIdle_ = true;
    }
  } else if (txFifo_.packetCount()) {
    if (txFifo_.read(reinterpret_cast<uint8_t *>(&p), sizeof(SerialTxPacket))) {
      status = VCP_Transmit(reinterpret_cast<uint8_t *>(p.payload), p.payloadSize);
      if (status == USBD_OK) {
        retry_ = 0;
        txIdle_ = false;
      } else {
        retry_ = 2;
        txIdle_ = true;
      }
    } else {
      retry_ = 0;
    }
  }
}

void Pixhawk6CMiniBoard::init_board()
{
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();
  pixhawk_clock_init();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  led0_off();
  led1_off();
  led0_on();
  HAL_Delay(50);
  led0_off();

  MX_USB_OTG_FS_PCD_Init();
  MX_USB_DEVICE_Init();
  vcp_.init(VCP_HZ);
}

void Pixhawk6CMiniBoard::board_reset(bool bootloader)
{
  (void) bootloader;
  HAL_NVIC_SystemReset();
}

void Pixhawk6CMiniBoard::sensors_init(void)
{
  Icm42688::Config imu_config = {};
  imu_config.hspi = &hspi1;
  imu_config.cs_port = IMU_ICM42688_CS_GPIO_Port;
  imu_config.cs_pin = IMU_ICM42688_CS_Pin;
  imu_config.dma_tx_buffer = icm42688_dma_tx_buf;
  imu_config.dma_rx_buffer = icm42688_dma_rx_buf;
  imu_config.dma_buffer_size = sizeof(icm42688_dma_tx_buf);
  imu_config.clock_micros = pixhawk_clock_micros_raw;
  imu_config.delay_ms = HAL_Delay;
  imu_config.rotation_sensor_to_body = IMU_TO_FMU_ROTATION;

  icm42688.init(imu_config);

  Ist8310::Config mag_config = {};
  mag_config.hi2c = &hi2c4;
  mag_config.i2c_address = IST8310_I2C_ADDRESS;
  mag_config.clock_micros = pixhawk_clock_micros_raw;
  mag_config.delay_ms = HAL_Delay;
  mag_config.rotation_sensor_to_body = MAG_TO_FMU_ROTATION;
  mag_config.sample_period_us = MAG_SAMPLE_PERIOD_US;
  mag_config.conversion_timeout_us = MAG_TIMEOUT_US;

  ist8310.init(mag_config);
  battery_monitor.init();
}
uint16_t Pixhawk6CMiniBoard::sensors_errors_count()
{
  uint16_t errors = 0;
  if (!icm42688.init_good()) { errors++; }
  if (!ist8310.init_good()) { errors++; }
  if (!battery_monitor.init_good()) { errors++; }
  return errors;
}
uint16_t Pixhawk6CMiniBoard::sensors_init_message_count() { return 3; }
bool Pixhawk6CMiniBoard::sensors_init_message_good(uint16_t i)
{
  if (i == 0) { return icm42688.init_good(); }
  if (i == 1) { return ist8310.init_good(); }
  if (i == 2) { return battery_monitor.init_good(); }
  return false;
}
uint16_t Pixhawk6CMiniBoard::sensors_init_message(char * message, uint16_t size, uint16_t i)
{
  if (message == nullptr || size == 0) { return 0; }
  if (i == 0) {
    if (icm42688.init_good()) {
      std::snprintf(message, size, "%s", "ICM42688: INIT OK");
    } else {
      std::snprintf(message, size, "ICM42688: INIT ERROR 0x%08lX ID 0x%02X",
                    static_cast<unsigned long>(icm42688.init_status()), icm42688.who_am_i());
    }
    return 1;
  }
  if (i == 1) {
    if (ist8310.init_good()) {
      std::snprintf(message, size, "%s", "IST8310: INIT OK");
    } else {
      std::snprintf(message, size, "IST8310: INIT ERROR 0x%08lX ID 0x%02X",
                    static_cast<unsigned long>(ist8310.init_status()), ist8310.who_am_i());
    }
    return 1;
  }
  if (i == 2) {
    if (battery_monitor.init_good()) {
      std::snprintf(message, size, "%s", "ADC1 BATTERY: INIT OK");
    } else {
      std::snprintf(message, size, "ADC1 BATTERY: INIT ERROR 0x%08lX",
                    static_cast<unsigned long>(battery_monitor.init_status()));
    }
    return 1;
  }
  return 0;
}

uint32_t Pixhawk6CMiniBoard::clock_millis()
{
  poll();
  return HAL_GetTick();
}
uint64_t Pixhawk6CMiniBoard::clock_micros()
{
  poll();
  return pixhawk_clock_micros_raw();
}
void Pixhawk6CMiniBoard::clock_delay(uint32_t milliseconds) { HAL_Delay(milliseconds); }

void Pixhawk6CMiniBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
  (void) baud_rate;
  (void) dev;
}

void Pixhawk6CMiniBoard::serial_write(const uint8_t * src, size_t len, uint8_t qos)
{
  SerialTxPacket p = {};
  const size_t payload_len = std::min(len, static_cast<size_t>(SERIAL_MAX_PAYLOAD_SIZE));

  p.header.timestamp = pixhawk_clock_micros_raw();
  p.payloadSize = payload_len;
  p.qos = qos;
  std::memcpy(p.payload, src, payload_len);

  const uint64_t timeout = pixhawk_clock_micros_raw() + VCP_QUEUE_TIMEOUT_US;
  while (!vcp_.writePacket(&p) && pixhawk_clock_micros_raw() < timeout) { poll(); }
  poll();
}

uint16_t Pixhawk6CMiniBoard::serial_bytes_available()
{
  poll();
  return vcp_.byteCount();
}

uint8_t Pixhawk6CMiniBoard::serial_read()
{
  uint8_t c = 0;
  vcp_.readByte(&c);
  poll();
  return c;
}

void Pixhawk6CMiniBoard::serial_flush() { poll(); }

bool Pixhawk6CMiniBoard::imu_read(rosflight_firmware::ImuStruct * imu)
{
  poll();
  return icm42688.read(imu);
}
bool Pixhawk6CMiniBoard::mag_read(rosflight_firmware::MagStruct * mag)
{
  poll();
  return ist8310.read(mag);
}
bool Pixhawk6CMiniBoard::baro_read(rosflight_firmware::PressureStruct * baro)
{
  (void) baro;
  return false;
}

bool Pixhawk6CMiniBoard::diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure)
{
  pace_board_loop(vcp_);
  const uint64_t timestamp = pixhawk_clock_micros_raw();
  static uint64_t next_sample_time = 0;

  if (timestamp < next_sample_time) {
    poll();
    return false;
  }
  next_sample_time = timestamp + DIFF_PRESSURE_PERIOD_US;

  diff_pressure->header.timestamp = timestamp;
  diff_pressure->header.complete = timestamp;
  diff_pressure->header.status = 0;
  diff_pressure->pressure = 50.0f + static_cast<float>(timestamp % 1000000ULL) / 100000.0f;
  diff_pressure->temperature = 300.0f;
  diff_pressure->ias = 0.0f;
  poll();
  return true;
}

bool Pixhawk6CMiniBoard::range_read(rosflight_firmware::RangeStruct * range)
{
  (void) range;
  return false;
}
bool Pixhawk6CMiniBoard::gnss_read(rosflight_firmware::GnssStruct * gnss)
{
  (void) gnss;
  return false;
}
bool Pixhawk6CMiniBoard::battery_read(rosflight_firmware::BatteryStruct * bat)
{
  battery_monitor.poll();
  return battery_monitor.read(bat);
}
void Pixhawk6CMiniBoard::battery_voltage_set_multiplier(double multiplier) { (void) multiplier; }
void Pixhawk6CMiniBoard::battery_current_set_multiplier(double multiplier) { (void) multiplier; }

void Pixhawk6CMiniBoard::rc_init(rc_type_t rc_type) { (void) rc_type; }
bool Pixhawk6CMiniBoard::rc_read(rosflight_firmware::RcStruct * rc)
{
  (void) rc;
  return false;
}

void Pixhawk6CMiniBoard::pwm_init(const float * rate, uint32_t channels)
{
  (void) rate;
  (void) channels;
}
void Pixhawk6CMiniBoard::pwm_disable() {}
void Pixhawk6CMiniBoard::pwm_write(float * value, uint32_t channels)
{
  (void) value;
  (void) channels;
}

void Pixhawk6CMiniBoard::memory_init()
{
  sd_ready_ = false;
  MX_CRC_Init();
  if (MX_SDMMC2_SD_Init() != HAL_OK) { return; }
  sd_ready_ = sd_wait_ready(SD_TIMEOUT_MS);
}

bool Pixhawk6CMiniBoard::memory_read(void * dest, size_t len)
{
  if (!sd_ready_ || dest == nullptr) { return false; }
  const uint32_t block_count = sd_blocks_for_len(len);
  if (block_count == 0U) { return true; }
  if (block_count > SD_MAXBLKS) { return false; }
  if (hcrc.InputDataFormat != CRC_INPUTDATA_FORMAT_BYTES) { return false; }
  if (!sd_wait_ready(SD_TIMEOUT_MS)) { return false; }

  if (HAL_SD_ReadBlocks(&hsd2, sd_rx_buf, 0, block_count, SD_TIMEOUT_MS) != HAL_OK) {
    sd_ready_ = false;
    return false;
  }
  if (!sd_wait_ready(SD_TIMEOUT_MS)) {
    sd_ready_ = false;
    return false;
  }

  const uint32_t crc_computed = HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t *>(sd_rx_buf), len);
  uint32_t crc_read = 0;
  std::memcpy(&crc_read, sd_rx_buf + len, SD_CRC_BYTES);
  if (crc_computed != crc_read) { return false; }

  std::memcpy(dest, sd_rx_buf, len);
  return true;
}

bool Pixhawk6CMiniBoard::memory_write(const void * src, size_t len)
{
  if (!sd_ready_ || src == nullptr) { return false; }
  const uint32_t block_count = sd_blocks_for_len(len);
  if (block_count == 0U) { return true; }
  if (block_count > SD_MAXBLKS) { return false; }
  if (hcrc.InputDataFormat != CRC_INPUTDATA_FORMAT_BYTES) { return false; }
  if (!sd_wait_ready(SD_TIMEOUT_MS)) { return false; }

  std::memset(sd_tx_buf, 0, SD_BUFF_SIZE);
  std::memcpy(sd_tx_buf, src, len);
  const uint32_t crc_computed = HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t *>(sd_tx_buf), len);
  std::memcpy(sd_tx_buf + len, &crc_computed, SD_CRC_BYTES);

  if (HAL_SD_WriteBlocks(&hsd2, sd_tx_buf, 0, block_count, SD_TIMEOUT_MS) != HAL_OK) {
    sd_ready_ = false;
    return false;
  }
  if (!sd_wait_ready(SD_TIMEOUT_MS)) {
    sd_ready_ = false;
    return false;
  }

  return true;
}

void Pixhawk6CMiniBoard::led0_on()
{
  HAL_GPIO_WritePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin, GPIO_PIN_RESET);
}
void Pixhawk6CMiniBoard::led0_off()
{
  HAL_GPIO_WritePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin, GPIO_PIN_SET);
}
void Pixhawk6CMiniBoard::led0_toggle() { HAL_GPIO_TogglePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin); }

void Pixhawk6CMiniBoard::led1_on()
{
  HAL_GPIO_WritePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin, GPIO_PIN_RESET);
}
void Pixhawk6CMiniBoard::led1_off()
{
  HAL_GPIO_WritePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin, GPIO_PIN_SET);
}
void Pixhawk6CMiniBoard::led1_toggle() { HAL_GPIO_TogglePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin); }

void Pixhawk6CMiniBoard::backup_memory_init() {}
bool Pixhawk6CMiniBoard::backup_memory_read(void * dest, size_t len)
{
  (void) dest;
  (void) len;
  return false;
}
void Pixhawk6CMiniBoard::backup_memory_write(const void * src, size_t len)
{
  (void) src;
  (void) len;
}
void Pixhawk6CMiniBoard::backup_memory_clear(size_t len) { (void) len; }

void Pixhawk6CMiniBoard::poll()
{
  vcp_.poll();
  ist8310.poll();
}

extern "C" void CDC_Receive_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  if (chan == 0) { pixhawk_6c_mini_board.vcp_.rxCdcCallback(buffer, size); }
}

extern "C" void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  (void) buffer;
  (void) size;
  if (chan == 0) { pixhawk_6c_mini_board.vcp_.txCdcCallback(); }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == IMU_ICM42688_DRDY_Pin) { icm42688.start_dma(); }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
  if (icm42688.is_my(hspi)) { icm42688.finish_dma(); }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi)
{
  if (icm42688.is_my(hspi)) { icm42688.abort_dma(); }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
  if (hadc == &hadc1) { battery_monitor.finish_dma(); }
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef * hadc)
{
  if (hadc == &hadc1) { battery_monitor.abort_dma(); }
}
