#include "Pixhawk6CMiniBoard.h"

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
constexpr uint32_t ICM42688_SPI_TIMEOUT_MS = 10U;
constexpr uint8_t ICM42688_WHO_AM_I_VALUE = 0x47U;
constexpr uint8_t ICM42688_REG_DEVICE_CONFIG = 0x11U;
constexpr uint8_t ICM42688_REG_INT_CONFIG = 0x14U;
constexpr uint8_t ICM42688_REG_TEMP_DATA1 = 0x1DU;
constexpr uint8_t ICM42688_REG_SIGNAL_PATH_RESET = 0x4BU;
constexpr uint8_t ICM42688_REG_PWR_MGMT0 = 0x4EU;
constexpr uint8_t ICM42688_REG_GYRO_CONFIG0 = 0x4FU;
constexpr uint8_t ICM42688_REG_ACCEL_CONFIG0 = 0x50U;
constexpr uint8_t ICM42688_REG_INT_SOURCE0 = 0x65U;
constexpr uint8_t ICM42688_REG_WHO_AM_I = 0x75U;
constexpr uint8_t ICM42688_READ = 0x80U;
constexpr uint8_t ICM42688_WRITE = 0x00U;
constexpr uint8_t ICM42688_BURST_BYTES = 15U;
constexpr float GRAVITY_MSS = 9.80665f;
constexpr float ICM42688_ACCEL_SCALE = GRAVITY_MSS / 2048.0f; // +/-16g
constexpr float ICM42688_GYRO_SCALE = 0.01745329252f / 16.4f; // +/-2000 dps
constexpr uint32_t SENSOR_ERROR_IMU = 0x0001U;

constexpr float IMU_TO_FMU_ROTATION[9] = {
  0.0f, -1.0f, 0.0f,
  -1.0f, 0.0f, 0.0f,
  0.0f, 0.0f, -1.0f,
};

DTCM_RAM uint8_t vcp_fifo_tx_buffer[VCP_TX_FIFO_BUFFERS * sizeof(SerialTxPacket)];
uint8_t vcp_fifo_rx_buffer[VCP_RX_FIFO_BUFFER_BYTES];
SD_DMA_RAM uint8_t sd_rx_buf[SD_BUFF_SIZE];
SD_DMA_RAM uint8_t sd_tx_buf[SD_BUFF_SIZE];
DMA_RAM uint8_t icm42688_dma_tx_buf[ICM42688_BURST_BYTES];
DMA_RAM uint8_t icm42688_dma_rx_buf[ICM42688_BURST_BYTES];

int16_t read_i16_be(const uint8_t * bytes)
{
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

void rotate_imu_to_fmu(float * v)
{
  const float x = v[0];
  const float y = v[1];
  const float z = v[2];

  v[0] = IMU_TO_FMU_ROTATION[0] * x + IMU_TO_FMU_ROTATION[1] * y + IMU_TO_FMU_ROTATION[2] * z;
  v[1] = IMU_TO_FMU_ROTATION[3] * x + IMU_TO_FMU_ROTATION[4] * y + IMU_TO_FMU_ROTATION[5] * z;
  v[2] = IMU_TO_FMU_ROTATION[6] * x + IMU_TO_FMU_ROTATION[7] * y + IMU_TO_FMU_ROTATION[8] * z;
}

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

class Icm42688
{
public:
  bool init()
  {
    initialized_ = false;
    dma_busy_ = false;
    new_sample_ = false;
    init_status_ = SENSOR_ERROR_IMU;

    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(3);

    write_register(ICM42688_REG_DEVICE_CONFIG, 0x01U);
    HAL_Delay(2);

    const uint8_t who_am_i = read_register(ICM42688_REG_WHO_AM_I);
    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
      who_am_i_ = who_am_i;
      return false;
    }

    write_register(ICM42688_REG_PWR_MGMT0, 0x00U);
    HAL_Delay(1);
    write_register(ICM42688_REG_GYRO_CONFIG0, 0x06U);
    write_register(ICM42688_REG_ACCEL_CONFIG0, 0x06U);
    write_register(ICM42688_REG_SIGNAL_PATH_RESET, 0x0AU);
    HAL_Delay(1);
    write_register(ICM42688_REG_INT_CONFIG, 0x1BU);
    write_register(ICM42688_REG_INT_SOURCE0, 0x08U);
    write_register(ICM42688_REG_PWR_MGMT0, 0x0FU);
    HAL_Delay(50);

    initialized_ = true;
    init_status_ = 0;
    return true;
  }

  void start_dma()
  {
    if (!initialized_ || dma_busy_) { return; }

    std::memset(icm42688_dma_tx_buf, 0, ICM42688_BURST_BYTES);
    icm42688_dma_tx_buf[0] = ICM42688_REG_TEMP_DATA1 | ICM42688_READ;

    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, icm42688_dma_tx_buf, icm42688_dma_rx_buf,
                                    ICM42688_BURST_BYTES) == HAL_OK) {
      dma_busy_ = true;
    } else {
      HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_SET);
    }
  }

  void finish_dma()
  {
    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_SET);
    if (!dma_busy_) { return; }
    dma_busy_ = false;

    rosflight_firmware::ImuStruct sample = {};
    const uint64_t now = pixhawk_clock_micros_raw();
    sample.header.timestamp = now;
    sample.header.complete = now;
    sample.header.status = 0;

    const int16_t temp_raw = read_i16_be(&icm42688_dma_rx_buf[1]);
    const int16_t accel_x = read_i16_be(&icm42688_dma_rx_buf[3]);
    const int16_t accel_y = read_i16_be(&icm42688_dma_rx_buf[5]);
    const int16_t accel_z = read_i16_be(&icm42688_dma_rx_buf[7]);
    const int16_t gyro_x = read_i16_be(&icm42688_dma_rx_buf[9]);
    const int16_t gyro_y = read_i16_be(&icm42688_dma_rx_buf[11]);
    const int16_t gyro_z = read_i16_be(&icm42688_dma_rx_buf[13]);

    sample.temperature = (static_cast<float>(temp_raw) / 132.48f) + 25.0f + 273.15f;
    sample.accel[0] = static_cast<float>(accel_x) * ICM42688_ACCEL_SCALE;
    sample.accel[1] = static_cast<float>(accel_y) * ICM42688_ACCEL_SCALE;
    sample.accel[2] = static_cast<float>(accel_z) * ICM42688_ACCEL_SCALE;
    sample.gyro[0] = static_cast<float>(gyro_x) * ICM42688_GYRO_SCALE;
    sample.gyro[1] = static_cast<float>(gyro_y) * ICM42688_GYRO_SCALE;
    sample.gyro[2] = static_cast<float>(gyro_z) * ICM42688_GYRO_SCALE;
    rotate_imu_to_fmu(sample.accel);
    rotate_imu_to_fmu(sample.gyro);

    latest_sample_ = sample;
    new_sample_ = true;
  }

  void abort_dma()
  {
    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_SET);
    dma_busy_ = false;
  }

  bool read(rosflight_firmware::ImuStruct * imu)
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

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  uint8_t who_am_i() const { return who_am_i_; }

private:
  uint8_t transfer_byte(uint8_t reg, uint8_t value)
  {
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {};
    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), ICM42688_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(IMU_ICM42688_CS_GPIO_Port, IMU_ICM42688_CS_Pin, GPIO_PIN_SET);
    return (status == HAL_OK) ? rx[1] : 0U;
  }

  uint8_t read_register(uint8_t reg) { return transfer_byte(reg | ICM42688_READ, 0U); }
  void write_register(uint8_t reg, uint8_t value)
  {
    (void) transfer_byte(reg | ICM42688_WRITE, value);
    for (volatile uint32_t i = 0; i < 2000U; i++) {}
  }

  volatile bool initialized_ = false;
  volatile bool dma_busy_ = false;
  volatile bool new_sample_ = false;
  uint32_t init_status_ = SENSOR_ERROR_IMU;
  uint8_t who_am_i_ = 0;
  rosflight_firmware::ImuStruct latest_sample_ = {};
};

Icm42688 icm42688;
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

void Pixhawk6CMiniBoard::sensors_init(void) { icm42688.init(); }
uint16_t Pixhawk6CMiniBoard::sensors_errors_count() { return icm42688.init_good() ? 0 : 1; }
uint16_t Pixhawk6CMiniBoard::sensors_init_message_count() { return 1; }
bool Pixhawk6CMiniBoard::sensors_init_message_good(uint16_t i)
{
  return i == 0 && icm42688.init_good();
}
uint16_t Pixhawk6CMiniBoard::sensors_init_message(char * message, uint16_t size, uint16_t i)
{
  if (message == nullptr || size == 0 || i != 0) { return 0; }
  if (icm42688.init_good()) {
    std::snprintf(message, size, "%s", "ICM42688: INIT OK");
  } else {
    std::snprintf(message, size, "ICM42688: INIT ERROR 0x%08lX ID 0x%02X",
                  static_cast<unsigned long>(icm42688.init_status()), icm42688.who_am_i());
  }
  return 1;
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
  (void) mag;
  return false;
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
  (void) bat;
  return false;
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

void Pixhawk6CMiniBoard::poll() { vcp_.poll(); }

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
  if (hspi == &hspi1) { icm42688.finish_dma(); }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi)
{
  if (hspi == &hspi1) { icm42688.abort_dma(); }
}
