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

DTCM_RAM uint8_t vcp_fifo_tx_buffer[VCP_TX_FIFO_BUFFERS * sizeof(SerialTxPacket)];
uint8_t vcp_fifo_rx_buffer[VCP_RX_FIFO_BUFFER_BYTES];
SD_DMA_RAM uint8_t sd_rx_buf[SD_BUFF_SIZE];
SD_DMA_RAM uint8_t sd_tx_buf[SD_BUFF_SIZE];

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

void Pixhawk6CMiniBoard::sensors_init(void) {}
uint16_t Pixhawk6CMiniBoard::sensors_errors_count() { return 0; }
uint16_t Pixhawk6CMiniBoard::sensors_init_message_count() { return 0; }
bool Pixhawk6CMiniBoard::sensors_init_message_good(uint16_t i)
{
  (void) i;
  return false;
}
uint16_t Pixhawk6CMiniBoard::sensors_init_message(char * message, uint16_t size, uint16_t i)
{
  (void) message;
  (void) size;
  (void) i;
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
  (void) imu;
  return false;
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
