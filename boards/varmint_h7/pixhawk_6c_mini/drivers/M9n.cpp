#include "M9n.h"

#include <algorithm>
#include <cstring>

namespace
{
constexpr uint32_t STATUS_GNSS_UART_ERROR = 0x00000001U;
constexpr uint32_t STATUS_MAG_NOT_FOUND = 0x00000002U;
constexpr uint32_t STATUS_MAG_CONFIG_ERROR = 0x00000004U;

constexpr uint8_t UBX_SYNC1 = 0xB5U;
constexpr uint8_t UBX_SYNC2 = 0x62U;
constexpr uint8_t UBX_CLASS_NAV = 0x01U;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07U;
constexpr uint8_t UBX_CLASS_CFG = 0x06U;
constexpr uint8_t UBX_ID_CFG_PRT = 0x00U;
constexpr uint8_t UBX_ID_CFG_MSG = 0x01U;
constexpr uint8_t UBX_ID_CFG_RATE = 0x08U;
constexpr uint8_t UBX_ID_CFG_VALSET = 0x8AU;

constexpr uint8_t UBX_CFG_LAYER_RAM = 0x01U;
constexpr uint16_t UBX_CFG_VALSET_MAX_PAYLOAD_BYTES = 256U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1_BAUDRATE = 0x40520001U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1_STOPBITS = 0x20520002U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1_DATABITS = 0x20520003U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1_PARITY = 0x20520004U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1INPROT_UBX = 0x10730001U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1INPROT_NMEA = 0x10730002U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1OUTPROT_UBX = 0x10740001U;
constexpr uint32_t UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA = 0x10740002U;
constexpr uint32_t UBX_CFG_KEY_RATE_MEAS = 0x30210001U;
constexpr uint32_t UBX_CFG_KEY_RATE_NAV = 0x30210002U;
constexpr uint32_t UBX_CFG_KEY_RATE_TIMEREF = 0x20210003U;
constexpr uint32_t UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007U;

constexpr uint8_t IST8308_WAI_REG = 0x00U;
constexpr uint8_t IST8308_WAI_VALUE = 0x08U;
constexpr uint8_t IST8308_STAT1_REG = 0x10U;
constexpr uint8_t IST8308_STAT1_DRDY = 0x01U;
constexpr uint8_t IST8308_DATA_REG = 0x11U;
constexpr uint8_t IST8308_CNTL2_REG = 0x31U;
constexpr uint8_t IST8308_CNTL2_SINGLE = 0x01U;
constexpr uint8_t IST8308_CNTL3_REG = 0x32U;
constexpr uint8_t IST8308_CNTL3_SRST = 0x01U;
constexpr uint8_t IST8308_CNTL4_REG = 0x34U;
constexpr uint8_t IST8308_RANGE_500UT = 0x00U;
constexpr uint8_t IST8308_OSRCNTL_REG = 0x41U;
constexpr uint8_t IST8308_OSR_16X = 0x24U;
constexpr uint32_t IST8308_CONVERSION_US = 6000U;
constexpr uint32_t IST8308_TIMEOUT_US = 20000U;
constexpr float UT_TO_T = 1.0e-6f;
constexpr float IST8308_COUNTS_PER_UT = 6.6f;

uint16_t read_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t read_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
    | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t read_i32(const uint8_t * p)
{
  return static_cast<int32_t>(read_u32(p));
}

int16_t read_i16(const uint8_t low, const uint8_t high)
{
  return static_cast<int16_t>(static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
}

void write_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
}

void write_u32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFFU);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFFU);
}

void init_cfg_valset(uint8_t * payload, uint16_t * length)
{
  std::memset(payload, 0, UBX_CFG_VALSET_MAX_PAYLOAD_BYTES);
  payload[1] = UBX_CFG_LAYER_RAM;
  *length = 4U;
}

bool leap_year(int year)
{
  return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
}

int64_t unix_seconds_utc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
  static constexpr uint16_t days_before_month[] = {
    0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U
  };

  int64_t days = 0;
  for (int y = 1970; y < static_cast<int>(year); y++) { days += leap_year(y) ? 366 : 365; }
  days += days_before_month[std::clamp<uint8_t>(month, 1U, 12U) - 1U];
  if (month > 2U && leap_year(year)) { days++; }
  days += static_cast<int64_t>(day) - 1;

  return (((days * 24) + hour) * 60 + min) * 60 + sec;
}
} // namespace

bool M9n::init(const Config & config)
{
  config_ = config;
  initialized_ = false;
  init_status_ = 0;
  gnss_alive_ = false;
  gnss_new_sample_ = false;
  latest_gnss_ = {};
  gnss_byte_count_ = 0;
  gnss_frame_count_ = 0;
  gnss_nav_pvt_count_ = 0;
  gnss_checksum_error_count_ = 0;
  gnss_uart_error_count_ = 0;
  gnss_last_uart_error_ = 0;
  gnss_current_baud_ = 0;
  gnss_last_byte_us_ = 0;
  gnss_last_nav_pvt_us_ = 0;
  gnss_fix_type_ = 0;
  mag_initialized_ = false;
  mag_new_sample_ = false;
  latest_mag_ = {};
  reset_parser();

  if (config_.huart == nullptr || config_.uart_instance == nullptr || config_.hi2c == nullptr
      || config_.clock_micros == nullptr || config_.delay_ms == nullptr) {
    init_status_ = STATUS_GNSS_UART_ERROR | STATUS_MAG_NOT_FOUND;
    return false;
  }

  if (!init_uart() || !configure_gnss()) { init_status_ |= STATUS_GNSS_UART_ERROR; }

  if (!init_mag()) { init_status_ |= STATUS_MAG_NOT_FOUND; }

  initialized_ = (init_status_ == 0U);
  return initialized_;
}

void M9n::poll()
{
  if (config_.huart != nullptr) { poll_uart(); }

  if (!mag_initialized_) { return; }

  const uint64_t now = clock_micros();
  if (!mag_measurement_active_) {
    if (now >= mag_next_sample_us_) { (void) mag_start_measurement(); }
    return;
  }

  if ((now - mag_last_start_us_) < IST8308_CONVERSION_US) { return; }

  if (!mag_read_sample() && (now - mag_last_start_us_) > IST8308_TIMEOUT_US) {
    mag_measurement_active_ = false;
    mag_status_ = STATUS_MAG_CONFIG_ERROR;
    mag_next_sample_us_ = now + config_.mag_sample_period_us;
  }
}

bool M9n::read_gnss(rosflight_firmware::GnssStruct * gnss)
{
  if (gnss == nullptr || !gnss_new_sample_) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = gnss_new_sample_;
  if (have_sample) {
    *gnss = latest_gnss_;
    gnss_new_sample_ = false;
  }
  if (!primask) { __enable_irq(); }
  return have_sample;
}

bool M9n::read_mag(rosflight_firmware::MagStruct * mag)
{
  if (mag == nullptr || !mag_new_sample_) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = mag_new_sample_;
  if (have_sample) {
    *mag = latest_mag_;
    mag_new_sample_ = false;
  }
  if (!primask) { __enable_irq(); }
  return have_sample;
}

bool M9n::init_uart()
{
  return set_uart_baud(config_.gnss_baud);
}

bool M9n::set_uart_baud(uint32_t baud)
{
  config_.huart->Instance = config_.uart_instance;
  config_.huart->Init.BaudRate = baud;
  config_.huart->Init.WordLength = UART_WORDLENGTH_8B;
  config_.huart->Init.StopBits = UART_STOPBITS_1;
  config_.huart->Init.Parity = UART_PARITY_NONE;
  config_.huart->Init.Mode = UART_MODE_TX_RX;
  config_.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  config_.huart->Init.OverSampling = UART_OVERSAMPLING_16;
  config_.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  config_.huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  config_.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(config_.huart) != HAL_OK) { return false; }
  (void) HAL_UARTEx_SetTxFifoThreshold(config_.huart, UART_TXFIFO_THRESHOLD_1_8);
  (void) HAL_UARTEx_SetRxFifoThreshold(config_.huart, UART_RXFIFO_THRESHOLD_1_8);
  (void) HAL_UARTEx_DisableFifoMode(config_.huart);
  gnss_current_baud_ = baud;
  return true;
}

bool M9n::configure_gnss()
{
  static constexpr uint32_t baud_rates_to_try[] = {
    38400U, 57600U, 9600U, 115200U, 230400U, 460800U, 921600U,
  };

  for (uint32_t baud : baud_rates_to_try) {
    if (configure_gnss_at_baud(baud)) { return true; }
  }

  (void) set_uart_baud(config_.gnss_baud);
  return false;
}

bool M9n::configure_gnss_at_baud(uint32_t baud)
{
  if (!set_uart_baud(baud)) { return false; }
  service_uart_for(50U);

  (void) send_cfg_valset_uart_protocol();
  service_uart_for(100U);
  (void) send_cfg_valset_uart_baud(config_.gnss_baud);
  send_cfg_prt(config_.gnss_baud);
  service_uart_for(100U);

  if (!set_uart_baud(config_.gnss_baud)) { return false; }
  service_uart_for(50U);

  const uint32_t previous_nav_pvt_count = gnss_nav_pvt_count_;
  (void) send_cfg_valset_uart_protocol();
  service_uart_for(50U);
  (void) send_cfg_valset_nav_pvt_rate();
  service_uart_for(50U);
  send_cfg_msg(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1U);
  send_cfg_msg(UBX_CLASS_NAV, 0x03U, 0U);
  send_cfg_msg(UBX_CLASS_NAV, 0x04U, 0U);
  send_cfg_msg(UBX_CLASS_NAV, 0x35U, 0U);
  send_cfg_rate(config_.gnss_rate_hz);

  return wait_for_nav_pvt(1000U, previous_nav_pvt_count);
}

void M9n::service_uart_for(uint32_t milliseconds)
{
  const uint64_t end = clock_micros() + static_cast<uint64_t>(milliseconds) * 1000ULL;
  while (clock_micros() < end) { poll_uart(); }
}

bool M9n::wait_for_nav_pvt(uint32_t milliseconds, uint32_t previous_nav_pvt_count)
{
  const uint64_t end = clock_micros() + static_cast<uint64_t>(milliseconds) * 1000ULL;
  while (clock_micros() < end) {
    poll_uart();
    if (gnss_nav_pvt_count_ != previous_nav_pvt_count) { return true; }
  }
  return false;
}

void M9n::poll_uart()
{
  uint8_t byte = 0;
  while (HAL_UART_Receive(config_.huart, &byte, 1U, 0U) == HAL_OK) {
    gnss_byte_count_++;
    gnss_last_byte_us_ = clock_micros();
    if (parse_byte(byte)) {
      consume_frame();
      reset_parser();
    }
  }

  const uint32_t error = HAL_UART_GetError(config_.huart);
  if (error != HAL_UART_ERROR_NONE) {
    gnss_uart_error_count_++;
    gnss_last_uart_error_ = error;
    __HAL_UART_CLEAR_PEFLAG(config_.huart);
    __HAL_UART_CLEAR_FEFLAG(config_.huart);
    __HAL_UART_CLEAR_NEFLAG(config_.huart);
    __HAL_UART_CLEAR_OREFLAG(config_.huart);
    config_.huart->ErrorCode = HAL_UART_ERROR_NONE;
  }
}

bool M9n::parse_byte(uint8_t byte)
{
  switch (parse_state_) {
    case ParseState::SYNC1:
      if (byte == UBX_SYNC1) { parse_state_ = ParseState::SYNC2; }
      break;
    case ParseState::SYNC2:
      if (byte == UBX_SYNC2) {
        checksum_a_ = 0;
        checksum_b_ = 0;
        parse_state_ = ParseState::CLASS;
      } else {
        parse_state_ = (byte == UBX_SYNC1) ? ParseState::SYNC2 : ParseState::SYNC1;
      }
      break;
    case ParseState::CLASS:
      frame_class_ = byte;
      checksum_update(byte);
      parse_state_ = ParseState::ID;
      break;
    case ParseState::ID:
      frame_id_ = byte;
      checksum_update(byte);
      parse_state_ = ParseState::LENGTH1;
      break;
    case ParseState::LENGTH1:
      payload_length_ = byte;
      checksum_update(byte);
      parse_state_ = ParseState::LENGTH2;
      break;
    case ParseState::LENGTH2:
      payload_length_ |= static_cast<uint16_t>(byte) << 8;
      checksum_update(byte);
      if (payload_length_ > UBX_MAX_PAYLOAD_BYTES) {
        reset_parser();
      } else {
        payload_index_ = 0;
        parse_state_ = (payload_length_ == 0U) ? ParseState::CHECKSUM_A : ParseState::PAYLOAD;
      }
      break;
    case ParseState::PAYLOAD:
      payload_[payload_index_++] = byte;
      checksum_update(byte);
      if (payload_index_ >= payload_length_) { parse_state_ = ParseState::CHECKSUM_A; }
      break;
    case ParseState::CHECKSUM_A:
      if (byte == checksum_a_) { parse_state_ = ParseState::CHECKSUM_B; }
      else {
        gnss_checksum_error_count_++;
        reset_parser();
      }
      break;
    case ParseState::CHECKSUM_B:
      if (byte == checksum_b_) { return true; }
      gnss_checksum_error_count_++;
      reset_parser();
      break;
  }
  return false;
}

void M9n::consume_frame()
{
  gnss_frame_count_++;
  if (frame_class_ == UBX_CLASS_NAV && frame_id_ == UBX_ID_NAV_PVT) {
    gnss_alive_ = decode_nav_pvt();
    if (gnss_alive_) {
      gnss_nav_pvt_count_++;
      gnss_last_nav_pvt_us_ = latest_gnss_.header.complete;
    }
  }
}

void M9n::reset_parser()
{
  parse_state_ = ParseState::SYNC1;
  payload_index_ = 0;
  payload_length_ = 0;
  checksum_a_ = 0;
  checksum_b_ = 0;
}

void M9n::checksum_update(uint8_t byte)
{
  checksum_a_ = static_cast<uint8_t>(checksum_a_ + byte);
  checksum_b_ = static_cast<uint8_t>(checksum_b_ + checksum_a_);
}

bool M9n::send_ubx(uint8_t cls, uint8_t id, const uint8_t * payload, uint16_t length)
{
  if (length > UBX_MAX_PAYLOAD_BYTES) { return false; }

  uint8_t message[UBX_MAX_PAYLOAD_BYTES + 8U] = {};
  message[0] = UBX_SYNC1;
  message[1] = UBX_SYNC2;
  message[2] = cls;
  message[3] = id;
  write_u16(&message[4], length);
  if (payload != nullptr && length > 0U) { std::memcpy(&message[6], payload, length); }

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (uint16_t i = 2; i < static_cast<uint16_t>(length + 6U); i++) {
    ck_a = static_cast<uint8_t>(ck_a + message[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  message[length + 6U] = ck_a;
  message[length + 7U] = ck_b;
  return HAL_UART_Transmit(config_.huart, message, static_cast<uint16_t>(length + 8U), 100U) == HAL_OK;
}

bool M9n::append_cfg_valset_u8(uint8_t * payload, uint16_t * length, uint32_t key, uint8_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 5U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  payload[*length] = value;
  *length = static_cast<uint16_t>(*length + 1U);
  return true;
}

bool M9n::append_cfg_valset_u16(uint8_t * payload, uint16_t * length, uint32_t key, uint16_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 6U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  write_u16(&payload[*length], value);
  *length = static_cast<uint16_t>(*length + 2U);
  return true;
}

bool M9n::append_cfg_valset_u32(uint8_t * payload, uint16_t * length, uint32_t key, uint32_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 8U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  write_u32(&payload[*length], value);
  *length = static_cast<uint16_t>(*length + 4U);
  return true;
}

bool M9n::send_cfg_valset_uart_protocol()
{
  uint8_t payload[UBX_MAX_PAYLOAD_BYTES] = {};
  uint16_t length = 0;
  init_cfg_valset(payload, &length);

  bool ok = true;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1_STOPBITS, 1U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1_DATABITS, 0U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1_PARITY, 0U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA, 0U) && ok;
  return ok && send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, length);
}

bool M9n::send_cfg_valset_uart_baud(uint32_t baud)
{
  uint8_t payload[UBX_MAX_PAYLOAD_BYTES] = {};
  uint16_t length = 0;
  init_cfg_valset(payload, &length);

  return append_cfg_valset_u32(payload, &length, UBX_CFG_KEY_CFG_UART1_BAUDRATE, baud)
         && send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, length);
}

bool M9n::send_cfg_valset_nav_pvt_rate()
{
  uint8_t payload[UBX_MAX_PAYLOAD_BYTES] = {};
  uint16_t length = 0;
  init_cfg_valset(payload, &length);

  const uint16_t measurement_ms = static_cast<uint16_t>(1000U / std::max<uint32_t>(config_.gnss_rate_hz, 1U));
  bool ok = true;
  ok = append_cfg_valset_u16(payload, &length, UBX_CFG_KEY_RATE_MEAS, measurement_ms) && ok;
  ok = append_cfg_valset_u16(payload, &length, UBX_CFG_KEY_RATE_NAV, 1U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_RATE_TIMEREF, 0U) && ok;
  ok = append_cfg_valset_u8(payload, &length, UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_UART1, 1U) && ok;
  return ok && send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, length);
}

void M9n::send_cfg_prt(uint32_t baud)
{
  uint8_t payload[20] = {};
  payload[0] = 0x01U;
  payload[4] = 0xD0U;
  payload[5] = 0x08U;
  write_u32(&payload[8], baud);
  payload[12] = 0x01U;
  payload[14] = 0x01U;
  (void) send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_PRT, payload, sizeof(payload));
}

void M9n::send_cfg_rate(uint32_t rate_hz)
{
  rate_hz = std::max<uint32_t>(rate_hz, 1U);
  uint8_t payload[6] = {};
  write_u16(&payload[0], static_cast<uint16_t>(1000U / rate_hz));
  write_u16(&payload[2], 1U);
  write_u16(&payload[4], 1U);
  (void) send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_RATE, payload, sizeof(payload));
}

void M9n::send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  const uint8_t payload[3] = {msg_class, msg_id, rate};
  (void) send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload, sizeof(payload));
}

bool M9n::decode_nav_pvt()
{
  if (payload_length_ < UBX_NAV_PVT_LENGTH) { return false; }

  rosflight_firmware::GnssStruct sample = {};
  const uint64_t now = clock_micros();
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.header.status = static_cast<uint16_t>(payload_[20]);
  sample.pps = 0;

  const uint8_t valid = payload_[11];
  if ((valid & 0x03U) == 0x03U) {
    sample.unix_seconds = unix_seconds_utc(read_u16(&payload_[4]), payload_[6], payload_[7],
                                           payload_[8], payload_[9], payload_[10]);
    sample.unix_nanos = read_i32(&payload_[16]);
    if (sample.unix_nanos < 0) {
      sample.unix_seconds--;
      sample.unix_nanos += 1000000000;
    }
  }

  sample.fix_type = payload_[20];
  gnss_fix_type_ = sample.fix_type;
  sample.num_sat = payload_[23];
  sample.lon = static_cast<double>(read_i32(&payload_[24])) * 1.0e-7;
  sample.lat = static_cast<double>(read_i32(&payload_[28])) * 1.0e-7;
  sample.height_msl = static_cast<float>(read_i32(&payload_[36])) * 1.0e-3f;
  sample.h_acc = static_cast<float>(read_u32(&payload_[40])) * 1.0e-3f;
  sample.v_acc = static_cast<float>(read_u32(&payload_[44])) * 1.0e-3f;
  sample.vel_n = static_cast<float>(read_i32(&payload_[48])) * 1.0e-3f;
  sample.vel_e = static_cast<float>(read_i32(&payload_[52])) * 1.0e-3f;
  sample.vel_d = static_cast<float>(read_i32(&payload_[56])) * 1.0e-3f;
  sample.speed_accy = static_cast<float>(read_u32(&payload_[68])) * 1.0e-3f;

  latest_gnss_ = sample;
  gnss_new_sample_ = true;
  return true;
}

bool M9n::init_mag()
{
  mag_status_ = STATUS_MAG_NOT_FOUND;
  mag_address_ = 0;
  mag_who_am_i_ = 0;

  for (uint8_t address = 0x0CU; address <= 0x0FU; address++) {
    uint8_t who = 0;
    if (!mag_read_register(address, IST8308_WAI_REG, &who)) { continue; }
    mag_who_am_i_ = who;
    if (who != IST8308_WAI_VALUE) { continue; }

    mag_address_ = address;
    if (!mag_write_register(address, IST8308_CNTL3_REG, IST8308_CNTL3_SRST)) {
      mag_status_ = STATUS_MAG_CONFIG_ERROR;
      return false;
    }
    delay_ms(20);
    if (!mag_write_register(address, IST8308_CNTL4_REG, IST8308_RANGE_500UT)
        || !mag_write_register(address, IST8308_OSRCNTL_REG, IST8308_OSR_16X)) {
      mag_status_ = STATUS_MAG_CONFIG_ERROR;
      return false;
    }

    mag_initialized_ = true;
    mag_measurement_active_ = false;
    mag_status_ = 0;
    mag_next_sample_us_ = clock_micros();
    return true;
  }

  return false;
}

bool M9n::mag_read_register(uint8_t address, uint8_t reg, uint8_t * value)
{
  return mag_read_registers(address, reg, value, 1U);
}

bool M9n::mag_write_register(uint8_t address, uint8_t reg, uint8_t value)
{
  const uint8_t tx[2] = {reg, value};
  return HAL_I2C_Master_Transmit(config_.hi2c, static_cast<uint16_t>(address << 1), const_cast<uint8_t *>(tx),
                                 sizeof(tx), 10U) == HAL_OK;
}

bool M9n::mag_read_registers(uint8_t address, uint8_t start_reg, uint8_t * dest, size_t len)
{
  if (dest == nullptr || len == 0U) { return false; }
  if (HAL_I2C_Master_Transmit(config_.hi2c, static_cast<uint16_t>(address << 1), &start_reg, 1U, 10U) != HAL_OK) {
    return false;
  }
  return HAL_I2C_Master_Receive(config_.hi2c, static_cast<uint16_t>(address << 1), dest,
                                static_cast<uint16_t>(len), 10U) == HAL_OK;
}

bool M9n::mag_start_measurement()
{
  if (!mag_write_register(mag_address_, IST8308_CNTL2_REG, IST8308_CNTL2_SINGLE)) {
    mag_status_ = STATUS_MAG_CONFIG_ERROR;
    return false;
  }

  mag_measurement_active_ = true;
  mag_last_start_us_ = clock_micros();
  return true;
}

bool M9n::mag_read_sample()
{
  uint8_t status = 0;
  if (!mag_read_register(mag_address_, IST8308_STAT1_REG, &status)) { return false; }
  if ((status & IST8308_STAT1_DRDY) == 0U) { return false; }

  uint8_t data[6] = {};
  if (!mag_read_registers(mag_address_, IST8308_DATA_REG, data, sizeof(data))) { return false; }

  rosflight_firmware::MagStruct sample = {};
  const uint64_t now = clock_micros();
  sample.header.timestamp = mag_last_start_us_;
  sample.header.complete = now;
  sample.header.status = status;

  const int16_t raw_x = read_i16(data[0], data[1]);
  const int16_t raw_y = read_i16(data[2], data[3]);
  const int16_t raw_z = read_i16(data[4], data[5]);
  sample.flux[0] = (static_cast<float>(raw_x) / IST8308_COUNTS_PER_UT) * UT_TO_T;
  sample.flux[1] = (static_cast<float>(raw_y) / IST8308_COUNTS_PER_UT) * UT_TO_T;
  sample.flux[2] = (static_cast<float>(raw_z) / IST8308_COUNTS_PER_UT) * UT_TO_T;
  sample.temperature = 0.0f;
  rotate_mag(sample.flux);

  latest_mag_ = sample;
  mag_new_sample_ = true;
  mag_measurement_active_ = false;
  mag_status_ = 0;
  mag_next_sample_us_ = mag_last_start_us_ + config_.mag_sample_period_us;
  return true;
}

void M9n::rotate_mag(float * v) const
{
  if (v == nullptr || config_.mag_rotation_sensor_to_body == nullptr) { return; }
  const float x = v[0];
  const float y = v[1];
  const float z = v[2];
  const float * r = config_.mag_rotation_sensor_to_body;
  v[0] = r[0] * x + r[1] * y + r[2] * z;
  v[1] = r[3] * x + r[4] * y + r[5] * z;
  v[2] = r[6] * x + r[7] * y + r[8] * z;
}

uint64_t M9n::clock_micros() const { return config_.clock_micros(); }

void M9n::delay_ms(uint32_t milliseconds) const { config_.delay_ms(milliseconds); }
