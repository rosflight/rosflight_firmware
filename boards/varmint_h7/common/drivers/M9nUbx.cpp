#include "M9nUbx.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Pixhawk 6C Mini-local UBX bring-up mirrors PX4's
// ../PX4-Autopilot/src/drivers/gps/devices/src/ubx.cpp behavior for baud probing,
// ACK/NAK-driven configuration, UBX stream parsing, and NAV-PVT decode.
namespace
{
constexpr uint32_t STATUS_GNSS_UART_ERROR = 0x00000001U;
constexpr uint32_t STATUS_GNSS_ACK_TIMEOUT = 0x00000002U;
constexpr uint32_t STATUS_GNSS_NAK = 0x00000004U;
constexpr uint32_t STATUS_GNSS_NO_NAV_PVT = 0x00000008U;
constexpr uint64_t NAV_PVT_POLL_PERIOD_US = 200000U;
constexpr uint64_t NAV_PVT_STREAM_STALE_US = 300000U;
constexpr uint64_t NMEA_ALIVE_SAMPLE_PERIOD_US = 200000U;
constexpr uint64_t DEBUG_GNSS_STALE_US = 2000000U;
constexpr uint32_t PASSIVE_BAUD_DETECT_MS = 1500U;

constexpr uint8_t UBX_SYNC1 = 0xB5U;
constexpr uint8_t UBX_SYNC2 = 0x62U;
constexpr uint8_t UBX_CLASS_NAV = 0x01U;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07U;
constexpr uint8_t UBX_CLASS_ACK = 0x05U;
constexpr uint8_t UBX_ID_ACK_NAK = 0x00U;
constexpr uint8_t UBX_ID_ACK_ACK = 0x01U;
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

bool M9nUbx::init(const Config & config)
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
  gnss_nmea_start_count_ = 0;
  gnss_nmea_sentence_count_ = 0;
  gnss_nmea_gga_count_ = 0;
  gnss_nmea_rmc_count_ = 0;
  gnss_sample_count_ = 0;
  gnss_rx_ring_overflow_count_ = 0;
  gnss_ubx_sync_count_ = 0;
  gnss_checksum_error_count_ = 0;
  gnss_uart_error_count_ = 0;
  gnss_last_uart_error_ = 0;
  gnss_current_baud_ = 0;
  gnss_last_byte_ = 0;
  gnss_last_frame_class_ = 0;
  gnss_last_frame_id_ = 0;
  gnss_last_byte_us_ = 0;
  gnss_last_nav_pvt_us_ = 0;
  gnss_last_nmea_sentence_us_ = 0;
  gnss_last_nav_pvt_poll_us_ = 0;
  gnss_fix_type_ = 0;
  ack_state_ = AckState::IDLE;
  ack_wait_class_ = 0;
  ack_wait_id_ = 0;
  nmea_line_length_ = 0;
  irq_rx_byte_ = 0;
  irq_rx_active_ = false;
  rx_ring_head_ = 0;
  rx_ring_tail_ = 0;
  gnss_ack_count_ = 0;
  gnss_nak_count_ = 0;
  gnss_ack_timeout_count_ = 0;
  gnss_last_ack_class_ = 0;
  gnss_last_ack_id_ = 0;
  gnss_last_nak_class_ = 0;
  gnss_last_nak_id_ = 0;
  reset_parser();

  if (config_.huart == nullptr || config_.uart_instance == nullptr || config_.clock_micros == nullptr
      || config_.delay_ms == nullptr) {
    init_status_ = STATUS_GNSS_UART_ERROR;
    return false;
  }

  if (!init_uart()) {
    init_status_ |= STATUS_GNSS_UART_ERROR;
  } else if (!configure_gnss() && gnss_byte_count_ == 0U) {
    init_status_ |= STATUS_GNSS_UART_ERROR;
  }

  initialized_ = (init_status_ == 0U) || (gnss_frame_count_ != 0U) || (gnss_nav_pvt_count_ != 0U)
    || (gnss_nmea_start_count_ != 0U) || (gnss_ubx_sync_count_ != 0U);
  if (initialized_) { (void) start_irq_rx(); }
  return initialized_;
}

void M9nUbx::poll()
{
  if (config_.huart == nullptr) { return; }

  poll_uart();

  const uint64_t now = clock_micros();
  const bool nav_pvt_stale = gnss_last_nav_pvt_us_ == 0U
    || (now - gnss_last_nav_pvt_us_) > NAV_PVT_STREAM_STALE_US;
  const bool poll_due = gnss_last_nav_pvt_poll_us_ == 0U
    || (now - gnss_last_nav_pvt_poll_us_) > NAV_PVT_POLL_PERIOD_US;

  if (initialized_ && gnss_frame_count_ != 0U && gnss_nmea_sentence_count_ == 0U && nav_pvt_stale && poll_due) {
    if (send_nav_pvt_poll()) { gnss_last_nav_pvt_poll_us_ = now; }
  }
}

bool M9nUbx::read_gnss(rosflight_firmware::GnssStruct * gnss)
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

uint16_t M9nUbx::init_message(char * message, uint16_t size) const
{
  if (message == nullptr || size == 0U) { return 0; }

  if (init_good()) {
    std::snprintf(message, size, "%s", "M9N UBX GPS1: INIT OK");
  } else {
    std::snprintf(message, size, "M9N UBX GPS1: INIT ERROR 0x%08lX",
                  static_cast<unsigned long>(init_status_));
  }

  return 1;
}

uint8_t M9nUbx::debug_pulse_count(uint64_t now_us) const
{
  const bool stale_bytes = gnss_last_byte_us_ == 0U || (now_us - gnss_last_byte_us_) > DEBUG_GNSS_STALE_US;
  if (stale_bytes) { return 1U; }

  if (gnss_frame_count_ == 0U && gnss_nmea_sentence_count_ == 0U) { return 4U; }

  const bool stale_nav_pvt = gnss_last_nav_pvt_us_ == 0U
    || (now_us - gnss_last_nav_pvt_us_) > DEBUG_GNSS_STALE_US;
  if (stale_nav_pvt) { return 2U; }

  if (gnss_fix_type_ < 3U) { return 3U; }

  return 0U;
}

void M9nUbx::handle_uart_rx_complete()
{
  const uint16_t next_head = static_cast<uint16_t>((rx_ring_head_ + 1U) % RX_RING_BYTES);
  if (next_head == rx_ring_tail_) {
    gnss_rx_ring_overflow_count_++;
  } else {
    rx_ring_[rx_ring_head_] = irq_rx_byte_;
    rx_ring_head_ = next_head;
  }

  (void) start_irq_rx();
}

void M9nUbx::handle_uart_error()
{
  if (config_.huart == nullptr) { return; }

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

  (void) start_irq_rx();
}

bool M9nUbx::init_uart()
{
  return set_uart_baud(config_.gnss_baud);
}

bool M9nUbx::set_uart_baud(uint32_t baud)
{
  if (config_.huart != nullptr) {
    (void) HAL_UART_AbortReceive(config_.huart);
    irq_rx_active_ = false;
    rx_ring_head_ = 0;
    rx_ring_tail_ = 0;
  }

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

bool M9nUbx::configure_gnss()
{
  static constexpr uint32_t baud_rates_to_try[] = {
    38400U, 57600U, 9600U, 115200U, 230400U, 460800U, 921600U,
  };

  if (passive_detect_baud(config_.gnss_baud, PASSIVE_BAUD_DETECT_MS)
      && configure_gnss_at_baud(config_.gnss_baud)) {
    return true;
  }

  for (uint32_t baud : baud_rates_to_try) {
    if (baud != config_.gnss_baud && passive_detect_baud(baud, PASSIVE_BAUD_DETECT_MS)
        && configure_gnss_at_baud(baud)) {
      return true;
    }
  }

  uint32_t live_baud = 0;
  for (uint32_t baud : baud_rates_to_try) {
    const uint32_t previous_frame_count = gnss_frame_count_;
    const uint32_t previous_nmea_start_count = gnss_nmea_start_count_;
    const uint32_t previous_ubx_sync_count = gnss_ubx_sync_count_;
    if (configure_gnss_at_baud(baud)) { return true; }
    if (gnss_current_baud_ == baud
        && (gnss_frame_count_ != previous_frame_count
            || gnss_nmea_start_count_ != previous_nmea_start_count
            || gnss_ubx_sync_count_ != previous_ubx_sync_count)) {
      live_baud = baud;
    }
  }

  if (gnss_nak_count_ != 0U) { init_status_ |= STATUS_GNSS_NAK; }
  if (gnss_ack_timeout_count_ != 0U) { init_status_ |= STATUS_GNSS_ACK_TIMEOUT; }
  if (gnss_byte_count_ != 0U && gnss_nav_pvt_count_ == 0U) { init_status_ |= STATUS_GNSS_NO_NAV_PVT; }
  (void) set_uart_baud((live_baud != 0U) ? live_baud : config_.gnss_baud);
  flush_input(20U);
  if (gnss_byte_count_ != 0U) { return true; }
  return false;
}

bool M9nUbx::passive_detect_baud(uint32_t baud, uint32_t milliseconds)
{
  if (!set_uart_baud(baud)) { return false; }

  reset_parser();
  nmea_line_length_ = 0;

  const uint32_t previous_frame_count = gnss_frame_count_;
  const uint32_t previous_nav_pvt_count = gnss_nav_pvt_count_;
  const uint32_t previous_nmea_sentence_count = gnss_nmea_sentence_count_;
  service_uart_for(milliseconds);

  const bool saw_ubx = gnss_frame_count_ != previous_frame_count || gnss_nav_pvt_count_ != previous_nav_pvt_count;
  const bool saw_nmea = gnss_nmea_sentence_count_ != previous_nmea_sentence_count;
  return saw_ubx || saw_nmea;
}

bool M9nUbx::configure_gnss_at_baud(uint32_t baud)
{
  if (!set_uart_baud(baud)) { return false; }
  flush_input(20U);

  bool cfg_valset_success = false;
  if (send_cfg_valset_uart_protocol()
      && wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 2000U, true)) {
    cfg_valset_success = true;

    if (!send_cfg_valset_uart_baud(config_.gnss_baud)) { return false; }
    (void) wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 100U, false);
  } else {
    if (!send_cfg_prt(baud)) { return false; }
    if (!wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_PRT, 1000U, false)) { return false; }

    if (!send_cfg_prt(config_.gnss_baud)) { return false; }
    (void) wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_PRT, 100U, false);
  }

  if (!set_uart_baud(config_.gnss_baud)) { return false; }
  flush_input(20U);

  const uint32_t previous_nav_pvt_count = gnss_nav_pvt_count_;
  bool configured_output = false;

  if (cfg_valset_success) {
    configured_output = send_cfg_valset_nav_pvt_rate()
                        && wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, 1000U, true);
  }

  if (!configured_output) {
    if (!send_cfg_rate(config_.gnss_rate_hz)
        || !wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_RATE, 1000U, false)) {
      return false;
    }

    if (!send_cfg_msg(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1U)
        || !wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_MSG, 1000U, false)) {
      return false;
    }
  }

  if (!wait_for_nav_pvt(2000U, previous_nav_pvt_count)) { init_status_ |= STATUS_GNSS_NO_NAV_PVT; }

  return true;
}

void M9nUbx::flush_input(uint32_t silence_ms)
{
  reset_parser();
  service_uart_for(silence_ms);
  reset_parser();
}

void M9nUbx::service_uart_for(uint32_t milliseconds)
{
  const uint64_t end = clock_micros() + static_cast<uint64_t>(milliseconds) * 1000ULL;
  while (clock_micros() < end) { poll_uart(); }
}

bool M9nUbx::wait_for_nav_pvt(uint32_t milliseconds, uint32_t previous_nav_pvt_count)
{
  const uint64_t end = clock_micros() + static_cast<uint64_t>(milliseconds) * 1000ULL;
  while (clock_micros() < end) {
    poll_uart();
    if (gnss_nav_pvt_count_ != previous_nav_pvt_count) { return true; }
  }
  return false;
}

bool M9nUbx::wait_for_ack(uint8_t cls, uint8_t id, uint32_t milliseconds, bool required)
{
  (void) required;
  ack_state_ = AckState::WAITING;
  ack_wait_class_ = cls;
  ack_wait_id_ = id;

  const uint64_t end = clock_micros() + static_cast<uint64_t>(milliseconds) * 1000ULL;
  while (clock_micros() < end && ack_state_ == AckState::WAITING) { poll_uart(); }

  const AckState result = ack_state_;
  ack_state_ = AckState::IDLE;

  if (result == AckState::GOT_ACK) { return true; }

  if (result == AckState::GOT_NAK) {
    return false;
  }

  gnss_ack_timeout_count_++;
  return false;
}

bool M9nUbx::start_irq_rx()
{
  if (config_.huart == nullptr) { return false; }
  if (HAL_UART_Receive_IT(config_.huart, &irq_rx_byte_, 1U) == HAL_OK) {
    irq_rx_active_ = true;
    return true;
  }
  irq_rx_active_ = false;
  return false;
}

void M9nUbx::poll_uart()
{
  while (rx_ring_tail_ != rx_ring_head_) {
    const uint8_t byte = rx_ring_[rx_ring_tail_];
    rx_ring_tail_ = static_cast<uint16_t>((rx_ring_tail_ + 1U) % RX_RING_BYTES);
    process_rx_byte(byte);
  }

  if (irq_rx_active_) { return; }

  uint8_t byte = 0;
  while (HAL_UART_Receive(config_.huart, &byte, 1U, 0U) == HAL_OK) {
    process_rx_byte(byte);
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

void M9nUbx::process_rx_byte(uint8_t byte)
{
  gnss_byte_count_++;
  gnss_last_byte_ = byte;
  if (byte == '$') { gnss_nmea_start_count_++; }
  if (byte == UBX_SYNC1) { gnss_ubx_sync_count_++; }
  gnss_last_byte_us_ = clock_micros();
  parse_nmea_byte(byte);
  if (parse_byte(byte)) {
    consume_frame();
    reset_parser();
  }
}

bool M9nUbx::parse_byte(uint8_t byte)
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

void M9nUbx::parse_nmea_byte(uint8_t byte)
{
  if (byte == '$') {
    nmea_line_[0] = static_cast<char>(byte);
    nmea_line_length_ = 1U;
    return;
  }

  if (nmea_line_length_ == 0U) { return; }

  if (byte == '\n' || byte == '\r') {
    nmea_line_[nmea_line_length_] = '\0';
    consume_nmea_line();
    nmea_line_length_ = 0U;
    return;
  }

  if (nmea_line_length_ < (sizeof(nmea_line_) - 1U)) {
    nmea_line_[nmea_line_length_++] = static_cast<char>(byte);
  } else {
    nmea_line_length_ = 0U;
  }
}

void M9nUbx::consume_nmea_line()
{
  gnss_nmea_sentence_count_++;
  gnss_last_nmea_sentence_us_ = clock_micros();

  if (std::strncmp(nmea_line_, "$GPGGA,", 7) == 0 || std::strncmp(nmea_line_, "$GNGGA,", 7) == 0
      || std::strncmp(nmea_line_, "$GLGGA,", 7) == 0) {
    if (decode_nmea_gga()) { gnss_nmea_gga_count_++; }
  } else if (std::strncmp(nmea_line_, "$GPRMC,", 7) == 0 || std::strncmp(nmea_line_, "$GNRMC,", 7) == 0
             || std::strncmp(nmea_line_, "$GLRMC,", 7) == 0) {
    if (decode_nmea_rmc()) { gnss_nmea_rmc_count_++; }
  } else {
    publish_nmea_alive_sample();
  }
}

void M9nUbx::consume_frame()
{
  gnss_frame_count_++;
  gnss_last_frame_class_ = frame_class_;
  gnss_last_frame_id_ = frame_id_;
  if (frame_class_ == UBX_CLASS_NAV && frame_id_ == UBX_ID_NAV_PVT) {
    gnss_alive_ = decode_nav_pvt();
    if (gnss_alive_) {
      gnss_nav_pvt_count_++;
      gnss_last_nav_pvt_us_ = latest_gnss_.header.complete;
    }
  } else if (frame_class_ == UBX_CLASS_ACK && frame_id_ == UBX_ID_ACK_ACK) {
    consume_ack(true);
  } else if (frame_class_ == UBX_CLASS_ACK && frame_id_ == UBX_ID_ACK_NAK) {
    consume_ack(false);
  }
}

void M9nUbx::consume_ack(bool ack)
{
  if (payload_length_ < 2U) { return; }

  const uint8_t cls = payload_[0];
  const uint8_t id = payload_[1];

  if (ack) {
    gnss_ack_count_++;
    gnss_last_ack_class_ = cls;
    gnss_last_ack_id_ = id;
  } else {
    gnss_nak_count_++;
    gnss_last_nak_class_ = cls;
    gnss_last_nak_id_ = id;
  }

  if (ack_state_ == AckState::WAITING && ack_wait_class_ == cls && ack_wait_id_ == id) {
    ack_state_ = ack ? AckState::GOT_ACK : AckState::GOT_NAK;
  }
}

void M9nUbx::reset_parser()
{
  parse_state_ = ParseState::SYNC1;
  payload_index_ = 0;
  payload_length_ = 0;
  checksum_a_ = 0;
  checksum_b_ = 0;
}

void M9nUbx::checksum_update(uint8_t byte)
{
  checksum_a_ = static_cast<uint8_t>(checksum_a_ + byte);
  checksum_b_ = static_cast<uint8_t>(checksum_b_ + checksum_a_);
}

bool M9nUbx::send_ubx(uint8_t cls, uint8_t id, const uint8_t * payload, uint16_t length)
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

bool M9nUbx::append_cfg_valset_u8(uint8_t * payload, uint16_t * length, uint32_t key, uint8_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 5U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  payload[*length] = value;
  *length = static_cast<uint16_t>(*length + 1U);
  return true;
}

bool M9nUbx::append_cfg_valset_u16(uint8_t * payload, uint16_t * length, uint32_t key, uint16_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 6U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  write_u16(&payload[*length], value);
  *length = static_cast<uint16_t>(*length + 2U);
  return true;
}

bool M9nUbx::append_cfg_valset_u32(uint8_t * payload, uint16_t * length, uint32_t key, uint32_t value)
{
  if (payload == nullptr || length == nullptr || (*length + 8U) > UBX_MAX_PAYLOAD_BYTES) { return false; }
  write_u32(&payload[*length], key);
  *length = static_cast<uint16_t>(*length + 4U);
  write_u32(&payload[*length], value);
  *length = static_cast<uint16_t>(*length + 4U);
  return true;
}

bool M9nUbx::send_cfg_valset_uart_protocol()
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

bool M9nUbx::send_cfg_valset_uart_baud(uint32_t baud)
{
  uint8_t payload[UBX_MAX_PAYLOAD_BYTES] = {};
  uint16_t length = 0;
  init_cfg_valset(payload, &length);

  return append_cfg_valset_u32(payload, &length, UBX_CFG_KEY_CFG_UART1_BAUDRATE, baud)
         && send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, length);
}

bool M9nUbx::send_cfg_valset_nav_pvt_rate()
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

bool M9nUbx::send_cfg_prt(uint32_t baud)
{
  uint8_t payload[20] = {};
  payload[0] = 0x01U;
  payload[4] = 0xD0U;
  payload[5] = 0x08U;
  write_u32(&payload[8], baud);
  payload[12] = 0x01U;
  payload[14] = 0x01U;
  return send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_PRT, payload, sizeof(payload));
}

bool M9nUbx::send_cfg_rate(uint32_t rate_hz)
{
  rate_hz = std::max<uint32_t>(rate_hz, 1U);
  uint8_t payload[6] = {};
  write_u16(&payload[0], static_cast<uint16_t>(1000U / rate_hz));
  write_u16(&payload[2], 1U);
  write_u16(&payload[4], 1U);
  return send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_RATE, payload, sizeof(payload));
}

bool M9nUbx::send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  const uint8_t payload[3] = {msg_class, msg_id, rate};
  return send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload, sizeof(payload));
}

bool M9nUbx::send_nav_pvt_poll()
{
  return send_ubx(UBX_CLASS_NAV, UBX_ID_NAV_PVT, nullptr, 0U);
}

bool M9nUbx::decode_nav_pvt()
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
  gnss_sample_count_++;
  return true;
}

bool M9nUbx::decode_nmea_gga()
{
  rosflight_firmware::GnssStruct sample = latest_gnss_;
  const uint64_t now = clock_micros();
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.pps = 0;

  const char * lat_begin = nullptr;
  const char * lat_end = nullptr;
  const char * ns_begin = nullptr;
  const char * ns_end = nullptr;
  const char * lon_begin = nullptr;
  const char * lon_end = nullptr;
  const char * ew_begin = nullptr;
  const char * ew_end = nullptr;
  const char * quality_begin = nullptr;
  const char * quality_end = nullptr;
  const char * sats_begin = nullptr;
  const char * sats_end = nullptr;
  const char * alt_begin = nullptr;
  const char * alt_end = nullptr;

  if (!nmea_field(2U, &lat_begin, &lat_end) || !nmea_field(3U, &ns_begin, &ns_end)
      || !nmea_field(4U, &lon_begin, &lon_end) || !nmea_field(5U, &ew_begin, &ew_end)
      || !nmea_field(6U, &quality_begin, &quality_end) || !nmea_field(7U, &sats_begin, &sats_end)
      || !nmea_field(9U, &alt_begin, &alt_end)) {
    return false;
  }

  const uint8_t quality = parse_uint8_field(quality_begin, quality_end);
  sample.fix_type = (quality == 0U) ? 0U : 3U;
  sample.num_sat = parse_uint8_field(sats_begin, sats_end);

  if (lat_begin != lat_end && ns_begin != ns_end && lon_begin != lon_end && ew_begin != ew_end) {
    sample.lat = parse_nmea_degrees(lat_begin, lat_end, *ns_begin);
    sample.lon = parse_nmea_degrees(lon_begin, lon_end, *ew_begin);
  }

  if (alt_begin != alt_end) { sample.height_msl = static_cast<float>(std::strtod(alt_begin, nullptr)); }

  gnss_fix_type_ = sample.fix_type;
  latest_gnss_ = sample;
  gnss_new_sample_ = true;
  gnss_sample_count_++;
  gnss_alive_ = true;
  return true;
}

bool M9nUbx::decode_nmea_rmc()
{
  rosflight_firmware::GnssStruct sample = latest_gnss_;
  const uint64_t now = clock_micros();
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.pps = 0;

  const char * status_begin = nullptr;
  const char * status_end = nullptr;
  const char * lat_begin = nullptr;
  const char * lat_end = nullptr;
  const char * ns_begin = nullptr;
  const char * ns_end = nullptr;
  const char * lon_begin = nullptr;
  const char * lon_end = nullptr;
  const char * ew_begin = nullptr;
  const char * ew_end = nullptr;
  const char * speed_begin = nullptr;
  const char * speed_end = nullptr;

  if (!nmea_field(2U, &status_begin, &status_end) || !nmea_field(3U, &lat_begin, &lat_end)
      || !nmea_field(4U, &ns_begin, &ns_end) || !nmea_field(5U, &lon_begin, &lon_end)
      || !nmea_field(6U, &ew_begin, &ew_end) || !nmea_field(7U, &speed_begin, &speed_end)) {
    return false;
  }

  sample.fix_type = (status_begin != status_end && *status_begin == 'A') ? 3U : 0U;

  if (lat_begin != lat_end && ns_begin != ns_end && lon_begin != lon_end && ew_begin != ew_end) {
    sample.lat = parse_nmea_degrees(lat_begin, lat_end, *ns_begin);
    sample.lon = parse_nmea_degrees(lon_begin, lon_end, *ew_begin);
  }

  if (speed_begin != speed_end) {
    static constexpr float KNOTS_TO_MPS = 0.514444f;
    sample.vel_n = static_cast<float>(std::strtod(speed_begin, nullptr)) * KNOTS_TO_MPS;
    sample.vel_e = 0.0f;
    sample.vel_d = 0.0f;
  }

  gnss_fix_type_ = sample.fix_type;
  latest_gnss_ = sample;
  gnss_new_sample_ = true;
  gnss_sample_count_++;
  gnss_alive_ = true;
  return true;
}

void M9nUbx::publish_nmea_alive_sample()
{
  const uint64_t now = clock_micros();
  if (latest_gnss_.header.complete != 0U
      && (now - latest_gnss_.header.complete) < NMEA_ALIVE_SAMPLE_PERIOD_US) {
    return;
  }

  rosflight_firmware::GnssStruct sample = latest_gnss_;
  sample.header.timestamp = now;
  sample.header.complete = now;
  sample.pps = 0;
  sample.fix_type = 0;
  gnss_fix_type_ = sample.fix_type;
  latest_gnss_ = sample;
  gnss_new_sample_ = true;
  gnss_sample_count_++;
  gnss_alive_ = true;
}

bool M9nUbx::nmea_field(uint8_t index, const char ** begin, const char ** end) const
{
  if (begin == nullptr || end == nullptr || nmea_line_length_ == 0U) { return false; }

  uint8_t current = 0;
  const char * field_begin = nmea_line_;
  const char * const line_end = nmea_line_ + nmea_line_length_;

  for (const char * p = nmea_line_; p <= line_end; p++) {
    if (p == line_end || *p == ',' || *p == '*') {
      if (current == index) {
        *begin = field_begin;
        *end = p;
        return true;
      }

      current++;
      field_begin = p + 1;
      if (p != line_end && *p == '*') { return false; }
    }
  }

  return false;
}

uint8_t M9nUbx::parse_uint8_field(const char * begin, const char * end)
{
  uint8_t value = 0;
  for (const char * p = begin; p != end; p++) {
    if (*p < '0' || *p > '9') { break; }
    value = static_cast<uint8_t>((value * 10U) + static_cast<uint8_t>(*p - '0'));
  }
  return value;
}

double M9nUbx::parse_nmea_degrees(const char * begin, const char * end, char hemisphere)
{
  if (begin == nullptr || begin == end) { return 0.0; }

  double raw = std::strtod(begin, nullptr);
  const int degrees = static_cast<int>(raw / 100.0);
  double value = static_cast<double>(degrees) + ((raw - static_cast<double>(degrees * 100)) / 60.0);

  if (hemisphere == 'S' || hemisphere == 'W') { value = -value; }
  return value;
}

uint64_t M9nUbx::clock_micros() const { return config_.clock_micros(); }

void M9nUbx::delay_ms(uint32_t milliseconds) const { config_.delay_ms(milliseconds); }
