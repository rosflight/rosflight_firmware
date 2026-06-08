#ifndef PIXHAWK_6C_MINI_DRIVERS_M9N_UBX_H
#define PIXHAWK_6C_MINI_DRIVERS_M9N_UBX_H

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class M9nUbx
{
public:
  struct Config
  {
    UART_HandleTypeDef * huart = nullptr;
    USART_TypeDef * uart_instance = nullptr;
    uint64_t (*clock_micros)() = nullptr;
    void (*delay_ms)(uint32_t milliseconds) = nullptr;
    uint32_t gnss_baud = 115200U;
    uint32_t gnss_rate_hz = 5U;
  };

  bool init(const Config & config);
  void poll();
  bool read_gnss(rosflight_firmware::GnssStruct * gnss);
  void handle_uart_rx_complete();
  void handle_uart_error();

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  uint8_t fix_type() const { return gnss_fix_type_; }
  uint16_t init_message(char * message, uint16_t size) const;
  uint8_t debug_pulse_count(uint64_t now_us) const;
  bool is_my(UART_HandleTypeDef * huart) const { return config_.huart == huart; }

private:
  enum class ParseState : uint8_t
  {
    SYNC1,
    SYNC2,
    CLASS,
    ID,
    LENGTH1,
    LENGTH2,
    PAYLOAD,
    CHECKSUM_A,
    CHECKSUM_B,
  };

  enum class AckState : uint8_t
  {
    IDLE,
    WAITING,
    GOT_ACK,
    GOT_NAK,
  };

  static constexpr size_t UBX_MAX_PAYLOAD_BYTES = 256U;
  static constexpr size_t UBX_NAV_PVT_LENGTH = 92U;
  static constexpr size_t RX_RING_BYTES = 1024U;

  bool init_uart();
  bool set_uart_baud(uint32_t baud);
  bool configure_gnss();
  bool passive_detect_baud(uint32_t baud, uint32_t milliseconds);
  bool configure_gnss_at_baud(uint32_t baud);
  void flush_input(uint32_t silence_ms);
  void service_uart_for(uint32_t milliseconds);
  bool wait_for_nav_pvt(uint32_t milliseconds, uint32_t previous_nav_pvt_count);
  bool wait_for_ack(uint8_t cls, uint8_t id, uint32_t milliseconds, bool required);
  bool start_irq_rx();
  void poll_uart();
  void process_rx_byte(uint8_t byte);
  bool parse_byte(uint8_t byte);
  void parse_nmea_byte(uint8_t byte);
  void consume_nmea_line();
  void consume_frame();
  void consume_ack(bool ack);
  void reset_parser();
  void checksum_update(uint8_t byte);
  bool send_ubx(uint8_t cls, uint8_t id, const uint8_t * payload, uint16_t length);
  bool append_cfg_valset_u8(uint8_t * payload, uint16_t * length, uint32_t key, uint8_t value);
  bool append_cfg_valset_u16(uint8_t * payload, uint16_t * length, uint32_t key, uint16_t value);
  bool append_cfg_valset_u32(uint8_t * payload, uint16_t * length, uint32_t key, uint32_t value);
  bool send_cfg_valset_uart_protocol();
  bool send_cfg_valset_uart_baud(uint32_t baud);
  bool send_cfg_valset_nav_pvt_rate();
  bool send_cfg_prt(uint32_t baud);
  bool send_cfg_rate(uint32_t rate_hz);
  bool send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
  bool send_nav_pvt_poll();
  bool decode_nav_pvt();
  bool decode_nmea_gga();
  bool decode_nmea_rmc();
  void publish_nmea_alive_sample();
  bool nmea_field(uint8_t index, const char ** begin, const char ** end) const;
  static uint8_t parse_uint8_field(const char * begin, const char * end);
  static double parse_nmea_degrees(const char * begin, const char * end, char hemisphere);

  uint64_t clock_micros() const;
  void delay_ms(uint32_t milliseconds) const;

  Config config_ = {};
  bool initialized_ = false;
  uint32_t init_status_ = 0;

  ParseState parse_state_ = ParseState::SYNC1;
  uint8_t frame_class_ = 0;
  uint8_t frame_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_index_ = 0;
  uint8_t checksum_a_ = 0;
  uint8_t checksum_b_ = 0;
  uint8_t payload_[UBX_MAX_PAYLOAD_BYTES] = {};
  char nmea_line_[96] = {};
  uint8_t nmea_line_length_ = 0;
  uint8_t irq_rx_byte_ = 0;
  volatile bool irq_rx_active_ = false;
  volatile uint16_t rx_ring_head_ = 0;
  volatile uint16_t rx_ring_tail_ = 0;
  uint8_t rx_ring_[RX_RING_BYTES] = {};
  bool gnss_alive_ = false;
  volatile bool gnss_new_sample_ = false;
  rosflight_firmware::GnssStruct latest_gnss_ = {};
  uint32_t gnss_byte_count_ = 0;
  uint32_t gnss_frame_count_ = 0;
  uint32_t gnss_nav_pvt_count_ = 0;
  uint32_t gnss_nmea_start_count_ = 0;
  uint32_t gnss_nmea_sentence_count_ = 0;
  uint32_t gnss_nmea_gga_count_ = 0;
  uint32_t gnss_nmea_rmc_count_ = 0;
  uint32_t gnss_sample_count_ = 0;
  uint32_t gnss_rx_ring_overflow_count_ = 0;
  uint32_t gnss_ubx_sync_count_ = 0;
  uint32_t gnss_checksum_error_count_ = 0;
  uint32_t gnss_uart_error_count_ = 0;
  uint32_t gnss_last_uart_error_ = 0;
  uint32_t gnss_current_baud_ = 0;
  uint8_t gnss_last_byte_ = 0;
  uint8_t gnss_last_frame_class_ = 0;
  uint8_t gnss_last_frame_id_ = 0;
  uint64_t gnss_last_byte_us_ = 0;
  uint64_t gnss_last_nav_pvt_us_ = 0;
  uint64_t gnss_last_nmea_sentence_us_ = 0;
  uint64_t gnss_last_nav_pvt_poll_us_ = 0;
  uint8_t gnss_fix_type_ = 0;
  AckState ack_state_ = AckState::IDLE;
  uint8_t ack_wait_class_ = 0;
  uint8_t ack_wait_id_ = 0;
  uint32_t gnss_ack_count_ = 0;
  uint32_t gnss_nak_count_ = 0;
  uint32_t gnss_ack_timeout_count_ = 0;
  uint8_t gnss_last_ack_class_ = 0;
  uint8_t gnss_last_ack_id_ = 0;
  uint8_t gnss_last_nak_class_ = 0;
  uint8_t gnss_last_nak_id_ = 0;
};

#endif // PIXHAWK_6C_MINI_DRIVERS_M9N_UBX_H
