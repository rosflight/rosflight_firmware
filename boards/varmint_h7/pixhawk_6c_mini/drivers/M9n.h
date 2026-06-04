#ifndef PIXHAWK_6C_MINI_DRIVERS_M9N_H
#define PIXHAWK_6C_MINI_DRIVERS_M9N_H

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class M9n
{
public:
  struct Config
  {
    UART_HandleTypeDef * huart = nullptr;
    USART_TypeDef * uart_instance = nullptr;
    I2C_HandleTypeDef * hi2c = nullptr;
    uint64_t (*clock_micros)() = nullptr;
    void (*delay_ms)(uint32_t milliseconds) = nullptr;
    const float * mag_rotation_sensor_to_body = nullptr;
    uint32_t gnss_baud = 115200U;
    uint32_t gnss_fallback_baud = 38400U;
    uint32_t gnss_rate_hz = 5U;
    uint32_t mag_sample_period_us = 10000U;
  };

  bool init(const Config & config);
  void poll();
  bool read_gnss(rosflight_firmware::GnssStruct * gnss);
  bool read_mag(rosflight_firmware::MagStruct * mag);

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  bool gnss_alive() const { return gnss_alive_; }
  uint32_t gnss_byte_count() const { return gnss_byte_count_; }
  uint32_t gnss_frame_count() const { return gnss_frame_count_; }
  uint32_t gnss_nav_pvt_count() const { return gnss_nav_pvt_count_; }
  uint32_t gnss_checksum_error_count() const { return gnss_checksum_error_count_; }
  uint32_t gnss_uart_error_count() const { return gnss_uart_error_count_; }
  uint32_t gnss_last_uart_error() const { return gnss_last_uart_error_; }
  uint32_t gnss_current_baud() const { return gnss_current_baud_; }
  uint64_t gnss_last_byte_us() const { return gnss_last_byte_us_; }
  uint64_t gnss_last_nav_pvt_us() const { return gnss_last_nav_pvt_us_; }
  uint8_t gnss_fix_type() const { return gnss_fix_type_; }
  bool mag_alive() const { return mag_initialized_; }
  uint8_t mag_address() const { return mag_address_; }
  uint8_t mag_who_am_i() const { return mag_who_am_i_; }
  uint32_t mag_status() const { return mag_status_; }
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

  static constexpr size_t UBX_MAX_PAYLOAD_BYTES = 256U;
  static constexpr size_t UBX_NAV_PVT_LENGTH = 92U;

  bool init_uart();
  bool set_uart_baud(uint32_t baud);
  bool configure_gnss();
  bool configure_gnss_at_baud(uint32_t baud);
  void service_uart_for(uint32_t milliseconds);
  bool wait_for_nav_pvt(uint32_t milliseconds, uint32_t previous_nav_pvt_count);
  void poll_uart();
  bool parse_byte(uint8_t byte);
  void consume_frame();
  void reset_parser();
  void checksum_update(uint8_t byte);
  bool send_ubx(uint8_t cls, uint8_t id, const uint8_t * payload, uint16_t length);
  bool append_cfg_valset_u8(uint8_t * payload, uint16_t * length, uint32_t key, uint8_t value);
  bool append_cfg_valset_u16(uint8_t * payload, uint16_t * length, uint32_t key, uint16_t value);
  bool append_cfg_valset_u32(uint8_t * payload, uint16_t * length, uint32_t key, uint32_t value);
  bool send_cfg_valset_uart_protocol();
  bool send_cfg_valset_uart_baud(uint32_t baud);
  bool send_cfg_valset_nav_pvt_rate();
  void send_cfg_prt(uint32_t baud);
  void send_cfg_rate(uint32_t rate_hz);
  void send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
  bool decode_nav_pvt();

  bool init_mag();
  bool mag_read_register(uint8_t address, uint8_t reg, uint8_t * value);
  bool mag_write_register(uint8_t address, uint8_t reg, uint8_t value);
  bool mag_read_registers(uint8_t address, uint8_t start_reg, uint8_t * dest, size_t len);
  bool mag_start_measurement();
  bool mag_read_sample();
  void rotate_mag(float * v) const;
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
  bool gnss_alive_ = false;
  volatile bool gnss_new_sample_ = false;
  rosflight_firmware::GnssStruct latest_gnss_ = {};
  uint32_t gnss_byte_count_ = 0;
  uint32_t gnss_frame_count_ = 0;
  uint32_t gnss_nav_pvt_count_ = 0;
  uint32_t gnss_checksum_error_count_ = 0;
  uint32_t gnss_uart_error_count_ = 0;
  uint32_t gnss_last_uart_error_ = 0;
  uint32_t gnss_current_baud_ = 0;
  uint64_t gnss_last_byte_us_ = 0;
  uint64_t gnss_last_nav_pvt_us_ = 0;
  uint8_t gnss_fix_type_ = 0;

  bool mag_initialized_ = false;
  bool mag_measurement_active_ = false;
  volatile bool mag_new_sample_ = false;
  uint32_t mag_status_ = 0;
  uint8_t mag_address_ = 0;
  uint8_t mag_who_am_i_ = 0;
  uint64_t mag_last_start_us_ = 0;
  uint64_t mag_next_sample_us_ = 0;
  rosflight_firmware::MagStruct latest_mag_ = {};
};

#endif // PIXHAWK_6C_MINI_DRIVERS_M9N_H
