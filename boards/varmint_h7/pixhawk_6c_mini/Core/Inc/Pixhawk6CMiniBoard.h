#ifndef PIXHAWK_6C_MINI_BOARD_H
#define PIXHAWK_6C_MINI_BOARD_H

#include "Vcp.h"
#include "interface/board.h"

class Pixhawk6CMiniBoard : public rosflight_firmware::Board
{
public:
  void init_board() override;
  void board_reset(bool bootloader) override;

  void sensors_init(void) override;
  uint16_t sensors_errors_count() override;
  uint16_t sensors_init_message_count() override;
  bool sensors_init_message_good(uint16_t i) override;
  uint16_t sensors_init_message(char * message, uint16_t size, uint16_t i) override;

  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t * src, size_t len, uint8_t qos) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

  bool imu_read(rosflight_firmware::ImuStruct * imu) override;
  bool mag_read(rosflight_firmware::MagStruct * mag) override;
  bool baro_read(rosflight_firmware::PressureStruct * baro) override;
  bool diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure) override;
  bool range_read(rosflight_firmware::RangeStruct * range) override;
  bool gnss_read(rosflight_firmware::GnssStruct * gnss) override;
  bool battery_read(rosflight_firmware::BatteryStruct * bat) override;
  void battery_voltage_set_multiplier(double multiplier) override;
  void battery_current_set_multiplier(double multiplier) override;

  void rc_init(rc_type_t rc_type) override;
  bool rc_read(rosflight_firmware::RcStruct * rc) override;

  void pwm_init(const float * rate, uint32_t channels) override;
  void pwm_disable() override;
  void pwm_write(float * value, uint32_t channels) override;

  void memory_init() override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;
  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

  void backup_memory_init() override;
  bool backup_memory_read(void * dest, size_t len) override;
  void backup_memory_write(const void * src, size_t len) override;
  void backup_memory_clear(size_t len) override;

  Vcp vcp_;

private:
  void poll();
  bool sd_ready_ = false;
};

#endif
