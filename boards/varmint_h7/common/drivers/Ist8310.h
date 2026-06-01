#ifndef IST8310_H_
#define IST8310_H_

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class Ist8310
{
public:
  struct Config
  {
    I2C_HandleTypeDef * hi2c = nullptr;
    uint8_t i2c_address = 0x0c;
    uint64_t (*clock_micros)() = nullptr;
    void (*delay_ms)(uint32_t milliseconds) = nullptr;
    const float * rotation_sensor_to_body = nullptr;
    uint32_t sample_period_us = 10000U;
    uint32_t conversion_timeout_us = 20000U;
  };

  bool init(const Config & config);
  void poll();
  bool read(rosflight_firmware::MagStruct * mag);

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  uint8_t who_am_i() const { return who_am_i_; }
  bool is_my(I2C_HandleTypeDef * hi2c) const { return config_.hi2c == hi2c; }

private:
  bool read_register(uint8_t reg, uint8_t * value);
  bool write_register(uint8_t reg, uint8_t value);
  bool read_registers(uint8_t start_reg, uint8_t * dest, size_t len);
  bool start_measurement();
  bool read_sample();
  void rotate_to_body(float * v) const;
  uint64_t clock_micros() const;
  void delay_ms(uint32_t milliseconds) const;

  Config config_ = {};
  volatile bool initialized_ = false;
  volatile bool new_sample_ = false;
  bool measurement_active_ = false;
  uint32_t init_status_ = 0;
  uint8_t who_am_i_ = 0;
  uint64_t last_start_time_us_ = 0;
  uint64_t next_sample_time_us_ = 0;
  uint64_t next_status_check_time_us_ = 0;
  rosflight_firmware::MagStruct latest_sample_ = {};
};

#endif // IST8310_H_
