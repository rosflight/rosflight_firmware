#ifndef MS5611_H_
#define MS5611_H_

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstdint>

class Ms5611
{
public:
  enum class Osr : uint8_t
  {
    OSR_256 = 0,
    OSR_512,
    OSR_1024,
    OSR_2048,
    OSR_4096,
  };

  struct Config
  {
    I2C_HandleTypeDef * hi2c = nullptr;
    uint64_t (*clock_micros)() = nullptr;
    void (*delay_ms)(uint32_t milliseconds) = nullptr;
    Osr osr = Osr::OSR_4096;
    uint8_t temperature_decimation = 10;
  };

  bool init(const Config & config);
  void poll();
  bool read(rosflight_firmware::PressureStruct * baro);

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  uint8_t i2c_address() const { return i2c_address_; }
  uint16_t prom_word(uint8_t index) const { return index < 8U ? prom_[index] : 0U; }
  bool is_my(I2C_HandleTypeDef * hi2c) const { return config_.hi2c == hi2c; }

private:
  enum class State : uint8_t
  {
    IDLE,
    D1_CONVERTING,
    D2_CONVERTING,
  };

  bool probe_address(uint8_t address);
  bool reset();
  bool read_prom();
  bool prom_sane() const;
  bool prom_crc_ok() const;
  bool command(uint8_t cmd);
  bool read_prom_word(uint8_t index, uint16_t * value);
  bool read_adc(uint32_t * value);
  bool start_pressure_conversion();
  bool start_temperature_conversion();
  bool take_initial_sample();
  bool compute_sample();
  bool pressure_plausible(float pressure_pa, float temperature_c) const;
  uint8_t d1_command() const;
  uint8_t d2_command() const;
  uint32_t conversion_time_us() const;
  uint64_t clock_micros() const;
  void delay_ms(uint32_t milliseconds) const;

  Config config_ = {};
  volatile bool initialized_ = false;
  volatile bool new_sample_ = false;
  uint32_t init_status_ = 0;
  uint8_t i2c_address_ = 0;
  uint16_t prom_[8] = {};
  uint32_t d1_ = 0;
  uint32_t d2_ = 0;
  uint8_t pressure_count_ = 0;
  State state_ = State::IDLE;
  uint64_t conversion_deadline_us_ = 0;
  rosflight_firmware::PressureStruct latest_sample_ = {};
};

#endif // MS5611_H_
