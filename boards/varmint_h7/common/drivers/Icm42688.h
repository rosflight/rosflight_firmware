#ifndef ICM42688_H_
#define ICM42688_H_

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class Icm42688
{
public:
  static constexpr uint16_t BURST_BYTES = 15U;

  struct Config
  {
    SPI_HandleTypeDef * hspi = nullptr;
    GPIO_TypeDef * cs_port = nullptr;
    uint16_t cs_pin = 0;
    uint8_t * dma_tx_buffer = nullptr;
    uint8_t * dma_rx_buffer = nullptr;
    size_t dma_buffer_size = 0;
    uint64_t (*clock_micros)() = nullptr;
    void (*delay_ms)(uint32_t milliseconds) = nullptr;
    const float * rotation_sensor_to_body = nullptr;
  };

  bool init(const Config & config);
  void start_dma();
  void finish_dma();
  void abort_dma();
  bool read(rosflight_firmware::ImuStruct * imu);

  bool init_good() const { return initialized_; }
  uint32_t init_status() const { return init_status_; }
  uint8_t who_am_i() const { return who_am_i_; }
  bool is_my(SPI_HandleTypeDef * hspi) const { return config_.hspi == hspi; }

private:
  uint8_t transfer_byte(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
  void write_register(uint8_t reg, uint8_t value);
  void update_register(uint8_t reg, uint8_t clear_mask, uint8_t set_mask);
  bool verify_configuration();
  void rotate_to_body(float * v) const;
  uint64_t clock_micros() const;
  void delay_ms(uint32_t milliseconds) const;

  Config config_ = {};
  volatile bool initialized_ = false;
  volatile bool dma_busy_ = false;
  volatile bool new_sample_ = false;
  uint32_t init_status_ = 0;
  uint8_t who_am_i_ = 0;
  volatile uint64_t last_drdy_time_us_ = 0;
  rosflight_firmware::ImuStruct latest_sample_ = {};
};

#endif // ICM42688_H_
