#ifndef SBUS_UART_H
#define SBUS_UART_H

#include "interface/board.h"
#include "stm32h7xx_hal.h"

#include <cstddef>
#include <cstdint>

class SbusUart
{
public:
  struct Config
  {
    UART_HandleTypeDef * huart = nullptr;
    USART_TypeDef * uart_instance = nullptr;
    DMA_HandleTypeDef * hdma_uart_rx = nullptr;
    uint64_t (*clock_micros)() = nullptr;
    uint8_t * dma_rx_buffer = nullptr;
    size_t dma_rx_buffer_size = 0;
    uint32_t baud = 100000U;
  };

  bool init(const Config & config);
  void poll();
  bool read(rosflight_firmware::RcStruct * rc);
  void handle_uart_rx_complete();
  void handle_uart_error();

  bool is_my(UART_HandleTypeDef * huart) const { return config_.huart == huart; }

private:
  static constexpr size_t SBUS_FRAME_BYTES = 25U;
  static constexpr uint64_t SBUS_RX_TIMEOUT_US = 100000U;

  bool init_uart();
  bool start_dma();
  bool dma_running() const;
  bool decode_frame(const uint8_t * frame, rosflight_firmware::RcStruct * rc) const;
  static bool valid_footer(uint8_t footer);
  uint64_t clock_micros() const;

  Config config_ = {};
  volatile bool initialized_ = false;
  volatile bool have_sample_ = false;
  rosflight_firmware::RcStruct latest_rc_ = {};
  uint64_t dma_start_us_ = 0;
  uint64_t timeout_us_ = 0;
};

#endif // SBUS_UART_H
