#include "SbusUart.h"

#include <algorithm>
#include <cstring>

bool SbusUart::init(const Config & config)
{
  config_ = config;
  initialized_ = false;
  have_sample_ = false;
  latest_rc_ = {};
  dma_start_us_ = 0;
  timeout_us_ = 0;

  if (config_.huart == nullptr || config_.uart_instance == nullptr || config_.hdma_uart_rx == nullptr
      || config_.clock_micros == nullptr || config_.dma_rx_buffer == nullptr
      || config_.dma_rx_buffer_size < SBUS_FRAME_BYTES) {
    return false;
  }

  initialized_ = init_uart();
  if (initialized_) { (void) start_dma(); }
  return initialized_;
}

void SbusUart::poll()
{
  if (!initialized_) { return; }

  const uint64_t now = clock_micros();
  if (timeout_us_ == 0U || now <= timeout_us_) { return; }

  if (!dma_running()) {
    (void) HAL_UART_Abort(config_.huart);
    (void) start_dma();
  } else {
    timeout_us_ = now + SBUS_RX_TIMEOUT_US;
  }
}

bool SbusUart::read(rosflight_firmware::RcStruct * rc)
{
  if (rc == nullptr || !have_sample_) { return false; }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool have_sample = have_sample_;
  if (have_sample) {
    *rc = latest_rc_;
    have_sample_ = false;
  }
  if (!primask) { __enable_irq(); }

  return have_sample;
}

void SbusUart::handle_uart_rx_complete()
{
  if (!initialized_) { return; }

  rosflight_firmware::RcStruct rc = {};
  const size_t received = config_.dma_rx_buffer_size - __HAL_DMA_GET_COUNTER(config_.hdma_uart_rx);
  bool decoded = false;

  for (size_t i = 0; i + SBUS_FRAME_BYTES <= received; i++) {
    if (decode_frame(&config_.dma_rx_buffer[i], &rc)) {
      decoded = true;
      break;
    }
  }

  if (decoded) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    latest_rc_ = rc;
    have_sample_ = true;
    if (!primask) { __enable_irq(); }
    timeout_us_ = dma_start_us_ + SBUS_RX_TIMEOUT_US;
  }

  (void) HAL_UART_Abort(config_.huart);
  (void) start_dma();
}

void SbusUart::handle_uart_error()
{
  if (!initialized_) { return; }

  __HAL_UART_CLEAR_PEFLAG(config_.huart);
  __HAL_UART_CLEAR_FEFLAG(config_.huart);
  __HAL_UART_CLEAR_NEFLAG(config_.huart);
  __HAL_UART_CLEAR_OREFLAG(config_.huart);
  (void) HAL_UART_Abort(config_.huart);
  (void) start_dma();
}

bool SbusUart::init_uart()
{
  config_.huart->Instance = config_.uart_instance;
  config_.huart->Init.BaudRate = config_.baud;
  config_.huart->Init.WordLength = UART_WORDLENGTH_9B;
  config_.huart->Init.StopBits = UART_STOPBITS_2;
  config_.huart->Init.Parity = UART_PARITY_EVEN;
  config_.huart->Init.Mode = UART_MODE_TX_RX;
  config_.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  config_.huart->Init.OverSampling = UART_OVERSAMPLING_16;
  config_.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  config_.huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  config_.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT | UART_ADVFEATURE_RXINVERT_INIT;
  config_.huart->AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  config_.huart->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

  if (HAL_UART_Init(config_.huart) != HAL_OK) { return false; }
  if (HAL_UARTEx_SetTxFifoThreshold(config_.huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { return false; }
  if (HAL_UARTEx_SetRxFifoThreshold(config_.huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { return false; }
  if (HAL_UARTEx_DisableFifoMode(config_.huart) != HAL_OK) { return false; }

  __HAL_UART_CLEAR_IDLEFLAG(config_.huart);
  __HAL_UART_DISABLE_IT(config_.huart, UART_IT_IDLE);
  return true;
}

bool SbusUart::start_dma()
{
  if (config_.huart == nullptr) { return false; }

  std::memset(config_.dma_rx_buffer, 0, config_.dma_rx_buffer_size);
  dma_start_us_ = clock_micros();
  timeout_us_ = dma_start_us_ + SBUS_RX_TIMEOUT_US;

  __HAL_UART_CLEAR_IDLEFLAG(config_.huart);
  __HAL_UART_ENABLE_IT(config_.huart, UART_IT_IDLE);
  return HAL_UART_Receive_DMA(config_.huart, config_.dma_rx_buffer, config_.dma_rx_buffer_size) == HAL_OK;
}

bool SbusUart::dma_running() const
{
  if (config_.hdma_uart_rx == nullptr || config_.hdma_uart_rx->Instance == nullptr) { return false; }
  return (((DMA_Stream_TypeDef *) config_.hdma_uart_rx->Instance)->CR & DMA_SxCR_EN) == DMA_SxCR_EN;
}

bool SbusUart::decode_frame(const uint8_t * frame, rosflight_firmware::RcStruct * rc) const
{
  if (frame == nullptr || rc == nullptr || frame[0] != 0x0FU || !valid_footer(frame[24])) { return false; }

  uint16_t raw[16] = {};
  raw[0] = static_cast<uint16_t>((frame[1] | frame[2] << 8) & 0x07FFU);
  raw[1] = static_cast<uint16_t>(((frame[2] >> 3) | (frame[3] << 5)) & 0x07FFU);
  raw[2] = static_cast<uint16_t>(((frame[3] >> 6) | (frame[4] << 2) | (frame[5] << 10)) & 0x07FFU);
  raw[3] = static_cast<uint16_t>(((frame[5] >> 1) | (frame[6] << 7)) & 0x07FFU);
  raw[4] = static_cast<uint16_t>(((frame[6] >> 4) | (frame[7] << 4)) & 0x07FFU);
  raw[5] = static_cast<uint16_t>(((frame[7] >> 7) | (frame[8] << 1) | (frame[9] << 9)) & 0x07FFU);
  raw[6] = static_cast<uint16_t>(((frame[9] >> 2) | (frame[10] << 6)) & 0x07FFU);
  raw[7] = static_cast<uint16_t>(((frame[10] >> 5) | (frame[11] << 3)) & 0x07FFU);
  raw[8] = static_cast<uint16_t>((frame[12] | frame[13] << 8) & 0x07FFU);
  raw[9] = static_cast<uint16_t>(((frame[13] >> 3) | (frame[14] << 5)) & 0x07FFU);
  raw[10] = static_cast<uint16_t>(((frame[14] >> 6) | (frame[15] << 2) | (frame[16] << 10)) & 0x07FFU);
  raw[11] = static_cast<uint16_t>(((frame[16] >> 1) | (frame[17] << 7)) & 0x07FFU);
  raw[12] = static_cast<uint16_t>(((frame[17] >> 4) | (frame[18] << 4)) & 0x07FFU);
  raw[13] = static_cast<uint16_t>(((frame[18] >> 7) | (frame[19] << 1) | (frame[20] << 9)) & 0x07FFU);
  raw[14] = static_cast<uint16_t>(((frame[20] >> 2) | (frame[21] << 6)) & 0x07FFU);
  raw[15] = static_cast<uint16_t>(((frame[21] >> 5) | (frame[22] << 3)) & 0x07FFU);

  rc->nChan = RC_STRUCT_CHANNELS;
  for (size_t i = 0; i < 16U && i < RC_STRUCT_CHANNELS; i++) {
    rc->chan[i] = (static_cast<float>(raw[i]) - 172.0f) / 1639.0f;
  }

  const uint8_t flags = frame[23];
  for (size_t i = 16U; i < RC_STRUCT_CHANNELS; i++) {
    rc->chan[i] = (flags & (1U << (i - 16U))) ? 0.0f : 1.0f;
  }

  rc->frameLost = (flags & (1U << 2U)) != 0U;
  rc->failsafeActivated = (flags & (1U << 3U)) != 0U;
  const uint64_t now = clock_micros();
  rc->header.timestamp = dma_start_us_;
  rc->header.complete = now;
  rc->header.status = !(rc->frameLost | rc->failsafeActivated);
  return true;
}

bool SbusUart::valid_footer(uint8_t footer)
{
  return footer == 0x00U || footer == 0x04U || footer == 0x14U || footer == 0x24U || footer == 0x34U;
}

uint64_t SbusUart::clock_micros() const
{
  return config_.clock_micros == nullptr ? 0U : config_.clock_micros();
}
