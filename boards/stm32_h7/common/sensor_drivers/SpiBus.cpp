/**
 ******************************************************************************
 * File     : SpiBus.cpp
 * Date     : Aug 26, 2026
 ******************************************************************************
 **/
#include "SpiBus.h"
#include "Time64.h"
#include <cstring>
extern Time64 time64;
namespace
{
constexpr uint32_t SPI_DMA_SLOT_COUNT = 6;
DMA_RAM uint8_t spi_tx_dma_bufs[SPI_DMA_SLOT_COUNT][SPI_DMA_MAX_BUFFER_SIZE] = {};
DMA_RAM uint8_t spi_rx_dma_bufs[SPI_DMA_SLOT_COUNT][SPI_DMA_MAX_BUFFER_SIZE] = {};
bool spi_dma_slots_claimed[SPI_DMA_SLOT_COUNT] = {};
inline void wait_2us() { time64.dUs(2); }
bool assign_dma_buffers(uint8_t *& tx_dma_buf, uint8_t *& rx_dma_buf)
{
  for (uint32_t slot = 0; slot < SPI_DMA_SLOT_COUNT; slot++) {
    if (spi_dma_slots_claimed[slot]) {
      continue;
    }
    spi_dma_slots_claimed[slot] = true;
    tx_dma_buf = spi_tx_dma_bufs[slot];
    rx_dma_buf = spi_rx_dma_bufs[slot];
    return true;
  }
  tx_dma_buf = nullptr;
  rx_dma_buf = nullptr;
  return false;
}
}
uint32_t SpiBus::init(SPI_HandleTypeDef * hspi)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "SpiBus");
  initializationStatus_ = DRIVER_OK;
  hspi_ = hspi;
  active_ = nullptr;
  tx_dma_buf_ = nullptr;
  rx_dma_buf_ = nullptr;
  if (!assign_dma_buffers(tx_dma_buf_, rx_dma_buf_)) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  queue_head_ = nullptr;
  queue_tail_ = nullptr;
  return initializationStatus_;
}
SpiBus::TransferAwaiter::TransferAwaiter(
  SpiBus & bus, const Device & device, const uint8_t * tx, uint8_t * rx, uint16_t size)
    : bus_(bus)
{
  request_.device = &device;
  request_.tx = tx;
  request_.rx = rx;
  request_.size = size;
}
bool SpiBus::TransferAwaiter::await_suspend(std::coroutine_handle<> handle) noexcept
{
  request_.waiter = handle;
  const AsyncStatus status = bus_.submit(request_);
  if (status == AsyncStatus::OK) {
    return true;
  }
  request_.result.status = status;
  return false;
}
AsyncStatus SpiBus::submit(TransferAwaiter::Request & request)
{
  if (hspi_ == nullptr || tx_dma_buf_ == nullptr || rx_dma_buf_ == nullptr || request.device == nullptr
      || request.tx == nullptr || request.rx == nullptr || request.size == 0
      || request.size > SPI_DMA_MAX_BUFFER_SIZE) {
    return AsyncStatus::ERROR;
  }
  request.next = nullptr;
  if (active_ == nullptr) {
    return start_request(request) ? AsyncStatus::OK : AsyncStatus::HAL_ERROR;
  }
  if (queue_tail_ != nullptr) {
    queue_tail_->next = &request;
  } else {
    queue_head_ = &request;
  }
  queue_tail_ = &request;
  return AsyncStatus::OK;
}
bool SpiBus::start_request(TransferAwaiter::Request & request)
{
  std::memcpy(tx_dma_buf_, request.tx, request.size);
  std::memset(rx_dma_buf_, 0, request.size);
  if (hspi_->Init.NSS != SPI_NSS_HARD_OUTPUT) {
    HAL_GPIO_WritePin(request.device->cs_port, request.device->cs_pin, GPIO_PIN_RESET);
  }
  active_ = &request;
  const HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(hspi_, tx_dma_buf_, rx_dma_buf_, request.size);
  if (hal_status != HAL_OK) {
    end_transfer();
    active_ = nullptr;
    return false;
  }
  wait_2us();
  return true;
}
void SpiBus::end_transfer()
{
  if (active_ == nullptr) {
    return;
  }
  if (hspi_->Init.NSS != SPI_NSS_HARD_OUTPUT) {
    HAL_GPIO_WritePin(active_->device->cs_port, active_->device->cs_pin, GPIO_PIN_SET);
  }
  wait_2us();
}
void SpiBus::finish_active(AsyncStatus status)
{
  if (active_ == nullptr) {
    return;
  }
  TransferAwaiter::Request * finished = active_;
  end_transfer();
  std::memcpy(finished->rx, rx_dma_buf_, finished->size);
  finished->result.status = status;
  active_ = nullptr;
  if (queue_head_ != nullptr) {
    TransferAwaiter::Request * next = queue_head_;
    queue_head_ = queue_head_->next;
    if (queue_head_ == nullptr) {
      queue_tail_ = nullptr;
    }
    if (!start_request(*next)) {
      next->result.status = AsyncStatus::HAL_ERROR;
      if (next->waiter) {
        std::coroutine_handle<> waiter = next->waiter;
        next->waiter = {};
        waiter.resume();
      }
    }
  }
  if (finished->waiter) {
    std::coroutine_handle<> waiter = finished->waiter;
    finished->waiter = {};
    waiter.resume();
  }
}
void SpiBus::spiTxRxCpltCallback()
{
  finish_active(AsyncStatus::OK);
}
