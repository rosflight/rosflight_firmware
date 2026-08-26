/**
 ******************************************************************************
 * File     : I2cBus.cpp
 * Date     : Aug 26, 2026
 ******************************************************************************
 **/

#include "I2cBus.h"

#include <cstdio>
#include <cstring>

namespace
{
constexpr uint32_t I2C_DMA_SLOT_COUNT = 2;

DMA_RAM uint8_t i2c_tx_dma_bufs[I2C_DMA_SLOT_COUNT][I2C_DMA_MAX_BUFFER_SIZE] = {};
DMA_RAM uint8_t i2c_rx_dma_bufs[I2C_DMA_SLOT_COUNT][I2C_DMA_MAX_BUFFER_SIZE] = {};
bool i2c_dma_slots_claimed[I2C_DMA_SLOT_COUNT] = {};

bool assign_dma_buffers(uint8_t *& tx_dma_buf, uint8_t *& rx_dma_buf)
{
  for (uint32_t slot = 0; slot < I2C_DMA_SLOT_COUNT; slot++) {
    if (i2c_dma_slots_claimed[slot]) {
      continue;
    }

    i2c_dma_slots_claimed[slot] = true;
    tx_dma_buf = i2c_tx_dma_bufs[slot];
    rx_dma_buf = i2c_rx_dma_bufs[slot];
    return true;
  }

  tx_dma_buf = nullptr;
  rx_dma_buf = nullptr;
  return false;
}
}

uint32_t I2cBus::init(I2C_HandleTypeDef * hi2c)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "I2cBus");
  initializationStatus_ = DRIVER_OK;
  hi2c_ = hi2c;
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

I2cBus::WriteAwaiter::WriteAwaiter(I2cBus & bus, uint16_t address, const uint8_t * tx, uint16_t size)
    : bus_(bus)
{
  request_.op = Operation::WRITE;
  request_.address = address;
  request_.tx = tx;
  request_.size = size;
}

bool I2cBus::WriteAwaiter::await_suspend(std::coroutine_handle<> handle) noexcept
{
  request_.waiter = handle;
  const AsyncStatus status = bus_.submit(request_);
  if (status == AsyncStatus::OK) {
    return true;
  }

  request_.result.status = status;
  return false;
}

I2cBus::ReadAwaiter::ReadAwaiter(I2cBus & bus, uint16_t address, uint8_t * rx, uint16_t size)
    : bus_(bus)
{
  request_.op = WriteAwaiter::Operation::READ;
  request_.address = address;
  request_.rx = rx;
  request_.size = size;
}

bool I2cBus::ReadAwaiter::await_suspend(std::coroutine_handle<> handle) noexcept
{
  request_.waiter = handle;
  const AsyncStatus status = bus_.submit(request_);
  if (status == AsyncStatus::OK) {
    return true;
  }

  request_.result.status = status;
  return false;
}

AsyncStatus I2cBus::submit(WriteAwaiter::Request & request)
{
  if (hi2c_ == nullptr || tx_dma_buf_ == nullptr || rx_dma_buf_ == nullptr || request.size == 0
      || request.size > I2C_DMA_MAX_BUFFER_SIZE) {
    return AsyncStatus::ERROR;
  }

  if (request.op == WriteAwaiter::Operation::WRITE && request.tx == nullptr) {
    return AsyncStatus::ERROR;
  }

  if (request.op == WriteAwaiter::Operation::READ && request.rx == nullptr) {
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

bool I2cBus::start_request(WriteAwaiter::Request & request)
{
  active_ = &request;
  HAL_StatusTypeDef hal_status = HAL_ERROR;

  if (request.op == WriteAwaiter::Operation::WRITE) {
    std::memcpy(tx_dma_buf_, request.tx, request.size);
    hal_status = HAL_I2C_Master_Transmit_DMA(hi2c_, request.address, tx_dma_buf_, request.size);
  } else {
    std::memset(rx_dma_buf_, 0, request.size);
    hal_status = HAL_I2C_Master_Receive_DMA(hi2c_, request.address, rx_dma_buf_, request.size);
  }

  if (hal_status != HAL_OK) {
    active_ = nullptr;
    return false;
  }

  return true;
}

void I2cBus::finish_active(AsyncStatus status, bool copy_rx)
{
  if (active_ == nullptr) {
    return;
  }

  WriteAwaiter::Request * finished = active_;
  if (copy_rx && finished->rx != nullptr) {
    std::memcpy(finished->rx, rx_dma_buf_, finished->size);
  }
  finished->result.status = status;
  active_ = nullptr;

  if (queue_head_ != nullptr) {
    WriteAwaiter::Request * next = queue_head_;
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

void I2cBus::i2cMasterTxCpltCallback()
{
  finish_active(AsyncStatus::OK, false);
}

void I2cBus::i2cMasterRxCpltCallback()
{
  finish_active(AsyncStatus::OK, true);
}
