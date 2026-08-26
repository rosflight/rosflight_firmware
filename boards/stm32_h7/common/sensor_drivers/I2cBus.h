/**
 ******************************************************************************
 * File     : I2cBus.h
 * Date     : Aug 26, 2026
 ******************************************************************************
 **/

#ifndef DRIVERS_I2CBUS_H_
#define DRIVERS_I2CBUS_H_

#include "Async.h"
#include "CommonConfig.h"
#include "Status.h"

#include <coroutine>
#include <cstdint>

class I2cBus : public Status
{
public:
  struct Result
  {
    AsyncStatus status = AsyncStatus::ERROR;
  };

  uint32_t init(I2C_HandleTypeDef * hi2c);
  bool isMy(I2C_HandleTypeDef * hi2c) const { return hi2c_ == hi2c; }
  void i2cMasterTxCpltCallback();
  void i2cMasterRxCpltCallback();

  class WriteAwaiter
  {
  public:
    WriteAwaiter(I2cBus & bus, uint16_t address, const uint8_t * tx, uint16_t size);

    bool await_ready() const noexcept { return false; }
    bool await_suspend(std::coroutine_handle<> handle) noexcept;
    Result await_resume() const noexcept { return request_.result; }

  private:
    enum class Operation : uint8_t
    {
      WRITE,
      READ,
    };

    struct Request
    {
      Operation op = Operation::WRITE;
      uint16_t address = 0;
      const uint8_t * tx = nullptr;
      uint8_t * rx = nullptr;
      uint16_t size = 0;
      Result result = {};
      std::coroutine_handle<> waiter{};
      Request * next = nullptr;
    };

    friend class I2cBus;
    friend class ReadAwaiter;

    I2cBus & bus_;
    Request request_{};
  };

  class ReadAwaiter
  {
  public:
    ReadAwaiter(I2cBus & bus, uint16_t address, uint8_t * rx, uint16_t size);

    bool await_ready() const noexcept { return false; }
    bool await_suspend(std::coroutine_handle<> handle) noexcept;
    Result await_resume() const noexcept { return request_.result; }

  private:
    friend class I2cBus;

    I2cBus & bus_;
    WriteAwaiter::Request request_{};
  };

  WriteAwaiter write(uint16_t address, const uint8_t * tx, uint16_t size)
  {
    return WriteAwaiter(*this, address, tx, size);
  }

  ReadAwaiter read(uint16_t address, uint8_t * rx, uint16_t size)
  {
    return ReadAwaiter(*this, address, rx, size);
  }

private:
  friend class WriteAwaiter;
  friend class ReadAwaiter;

  AsyncStatus submit(WriteAwaiter::Request & request);
  bool start_request(WriteAwaiter::Request & request);
  void finish_active(AsyncStatus status, bool copy_rx);

  I2C_HandleTypeDef * hi2c_ = nullptr;
  WriteAwaiter::Request * active_ = nullptr;
  WriteAwaiter::Request * queue_head_ = nullptr;
  WriteAwaiter::Request * queue_tail_ = nullptr;

  uint8_t * tx_dma_buf_ = nullptr;
  uint8_t * rx_dma_buf_ = nullptr;
};

#endif /* DRIVERS_I2CBUS_H_ */



