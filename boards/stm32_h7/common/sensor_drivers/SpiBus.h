/**
 ******************************************************************************
 * File     : SpiBus.h
 * Date     : Aug 26, 2026
 ******************************************************************************
 **/

#ifndef DRIVERS_SPIBUS_H_
#define DRIVERS_SPIBUS_H_

#include "Async.h"
#include "CommonConfig.h"
#include "Status.h"

#include <coroutine>
#include <cstdint>

class SpiBus : public Status
{
public:
  struct Device
  {
    GPIO_TypeDef * cs_port = nullptr;
    uint16_t cs_pin = 0;
  };

  struct Result
  {
    AsyncStatus status = AsyncStatus::ERROR;
  };

  uint32_t init(SPI_HandleTypeDef * hspi);
  bool isMy(SPI_HandleTypeDef * hspi) const { return hspi_ == hspi; }
  void spiTxRxCpltCallback();

  class TransferAwaiter
  {
  public:
    TransferAwaiter(SpiBus & bus, const Device & device, const uint8_t * tx, uint8_t * rx, uint16_t size);

    bool await_ready() const noexcept { return false; }
    bool await_suspend(std::coroutine_handle<> handle) noexcept;
    Result await_resume() const noexcept { return request_.result; }

  private:
    struct Request
    {
      const Device * device = nullptr;
      const uint8_t * tx = nullptr;
      uint8_t * rx = nullptr;
      uint16_t size = 0;
      Result result = {};
      std::coroutine_handle<> waiter{};
      Request * next = nullptr;
    };

    friend class SpiBus;

    SpiBus & bus_;
    Request request_{};
  };

  TransferAwaiter transfer(const Device & device, const uint8_t * tx, uint8_t * rx, uint16_t size)
  {
    return TransferAwaiter(*this, device, tx, rx, size);
  }

private:
  friend class TransferAwaiter;

  AsyncStatus submit(TransferAwaiter::Request & request);
  bool start_request(TransferAwaiter::Request & request);
  void finish_active(AsyncStatus status);
  void end_transfer();

  SPI_HandleTypeDef * hspi_ = nullptr;
  TransferAwaiter::Request * active_ = nullptr;
  TransferAwaiter::Request * queue_head_ = nullptr;
  TransferAwaiter::Request * queue_tail_ = nullptr;

  uint8_t * tx_dma_buf_ = nullptr;
  uint8_t * rx_dma_buf_ = nullptr;
};

#endif /* DRIVERS_SPIBUS_H_ */



