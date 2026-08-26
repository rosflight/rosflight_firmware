/**
 ******************************************************************************
 * File     : Async.h
 * Date     : Aug 26, 2026
 ******************************************************************************
 **/

#ifndef DRIVERS_ASYNC_H_
#define DRIVERS_ASYNC_H_

#include <coroutine>
#include <cstdint>

enum class AsyncStatus : uint8_t
{
  OK = 0,
  ERROR,
  BUSY,
  QUEUE_FULL,
  HAL_ERROR,
};

template<typename T = void>
class AsyncTask;

template<>
class AsyncTask<void>
{
public:
  struct promise_type
  {
    AsyncTask get_return_object()
    {
      return AsyncTask{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_never initial_suspend() noexcept { return {}; }
    std::suspend_always final_suspend() noexcept { return {}; }
    void return_void() noexcept {}
    void unhandled_exception() { __builtin_trap(); }
  };

  AsyncTask() = default;

  explicit AsyncTask(std::coroutine_handle<promise_type> handle)
      : handle_(handle)
  {}

  AsyncTask(AsyncTask && other) noexcept
      : handle_(other.handle_)
  {
    other.handle_ = {};
  }

  AsyncTask & operator=(AsyncTask && other) noexcept
  {
    if (this != &other) {
      reset();
      handle_ = other.handle_;
      other.handle_ = {};
    }
    return *this;
  }

  AsyncTask(const AsyncTask &) = delete;
  AsyncTask & operator=(const AsyncTask &) = delete;

  ~AsyncTask() { reset(); }

private:
  void reset()
  {
    if (handle_) {
      handle_.destroy();
      handle_ = {};
    }
  }

  std::coroutine_handle<promise_type> handle_{};
};

class AcquisitionSignal
{
public:
  class TriggerAwaiter
  {
  public:
    explicit TriggerAwaiter(AcquisitionSignal & signal)
        : signal_(signal)
    {}

    bool await_ready() noexcept { return signal_.consume_trigger(); }

    bool await_suspend(std::coroutine_handle<> handle) noexcept
    {
      if (signal_.consume_trigger()) {
        return false;
      }

      signal_.waiter_ = handle;
      signal_.wait_mode_ = WaitMode::TRIGGER;
      return true;
    }

    void await_resume() const noexcept {}

  private:
    AcquisitionSignal & signal_;
  };

  class DelayAwaiter
  {
  public:
    DelayAwaiter(AcquisitionSignal & signal, uint64_t ticks)
        : signal_(signal)
        , deadline_(signal.current_tick_ + ticks)
    {}

    bool await_ready() const noexcept { return signal_.current_tick_ >= deadline_; }

    bool await_suspend(std::coroutine_handle<> handle) noexcept
    {
      if (signal_.current_tick_ >= deadline_) {
        return false;
      }

      signal_.waiter_ = handle;
      signal_.wait_mode_ = WaitMode::DELAY;
      signal_.deadline_tick_ = deadline_;
      return true;
    }

    void await_resume() const noexcept {}

  private:
    AcquisitionSignal & signal_;
    uint64_t deadline_ = 0;
  };

  TriggerAwaiter wait_for_trigger() { return TriggerAwaiter(*this); }
  DelayAwaiter delay_ticks(uint64_t ticks) { return DelayAwaiter(*this, ticks); }

  void tick(uint64_t current_tick)
  {
    current_tick_ = current_tick;
    if (wait_mode_ == WaitMode::DELAY && waiter_ && current_tick_ >= deadline_tick_) {
      std::coroutine_handle<> waiter = waiter_;
      waiter_ = {};
      wait_mode_ = WaitMode::NONE;
      waiter.resume();
    }
  }

  void trigger()
  {
    if (wait_mode_ == WaitMode::TRIGGER && waiter_) {
      std::coroutine_handle<> waiter = waiter_;
      waiter_ = {};
      wait_mode_ = WaitMode::NONE;
      waiter.resume();
      return;
    }

    trigger_pending_ = true;
  }

private:
  enum class WaitMode : uint8_t
  {
    NONE,
    TRIGGER,
    DELAY,
  };

  bool consume_trigger()
  {
    if (!trigger_pending_) {
      return false;
    }

    trigger_pending_ = false;
    return true;
  }

  std::coroutine_handle<> waiter_{};
  WaitMode wait_mode_ = WaitMode::NONE;
  uint64_t current_tick_ = 0;
  uint64_t deadline_tick_ = 0;
  bool trigger_pending_ = false;
};

#endif /* DRIVERS_ASYNC_H_ */
