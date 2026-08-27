# Async Framework in `rosflight_firmware`

This repository has two layers:

- The platform-independent flight stack lives mostly in `src/` and `include/`.
- The async framework lives in the STM32H7 board support code under `boards/stm32_h7/common/` and `boards/stm32_h7/common/sensor_drivers/`.

The important build detail is that the generic firmware library is built as C++17, while STM32H7 board targets are built as C++20 in `boards/stm32_h7/common/stm32_h7_board.cmake`. That is what enables the coroutine-based code in the board layer.

## What the framework is

This is not a general task scheduler or an RTOS wrapper. It is a small, interrupt-driven coroutine layer used mainly for sensor and bus drivers.

The design goals are:

- no heap allocation
- no background thread
- no separate event loop
- resume work directly from hardware interrupts
- express multi-step sensor transactions as linear code with `co_await`

In practice, the framework is built from four pieces:

1. `AsyncTask<void>` in `boards/stm32_h7/common/sensor_drivers/Async.h`
2. `AcquisitionSignal`, `ExtiSignal`, and `UartSignal` in the same file
3. the callback dispatcher in `boards/stm32_h7/common/Callbacks.h` and `.cpp`
4. async SPI/I2C bus wrappers in `SpiBus.*` and `I2cBus.*`

## Core coroutine primitive

`AsyncTask<void>` is the coroutine return type.

- `initial_suspend()` returns `std::suspend_never`, so the coroutine starts running immediately when `run()` is called.
- `final_suspend()` returns `std::suspend_always`, so the coroutine frame stays alive until the owning `AsyncTask` destroys it.
- the task object just owns a `std::coroutine_handle`; there is no scheduler state and no result value.

That means code like this starts the coroutine immediately:

```cpp
task_ = run();
```

Drivers keep the task object as a member, for example `task_`, `pps_task_`, or `baro_task_`. That member owns the coroutine frame for the lifetime of the driver object.

## Signals and wakeups

`AcquisitionSignal` is the main synchronization primitive.

It supports two await patterns:

- `wait_for_trigger()` pauses until someone calls `trigger()`.
- `delay_ticks(n)` pauses until `tick(current_tick)` advances far enough.

Internally it stores:

- one waiting coroutine handle
- one wait mode: none, trigger, or delay
- the current poll tick
- an optional deadline tick
- one pending trigger flag

This has a few important consequences:

- A signal supports only one suspended waiter at a time.
- Trigger events are level-latched only as a single pending bit, not a counted queue.
- If a trigger arrives before the coroutine waits, the next `wait_for_trigger()` consumes the pending flag and continues immediately.

`ExtiSignal` adds GPIO EXTI matching plus a captured IRQ timestamp.

`UartSignal` adds UART handle matching so a specific UART event path can wake a specific coroutine.

Today that event path is used for UART idle detection. `UartSignal` itself does not own a buffer or encode completion semantics; it is just a handle-matched wake source layered on top of `AcquisitionSignal`.

## How interrupts reach coroutines

`STM32H7Callbacks` is the central dispatcher.

At board init time, the board registers callback clients for:

- periodic polling
- EXTI interrupts
- SPI DMA completion
- I2C DMA completion
- UART RX complete, RX ISR, idle, and TX complete
- ADC completion
- USB CDC callbacks
- SD callbacks

The dispatcher itself is simple:

- it stores fixed-size arrays of clients
- each client has a `matches(...)` function and a callback function
- HAL callbacks fan events out to all matching clients

Examples in `Callbacks.cpp`:

- `HAL_TIM_PeriodElapsedCallback()` increments a global `poll_counter` and calls `dispatch_poll(...)`
- `HAL_GPIO_EXTI_Callback()` timestamps the interrupt and calls `dispatch_exti(...)`
- `HAL_SPI_TxRxCpltCallback()` calls `dispatch_spi(...)`
- `HAL_I2C_MasterTxCpltCallback()` and `HAL_I2C_MasterRxCpltCallback()` call the I2C dispatchers
- UART and CDC callbacks do the same for serial drivers

The UART idle path is slightly special:

- `UART_RxIsrCallback()` checks `UART_FLAG_IDLE`
- if idle is set, it clears the idle flag, disables RX DMA if present, and calls `dispatch_uart_idle(...)`
- after that, it still calls `dispatch_uart_rxisr(...)` so legacy RXISR-based drivers continue to work

The critical behavior is that coroutine resumption happens inside these callbacks. There is no deferred "run later on the main loop" step.

## Polling timer role

The polling framework is defined in `Polling.h` and `Polling.cpp`.

`PollingTimer` configures a hardware timer to fire periodically. On STM32H7 this is used as a high-rate heartbeat for drivers that need timed state progression.

Two helpers matter most:

- `period_us()` and `frequency_hz()` expose the timer rate.
- `polling_state(poll_counter, rollover_us, state)` converts the monotonic poll counter into a repeating phase index.

Many coroutine-based drivers use a pattern like this in `poll()`:

```cpp
poll_signal_.tick(poll_counter);
if (poll_state == SOME_PHASE) {
  poll_signal_.trigger();
}
```

That does two jobs:

- it advances any `delay_ticks(...)` waits
- it injects a start trigger at a specific phase in the repeating schedule

`register_poll_client(this, phase_offset)` can shift a driver's effective poll phase. The callback dispatcher applies the offset before invoking `poll()`.

## Async SPI and I2C buses

The bus wrappers are where `co_await` meets DMA.

### `SpiBus`

`SpiBus::transfer(...)` returns an awaiter whose request object lives inside the suspended coroutine frame.

When the coroutine does:

```cpp
co_await async_bus_->transfer(device, tx, rx, size)
```

the awaiter:

- stores the coroutine handle
- submits a request to the bus
- starts DMA immediately if the bus is idle
- otherwise appends the request to an intrusive linked list queue

When DMA completes:

- `HAL_SPI_TxRxCpltCallback()` fires
- `STM32H7Callbacks` routes the event to the matching `SpiBus`
- `SpiBus::spiTxRxCpltCallback()` finishes the active request
- the bus copies DMA RX bytes back into the caller buffer
- the next queued request, if any, is started
- the finished coroutine is resumed

Notable details:

- each `SpiBus` instance claims one pair of static DMA buffers from a small global pool
- software-controlled chip select is asserted before DMA and released after completion
- the queue is unbounded only in theory; in reality it is limited by how many coroutine frames exist

### `I2cBus`

`I2cBus` follows the same pattern, but splits operations into `write(...)` and `read(...)` awaiters.

Differences from SPI:

- separate TX-complete and RX-complete callbacks
- RX data is copied back only for read operations
- there is no explicit chip-select handling

Like `SpiBus`, each instance claims static DMA buffers from a small pool.

## Driver patterns built on top

The repo uses a few repeating patterns.

### 1. EXTI-driven sensor coroutine

Examples: `Bmi088`, `Adis165xx`

Pattern:

- driver initializes an `ExtiSignal`
- `start()` registers that signal with the dispatcher
- `task_ = run()` starts the coroutine
- the coroutine waits on `co_await exti_signal_.wait_for_trigger()`
- after DRDY, it performs one or more async SPI transfers and publishes a packet

This is the cleanest event-driven path: hardware ready interrupt directly wakes the coroutine.

### 2. Poll-driven timed transaction

Examples: `Dps310`, `Iis2mdc`, `DlhrL20G`, `Ms4525`, `Ist8308`

Pattern:

- `poll()` runs from the periodic timer callback
- `poll()` calls `poll_signal_.tick(poll_counter)` every time
- when the driver reaches a configured phase, `poll()` calls `poll_signal_.trigger()`
- the coroutine wakes, performs one DMA transaction step, then uses `delay_ticks(...)` to wait for the next phase

This lets multi-step protocols read like straight-line code while still being paced by the polling timer.

`Dps310::run()` is a good representative example: it alternates command, readiness check, pressure read, temperature command, readiness check, and temperature read with `delay_ticks(...)` between stages.

### 3. UART event coroutine

Examples: `Sbus`, `Ubx`

Pattern:

- start UART DMA once at coroutine startup
- wait on a signal triggered by UART idle or RX callbacks
- parse the DMA buffer
- restart DMA

For `Sbus`, the wake source is a dedicated `UartSignal` registered through `register_uart_idle_signal(...)`.

The flow is:

- `start()` registers `uart_idle_signal_` and launches `task_ = run()`
- `run()` starts UART DMA and then waits on `co_await uart_idle_signal_.wait_for_trigger()`
- the UART ISR notices an idle gap, disables DMA, and triggers the signal
- the coroutine resumes, parses the fixed-format SBUS frame from the DMA buffer, publishes it, and restarts DMA

That keeps the IRQ-side work minimal: the interrupt only stops DMA and wakes the coroutine, while packet parsing stays in coroutine context.

This design fits SBUS because packets are fixed-size, well-spaced, and expected to terminate with an idle gap before the DMA buffer fills.

For `Ubx`, the driver uses two async flows:

- `pps_task_` waits on an `ExtiSignal` for PPS timestamps
- `ubx_task_` waits on a generic `AcquisitionSignal` triggered by UART callbacks

So the async framework currently supports both UART styles:

- a dedicated idle-event awaitable path for protocols like SBUS
- the older RXISR/RX-complete callback path for streaming or byte-oriented drivers

### 4. Traditional callback-only drivers

Examples: `Adc`, `Telem`, `Vcp`, `Sd`

Not every asynchronous peripheral uses coroutines. Some drivers still use direct callbacks and FIFO-style buffering without `co_await`.

So the board support package mixes two styles:

- coroutine-based async sequencing for sensor acquisition
- classic callback/state-machine code for some other peripherals

## Startup sequence

The board-specific `STM32H7_Init.cpp` files show how everything is wired.

The order is roughly:

1. clear callback registrations
2. initialize shared async bus objects like `spi_bus_hspi1_` and `i2c_bus_hi2c1_`
3. register those bus objects with `STM32H7Callbacks`
4. initialize drivers
5. attach a driver to a shared async bus, when needed
6. call `driver.start(*this, maybe_phase_offset)`
7. inside `start()`, register the signal or poll client and then start the coroutine with `task_ = run()`

So the framework becomes active during board bring-up, not from the generic firmware main loop.

## Execution model in one sentence

The async framework is a set of long-lived driver coroutines that suspend on signals or DMA awaiters and then resume directly from STM32 interrupt callbacks.

## Important constraints and caveats

- There is no scheduler fairness policy beyond interrupt order and bus queue order.
- Signal objects support only one waiter.
- Signal triggers are not counted; multiple fast events can collapse into one pending trigger.
- Coroutine code may resume in interrupt context, so it must stay ISR-safe.
- The dedicated UART idle path is a good fit only when the protocol guarantees a meaningful idle boundary before the DMA buffer fills; otherwise a byte-stream or RX callback path is safer.
- `AsyncStatus::BUSY` and `AsyncStatus::QUEUE_FULL` exist in the enum but are not currently used by the SPI/I2C bus implementations.
- The coroutine layer is board-specific; unit-test and core firmware code do not depend on it.

## How to add a new async driver

The usual pattern is:

1. Add an `AsyncTask<void>` member to own the coroutine frame.
2. Add one or more signals: `AcquisitionSignal`, `ExtiSignal`, or `UartSignal`.
3. If the driver uses SPI or I2C, add `attach_bus(...)` and store a pointer to `SpiBus` or `I2cBus`.
4. Implement `run()` as a `while (true)` coroutine using `co_await` on signals and bus operations.
5. Implement either `poll()` or an IRQ-trigger path that calls `trigger()`.
6. In `start()`, register the signal or poll client and then start the coroutine with `task_ = run()`.

If the device protocol has several timed stages, prefer the poll-plus-`delay_ticks(...)` model. If the device already provides a data-ready interrupt, prefer an `ExtiSignal`-driven coroutine. If the device emits fixed-size packets with a reliable idle gap, a `UartSignal`-driven coroutine can be a good fit.
