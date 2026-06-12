# Bus Sharing Architecture Proposal

## Current State

The Varmint firmware currently has drivers that individually own SPI/I2C bus handles. Each driver stores a pointer to the HAL handle (`SPI_HandleTypeDef*` or `I2C_HandleTypeDef*`) and performs DMA transactions independently. Bus conflicts are avoided by:

1. **Manual offset in polling** - See [Callbacks.cpp:61-62](../boards/varmint_h7/varmint_11X/specific/Callbacks.cpp#L61-L62) where baro and mag on shared SPI2 are offset by one poll count
2. **Multiple bus instances** - IMUs use separate SPI buses (SPI1 for BMI088, SPI4 for ADIS165xx)
3. **`isMy()` pattern in callbacks** - All drivers on a bus checked with if-statements (not else-if) in [HAL_SPI_TxRxCpltCallback](../boards/varmint_h7/varmint_11X/specific/Callbacks.cpp#L94-L101)

### Problems with Current Approach

- **Manual coordination required** - Poll offset timing is fragile and undocumented
- **No protection against conflicts** - Nothing prevents simultaneous DMA on shared bus
- **Limited scalability** - Adding more sensors to a shared bus requires careful manual timing
- **Unclear ownership** - No central registry of what's on each bus

## Proposed Architecture

### Core Concept: Bus Manager Pattern

Introduce lightweight bus manager classes (`SpiBus`, `I2cBus`) that:
1. Own the HAL handle and DMA buffers
2. Maintain a queue of pending transactions
3. Serialize access to the bus
4. Route completion callbacks to the correct driver

This fits the existing driver architecture by acting as a **mediator** between drivers and hardware, similar to how `DoubleBuffer` mediates between interrupt and main loop contexts.

### Design

#### 1. Bus Manager Classes

```cpp
// boards/varmint_h7/common/drivers/SpiBus.h

class SpiBus
{
public:
  /**
   * @brief Initialize bus manager with HAL handle and shared DMA buffers
   * @param hspi SPI peripheral handle
   * @param tx_buffer Shared transmit buffer (SPI_DMA_MAX_BUFFER_SIZE)
   * @param rx_buffer Shared receive buffer (SPI_DMA_MAX_BUFFER_SIZE)
   */
  void init(SPI_HandleTypeDef* hspi, uint8_t* tx_buffer, uint8_t* rx_buffer);
  
  /**
   * @brief Request a DMA transaction on this bus
   * @param owner Pointer to driver requesting transaction (for callback routing)
   * @param cs_port Chip select GPIO port
   * @param cs_pin Chip select GPIO pin
   * @param tx_data Data to transmit (will be copied to shared buffer)
   * @param size Transaction size in bytes
   * @return true if transaction started immediately, false if queued
   */
  bool transact(void* owner, GPIO_TypeDef* cs_port, uint16_t cs_pin, 
                uint8_t* tx_data, uint16_t size);
  
  /**
   * @brief Called from HAL_SPI_TxRxCpltCallback - routes to active driver
   * @return Pointer to rx buffer for driver to process
   */
  uint8_t* completionCallback();
  
  /**
   * @brief Check if this bus is idle (available for new transaction)
   */
  bool isIdle() const { return state_ == BusState::IDLE; }
  
  /**
   * @brief Check if this bus instance owns the given HAL handle
   */
  bool isMy(SPI_HandleTypeDef* hspi) const { return hspi_ == hspi; }
  
private:
  enum class BusState { IDLE, BUSY };
  
  struct Transaction {
    void* owner;              // Driver that initiated this transaction
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    uint16_t size;
  };
  
  SPI_HandleTypeDef* hspi_;
  uint8_t* txBuffer_;
  uint8_t* rxBuffer_;
  BusState state_ = BusState::IDLE;
  Transaction active_;         // Currently executing transaction
  Transaction queue_[4];       // Simple circular queue for pending transactions
  uint8_t queue_head_ = 0;
  uint8_t queue_tail_ = 0;
  
  void startTransaction(const Transaction& txn);
  void serviceQueue();
};

// boards/varmint_h7/common/drivers/I2cBus.h

class I2cBus
{
public:
  void init(I2C_HandleTypeDef* hi2c);
  
  /**
   * @brief Request a DMA read transaction
   * @param owner Pointer to driver requesting transaction
   * @param dev_address I2C device address (7-bit)
   * @param mem_address Register address
   * @param mem_size Address size (I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT)
   * @param rx_buffer Driver's buffer to receive data (NOT shared - I2C HAL limitation)
   * @param size Number of bytes to read
   * @return true if transaction started, false if queued
   */
  bool readReg(void* owner, uint16_t dev_address, uint16_t mem_address,
               uint16_t mem_size, uint8_t* rx_buffer, uint16_t size);
  
  /**
   * @brief Request a DMA write transaction
   */
  bool writeReg(void* owner, uint16_t dev_address, uint16_t mem_address,
                uint16_t mem_size, uint8_t* tx_data, uint16_t size);
  
  void completionCallback();
  bool isIdle() const { return state_ == BusState::IDLE; }
  bool isMy(I2C_HandleTypeDef* hi2c) const { return hi2c_ == hi2c; }
  
private:
  enum class BusState { IDLE, BUSY };
  
  struct Transaction {
    void* owner;
    uint16_t dev_address;
    uint16_t mem_address;
    uint16_t mem_size;
    uint8_t* buffer;          // Points to driver's buffer
    uint16_t size;
    bool is_read;
  };
  
  I2C_HandleTypeDef* hi2c_;
  BusState state_ = BusState::IDLE;
  Transaction active_;
  Transaction queue_[4];
  uint8_t queue_head_ = 0;
  uint8_t queue_tail_ = 0;
  
  void startTransaction(const Transaction& txn);
  void serviceQueue();
};
```

#### 2. Modified Driver Pattern

Drivers change from owning a `Spi` helper object to holding a pointer to a shared `SpiBus`:

```cpp
// boards/varmint_h7/common/drivers/Dps310.h (BEFORE)
class Dps310 : public Status
{
  Spi spi_;  // Owns SPI resources
  // ...
};

// boards/varmint_h7/common/drivers/Dps310.h (AFTER)
class Dps310 : public Status
{
  SpiBus* bus_;           // Pointer to shared bus manager
  GPIO_TypeDef* csPort_;  // Chip select ownership stays with driver
  uint16_t csPin_;
  // ...
};
```

Driver initialization changes to receive bus manager reference:

```cpp
// BEFORE
uint32_t Dps310::init(uint16_t sample_rate_hz, GPIO_TypeDef* drdy_port, 
                      uint16_t drdy_pin, SPI_HandleTypeDef* hspi, 
                      GPIO_TypeDef* cs_port, uint16_t cs_pin, bool three_wire);

// AFTER
uint32_t Dps310::init(uint16_t sample_rate_hz, GPIO_TypeDef* drdy_port,
                      uint16_t drdy_pin, SpiBus* bus,
                      GPIO_TypeDef* cs_port, uint16_t cs_pin, bool three_wire);
```

#### 3. Board Configuration

Board-specific config instantiates bus managers and passes them to drivers:

```cpp
// boards/varmint_h7/varmint_11X/specific/BoardConfig.h

// Bus managers (one per physical bus)
#define BUS_MANAGERS \
  SpiBus spi1_bus_;  /* IMU BMI088 */ \
  SpiBus spi2_bus_;  /* Baro (DPS310) + Mag (IIS2MDC) - SHARED */ \
  SpiBus spi4_bus_;  /* IMU ADIS165xx */ \
  I2cBus i2c1_bus_;  /* Pitot (DLHR) + Servo voltage (MCP4017) - SHARED */ \
  /**/

// Modified interface list - drivers receive bus pointers
#define INTERFACE_LIST \
  BUS_MANAGERS \
  Sbus rc_; \
  /* ... */ \
  Dps310 baro_;    /* Initialized with &spi2_bus_ */ \
  Iis2mdc mag_;    /* Initialized with &spi2_bus_ */ \
  Adis165xx imu0_; /* Initialized with &spi4_bus_ */ \
  Bmi088 imu1_;    /* Initialized with &spi1_bus_ */ \
  DlhrL20G pitot_; /* Initialized with &i2c1_bus_ */ \
  /* ... */ \
  /**/
```

Initialization in `Varmint::init_board()`:

```cpp
// Initialize bus managers first
spi1_bus_.init(&hspi1, spi1_tx_buffer, spi1_rx_buffer);
spi2_bus_.init(&hspi2, spi2_tx_buffer, spi2_rx_buffer);
spi4_bus_.init(&hspi4, spi4_tx_buffer, spi4_rx_buffer);
i2c1_bus_.init(&hi2c1);

// Initialize drivers with bus references
baro_.init(DPS310_HZ, BARO_DRDY_GPIO_Port, BARO_DRDY_Pin, 
           &spi2_bus_, BARO_CS_GPIO_Port, BARO_CS_Pin, DPS310_3_WIRE);
mag_.init(IIS2MDC_HZ, &spi2_bus_, MAG_CS_GPIO_Port, MAG_CS_Pin, IIS2MDC_ROTATION);
```

#### 4. Callback Routing

Callbacks simplify to bus-level routing:

```cpp
// boards/varmint_h7/varmint_11X/specific/Callbacks.cpp

// BEFORE: Check every driver on every callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (varmint.imu0_.isMy(hspi)) { varmint.imu0_.endDma(); }
  if (varmint.imu1_.isMy(hspi)) { varmint.imu1_.endDma(); }
  if (varmint.mag_.isMy(hspi)) { varmint.mag_.endDma(); }
  if (varmint.baro_.isMy(hspi)) { varmint.baro_.endDma(); }
}

// AFTER: Bus manager routes to active driver
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (varmint.spi1_bus_.isMy(hspi)) {
    uint8_t* rx_data = varmint.spi1_bus_.completionCallback();
    // Bus manager internally calls the active driver's endDma()
  }
  if (varmint.spi2_bus_.isMy(hspi)) {
    uint8_t* rx_data = varmint.spi2_bus_.completionCallback();
  }
  if (varmint.spi4_bus_.isMy(hspi)) {
    uint8_t* rx_data = varmint.spi4_bus_.completionCallback();
  }
}
```

### How Bus Manager Routes Callbacks

Two possible approaches:

#### Option A: Function Pointer Registration
Drivers register a completion callback with the bus manager:

```cpp
class SpiBus {
  using CompletionCallback = void (*)(void* driver, uint8_t* rx_data);
  
  bool transact(void* owner, CompletionCallback callback, /* ... */);
  
  uint8_t* completionCallback() {
    if (active_.callback) {
      active_.callback(active_.owner, rxBuffer_);
    }
    serviceQueue();
    return rxBuffer_;
  }
};

// In driver
bus_->transact(this, [](void* self, uint8_t* data) {
  static_cast<Dps310*>(self)->endDma(data);
}, /* ... */);
```

#### Option B: Virtual Base Class (Cleaner)
Define a bus client interface:

```cpp
class SpiClient {
public:
  virtual void spiTransactionComplete(uint8_t* rx_data) = 0;
};

class SpiBus {
  bool transact(SpiClient* owner, /* ... */);
  
  void completionCallback() {
    if (active_.owner) {
      active_.owner->spiTransactionComplete(rxBuffer_);
    }
    serviceQueue();
  }
};

// Drivers inherit from interface
class Dps310 : public Status, public SpiClient {
  void spiTransactionComplete(uint8_t* rx_data) override {
    // Process rx_data, finalize packet, write to double buffer
  }
};
```

**Recommendation: Option B** - Type-safe, clearer intent, no function pointer overhead.

### Queue Depth Rationale

A queue depth of 4 transactions is sufficient because:
1. Typical boards have 2-3 sensors per bus
2. Polling period (100μs) is much longer than SPI transaction time (~10-50μs for typical sensor reads)
3. Collision window is narrow - only if multiple sensors assert DRDY within the same 100μs window
4. Queue overflow can be detected and flagged as a sensor error

For I2C (slower), the same queue depth works because I2C sensors are polled less frequently (50-100Hz vs 1-2kHz for IMUs).

## Migration Path

### Phase 1: Add Bus Managers Alongside Existing Code
- Implement `SpiBus` and `I2cBus` classes
- Add unit tests for queueing behavior
- No driver changes yet - bus managers exist but unused

### Phase 2: Migrate Non-Critical Sensors
- Start with baro and mag (shared SPI2)
- Remove manual poll offset hack
- Validate no timing regressions

### Phase 3: Migrate IMUs
- Higher risk due to real-time requirements
- Validate latency impact is negligible
- Benchmark IMU sample timestamps for jitter

### Phase 4: Migrate I2C Sensors
- Pitot and other I2C peripherals
- Simplifies addition of future I2C sensors

### Phase 5: Remove Old `Spi` Helper Class
- Once all drivers migrated, delete `Spi.h`
- Cleanup

## Benefits

1. **Explicit bus ownership** - Clear which sensors share which buses
2. **Automatic arbitration** - No manual timing coordination needed
3. **Scalability** - Easy to add sensors to existing buses
4. **Maintainability** - Bus sharing logic centralized, not scattered across polling code
5. **Debugging** - Can log queue depth, detect starvation, measure bus utilization
6. **Consistency** - Matches existing patterns (`DoubleBuffer`, driver base classes)

## Compatibility Notes

- **Maintains existing timing** - Transactions still initiated from poll or DRDY interrupts
- **No malloc** - Fixed-size queues, allocated at compile time
- **ISR-safe** - Queue operations are simple enough for interrupt context
- **Zero-copy for SPI** - Drivers see same DMA buffer pointer
- **I2C caveat** - I2C HAL requires driver-owned buffers (HAL limitation), so less sharing benefit

## Alternative Considered: FreeRTOS Mutexes

Using RTOS mutexes for bus locking was rejected because:
- Adds RTOS dependency (ROSflight currently bare-metal)
- Mutex overhead (context switches) unacceptable for 1-2kHz IMU reads
- Priority inversion risk with high-rate sensors
- Queue-based approach fits better with existing interrupt-driven architecture

## Open Questions

1. **Queue overflow policy** - Drop transaction? Block caller? Flag error?
   - **Recommendation**: Flag error in sensor status, drop oldest queued transaction
   
2. **Priority** - Should IMUs preempt lower-rate sensors?
   - **Recommendation**: Start FIFO, add priority later if needed (YAGNI)
   
3. **Bus speed differences** - Some sensors require different SPI clock speeds
   - **Recommendation**: Bus manager can reconfigure clock via `HAL_SPI_Init()` per transaction if needed, but prefer fixed speed per bus (simpler, faster)

4. **Blocking vs non-blocking** - Should `transact()` block until completion?
   - **Recommendation**: Always non-blocking (queue), matches existing async DMA pattern

## Testing Strategy

1. **Unit tests** - Queue operations, overflow handling
2. **Integration test** - Three dummy sensors on one bus, verify serialization
3. **Hardware validation** - Logic analyzer capture of SPI bus with shared sensors
4. **Performance test** - Measure IMU timestamp jitter before/after
5. **Stress test** - Simultaneous DRDY from all sensors, verify no lost samples

## Estimated Effort

- **SpiBus implementation**: 2-3 hours
- **I2cBus implementation**: 2 hours (simpler, reference SpiBus)
- **Unit tests**: 2 hours
- **Migrate one sensor (baro)**: 1 hour
- **Migrate remaining sensors**: 3-4 hours
- **Integration testing**: 2-3 hours
- **Total**: ~12-15 hours

## References

- Current implementation: [Spi.h](../boards/varmint_h7/common/drivers/Spi.h)
- Callback routing: [Callbacks.cpp:94-101](../boards/varmint_h7/varmint_11X/specific/Callbacks.cpp#L94-L101)
- Polling coordination: [Callbacks.cpp:61-62](../boards/varmint_h7/varmint_11X/specific/Callbacks.cpp#L61-L62)
- Driver base class pattern: [ImuDriver.h](../boards/varmint_h7/common/drivers/imu/ImuDriver.h), [GpsDriver.h](../boards/varmint_h7/common/drivers/gps/GpsDriver.h)
