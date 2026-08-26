$ErrorActionPreference = 'Stop'

function Replace-Exact {
  param(
    [string]$Path,
    [string]$Old,
    [string]$New
  )
  $text = Get-Content -Raw -Path $Path
  if (-not $text.Contains($Old)) {
    throw "Pattern not found in $Path"
  }
  $text = $text.Replace($Old, $New)
  Set-Content -Path $Path -Value $text
}

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Async.h' -Old @"
};

#endif /* DRIVERS_ASYNC_H_ */
"@ -New @"
};

class ExtiSignal : public AcquisitionSignal
{
public:
  void init(uint16_t exti_pin)
  {
    exti_pin_ = exti_pin;
    timestamp_us_ = 0;
  }

  bool is_my(uint16_t exti_pin) const { return exti_pin_ == exti_pin; }

  void trigger_from_irq(uint64_t timestamp_us)
  {
    timestamp_us_ = timestamp_us;
    trigger();
  }

  uint64_t timestamp_us() const { return timestamp_us_; }

private:
  uint16_t exti_pin_ = 0;
  uint64_t timestamp_us_ = 0;
};

#endif /* DRIVERS_ASYNC_H_ */
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old @"
#ifdef __cplusplus

class STM32H7Callbacks
"@ -New @"
#ifdef __cplusplus

class ExtiSignal;

class STM32H7Callbacks
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old @"
  struct ExtiClient
  {
    bool (*matches)(void * context, uint16_t exti_pin);
    void (*callback)(void * context);
    void * context;
  };
"@ -New @"
  struct ExtiClient
  {
    bool (*matches)(void * context, uint16_t exti_pin);
    void (*callback)(void * context, uint16_t exti_pin, uint64_t timestamp_us);
    void * context;
  };
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old @"
  template<typename T>
  static void exti_client_callback(void * context)
  {
    static_cast<T *>(context)->extiCallback();
  }
"@ -New @"
  template<typename T>
  static void exti_client_callback(void * context, uint16_t exti_pin, uint64_t timestamp_us)
  {
    (void) exti_pin;
    (void) timestamp_us;
    static_cast<T *>(context)->extiCallback();
  }

  static bool exti_signal_matches(void * context, uint16_t exti_pin);
  static void exti_signal_callback(void * context, uint16_t exti_pin, uint64_t timestamp_us);
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old @"
  void register_exti_client(
    void * context, bool (*matches)(void * context, uint16_t exti_pin), void (*callback)(void * context));
"@ -New @"
  void register_exti_client(void * context, bool (*matches)(void * context, uint16_t exti_pin),
                            void (*callback)(void * context, uint16_t exti_pin, uint64_t timestamp_us));
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old @"
  template<typename T>
  void register_exti_client(T * driver)
  {
    register_exti_client(static_cast<void *>(driver), &exti_client_matches<T>, &exti_client_callback<T>);
  }
"@ -New @"
  template<typename T>
  void register_exti_client(T * driver)
  {
    register_exti_client(static_cast<void *>(driver), &exti_client_matches<T>, &exti_client_callback<T>);
  }

  void register_exti_signal(ExtiSignal * signal);
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.h' -Old '  void dispatch_exti(uint16_t exti_pin);' -New '  void dispatch_exti(uint16_t exti_pin, uint64_t timestamp_us);'

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.cpp' -Old @"
#include \"Callbacks.h\"
"@ -New @"
#include \"Callbacks.h\"
#include \"Async.h\"
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.cpp' -Old @"
void STM32H7Callbacks::register_exti_client(
  void * context, bool (*matches)(void * context, uint16_t exti_pin), void (*callback)(void * context))
{
  if (exti_client_len_ >= EXTI_CLIENTS_MAX_LEN) return;

  ExtiClient & client = exti_clients_[exti_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}
"@ -New @"
bool STM32H7Callbacks::exti_signal_matches(void * context, uint16_t exti_pin)
{
  return static_cast<ExtiSignal *>(context)->is_my(exti_pin);
}

void STM32H7Callbacks::exti_signal_callback(void * context, uint16_t exti_pin, uint64_t timestamp_us)
{
  (void) exti_pin;
  static_cast<ExtiSignal *>(context)->trigger_from_irq(timestamp_us);
}

void STM32H7Callbacks::register_exti_client(
  void * context, bool (*matches)(void * context, uint16_t exti_pin),
  void (*callback)(void * context, uint16_t exti_pin, uint64_t timestamp_us))
{
  if (exti_client_len_ >= EXTI_CLIENTS_MAX_LEN) return;

  ExtiClient & client = exti_clients_[exti_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_exti_signal(ExtiSignal * signal)
{
  register_exti_client(static_cast<void *>(signal), &STM32H7Callbacks::exti_signal_matches,
                       &STM32H7Callbacks::exti_signal_callback);
}
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.cpp' -Old @"
void STM32H7Callbacks::dispatch_exti(uint16_t exti_pin)
{
  for (uint32_t i = 0; i < exti_client_len_; i++) {
    const ExtiClient & client = exti_clients_[i];
    if (!client.matches(client.context, exti_pin)) continue;
    client.callback(client.context);
  }
}
"@ -New @"
void STM32H7Callbacks::dispatch_exti(uint16_t exti_pin, uint64_t timestamp_us)
{
  for (uint32_t i = 0; i < exti_client_len_; i++) {
    const ExtiClient & client = exti_clients_[i];
    if (!client.matches(client.context, exti_pin)) continue;
    client.callback(client.context, exti_pin, timestamp_us);
  }
}
"@

Replace-Exact -Path 'boards/stm32_h7/common/Callbacks.cpp' -Old @"
void HAL_GPIO_EXTI_Callback(uint16_t exti_pin)
{
  stm32_h7_board.callbacks().dispatch_exti(exti_pin);
}
"@ -New @"
void HAL_GPIO_EXTI_Callback(uint16_t exti_pin)
{
  const uint64_t timestamp_us = time64.Us();
  stm32_h7_board.callbacks().dispatch_exti(exti_pin, timestamp_us);
}
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.h' -Old @"
  void attach_bus(SpiBus & bus) { async_bus_ = &bus; }
  void register_callbacks(STM32H7Board & board, int32_t poll_phase_offset = 0);

  void extiCallback(void);
  bool display(void);
  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  void set_rotation(double rotation[9]) { memcpy(rotation_,&rotation, 9*sizeof(double));}
"@ -New @"
  void attach_bus(SpiBus & bus) { async_bus_ = &bus; }
  void register_callbacks(STM32H7Board & board, int32_t poll_phase_offset = 0);

  bool display(void);
  void set_rotation(double rotation[9]) { memcpy(rotation_,&rotation, 9*sizeof(double));}
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.h' -Old @"
  uint16_t sampleRateHz_;
  uint64_t groupDelay_;
  uint16_t drdyPin_;
  uint64_t drdy_;

  SpiBus * async_bus_ = nullptr;
  SpiBus::Device async_device_ = {};
  AcquisitionSignal exti_signal_;
"@ -New @"
  uint16_t sampleRateHz_;
  uint64_t groupDelay_;

  SpiBus * async_bus_ = nullptr;
  SpiBus::Device async_device_ = {};
  ExtiSignal exti_signal_;
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.cpp' -Old @"
  drdyPin_ = drdy_pin;
  async_device_.cs_port = cs_port;
"@ -New @"
  exti_signal_.init(drdy_pin);
  async_device_.cs_port = cs_port;
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.cpp' -Old @"
void Adis165xx::extiCallback(void)
{
  if (async_bus_ == nullptr) {
    return;
  }

  drdy_ = time64.Us();
  exti_signal_.trigger();
}

AsyncTask<void> Adis165xx::run()
"@ -New @"
AsyncTask<void> Adis165xx::run()
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.cpp' -Old '      p.header.timestamp = drdy_-groupDelay_;' -New '      p.header.timestamp = exti_signal_.timestamp_us() - groupDelay_;'

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Adis165xx.cpp' -Old '  board.callbacks().register_exti_client(this);' -New '  board.callbacks().register_exti_signal(&exti_signal_);'

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.h' -Old @"
  void attach_bus(SpiBus & bus) { async_bus_ = &bus; }
  void register_callbacks(STM32H7Board & board, int32_t poll_phase_offset = 0);

  void extiCallback(void);
  bool display(void);

  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
"@ -New @"
  void attach_bus(SpiBus & bus) { async_bus_ = &bus; }
  void register_callbacks(STM32H7Board & board, int32_t poll_phase_offset = 0);

  bool display(void);
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.h' -Old @"
  uint16_t sampleRateHz_;
  uint64_t groupDelay_;
  uint16_t drdyPin_;
  uint64_t drdy_;
"@ -New @"
  uint16_t sampleRateHz_;
  uint64_t groupDelay_;
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.h' -Old '  AcquisitionSignal exti_signal_;' -New '  ExtiSignal exti_signal_;'

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.cpp' -Old @"
  drdyPin_ = drdy_pin;
  rangeA_ = range_a;
  rangeG_ = range_g;
  drdy_ = 0;
"@ -New @"
  exti_signal_.init(drdy_pin);
  rangeA_ = range_a;
  rangeG_ = range_g;
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.cpp' -Old @"
void Bmi088::extiCallback(void)
{
  if (async_bus_ == nullptr) {
    return;
  }

  drdy_ = time64.Us();
  exti_signal_.trigger();
}

AsyncTask<void> Bmi088::run()
"@ -New @"
AsyncTask<void> Bmi088::run()
"@

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.cpp' -Old '    p.header.timestamp = drdy_ - groupDelay_;' -New '    p.header.timestamp = exti_signal_.timestamp_us() - groupDelay_;'

Replace-Exact -Path 'boards/stm32_h7/common/sensor_drivers/Bmi088.cpp' -Old '  board.callbacks().register_exti_client(this);' -New '  board.callbacks().register_exti_signal(&exti_signal_);'
