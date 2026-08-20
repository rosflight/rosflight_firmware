/**
 ******************************************************************************
 * File     : stm32_h7.h
 * Date     : Sep 27, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023, AeroVironment, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3.Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 **/

#ifndef STM32_H7_HPP_
#define STM32_H7_HPP_

#include "BoardConfig.h"

#include "Adc.h"
#include "Adis165xx.h"
#include "Auav.h"
#include "Bmi088.h"
#include "DlhrL20G.h"
#include "Dps310.h"
#include "Iis2mdc.h"
#include "Ist8308.h"
#include "Mcp4017.h"
#include "Ms4525.h"
#include "Pwm.h"
#include "Sbus.h"
#include "Sd.h"
#include "Telem.h"
#include "Ubx.h"
#include "Vcp.h"
#include "interface/board.h"

/*
 *
 */

class STM32H7Board : public rosflight_firmware::Board
{
  /**
     * \brief
     *
     *
     */
private:
  static constexpr uint32_t POLL_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t EXTI_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t SPI_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t I2C_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t ADC_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t CDC_CLIENTS_MAX_LEN = 4;
  static constexpr uint32_t SD_CLIENTS_MAX_LEN = 4;
  static constexpr uint32_t UART_RXCPLT_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t UART_RXISR_CLIENTS_MAX_LEN = 16;
  static constexpr uint32_t UART_TXCPLT_CLIENTS_MAX_LEN = 16;

  struct PollClient
  {
    void (*callback)(void * context, uint64_t poll_counter);
    void * context;
    int32_t phase_offset;
  };

  struct ExtiClient
  {
    bool (*matches)(void * context, uint16_t exti_pin);
    void (*callback)(void * context);
    void * context;
  };

  struct SpiClient
  {
    bool (*matches)(void * context, SPI_HandleTypeDef * hspi);
    void (*callback)(void * context);
    void * context;
  };

  struct I2cClient
  {
    bool (*matches)(void * context, I2C_HandleTypeDef * hi2c);
    void (*callback)(void * context);
    void * context;
  };

  struct AdcClient
  {
    bool (*matches)(void * context, ADC_HandleTypeDef * hadc);
    void (*callback)(void * context, ADC_HandleTypeDef * hadc);
    void * context;
  };

  struct UartClient
  {
    bool (*matches)(void * context, UART_HandleTypeDef * huart);
    void (*callback)(void * context);
    void * context;
  };

  struct CdcClient
  {
    bool (*matches)(void * context, uint8_t chan);
    void (*receive_callback)(void * context, uint8_t * buffer, uint16_t size);
    void (*transmit_cplt_callback)(void * context);
    void * context;
  };


  struct SdClient
  {
    bool (*matches)(void * context, SD_HandleTypeDef * hsd);
    void (*txcplt_callback)(void * context);
    void (*rxcplt_callback)(void * context);
    void * context;
  };
  uint32_t serial_device_;
  uint32_t sensor_errors_ = 0;
  uint32_t status_len_ = 0;
  uint32_t poll_client_len_ = 0;
  uint32_t exti_client_len_ = 0;
  uint32_t spi_client_len_ = 0;
  uint32_t i2c_client_len_ = 0;
  uint32_t adc_client_len_ = 0;
  uint32_t cdc_client_len_ = 0;
  uint32_t sd_client_len_ = 0;
  uint32_t uart_rxcplt_client_len_ = 0;
  uint32_t uart_rxisr_client_len_ = 0;
  uint32_t uart_txcplt_client_len_ = 0;
  Status * status_list_[STATUS_LIST_MAX_LEN];
  PollClient poll_clients_[POLL_CLIENTS_MAX_LEN] = {};
  ExtiClient exti_clients_[EXTI_CLIENTS_MAX_LEN] = {};
  SpiClient spi_clients_[SPI_CLIENTS_MAX_LEN] = {};
  I2cClient i2c_clients_[I2C_CLIENTS_MAX_LEN] = {};
  AdcClient adc_clients_[ADC_CLIENTS_MAX_LEN] = {};
  CdcClient cdc_clients_[CDC_CLIENTS_MAX_LEN] = {};
  SdClient sd_clients_[SD_CLIENTS_MAX_LEN] = {};
  UartClient uart_rxcplt_clients_[UART_RXCPLT_CLIENTS_MAX_LEN] = {};
  UartClient uart_rxisr_clients_[UART_RXISR_CLIENTS_MAX_LEN] = {};
  UartClient uart_txcplt_clients_[UART_TXCPLT_CLIENTS_MAX_LEN] = {};

  RcPacket rcPacket_;

  template<typename T>
  static void poll_client_callback(void * context, uint64_t poll_counter)
  {
    static_cast<T *>(context)->poll(poll_counter);
  }

  template<typename T>
  static bool exti_client_matches(void * context, uint16_t exti_pin)
  {
    return static_cast<T *>(context)->isMy(exti_pin);
  }

  template<typename T>
  static void exti_client_callback(void * context)
  {
    static_cast<T *>(context)->extiCallback();
  }

  template<typename T>
  static bool spi_client_matches(void * context, SPI_HandleTypeDef * hspi)
  {
    return static_cast<T *>(context)->isMy(hspi);
  }

  template<typename T>
  static void spi_client_callback(void * context)
  {
    static_cast<T *>(context)->spiTxRxCpltCallback();
  }

  template<typename T>
  static bool i2c_client_matches(void * context, I2C_HandleTypeDef * hi2c)
  {
    return static_cast<T *>(context)->isMy(hi2c);
  }

  template<typename T>
  static void i2c_client_callback(void * context)
  {
    static_cast<T *>(context)->i2cMasterRxCpltCallback();
  }

  template<typename T>
  static bool adc_client_matches(void * context, ADC_HandleTypeDef * hadc)
  {
    return static_cast<T *>(context)->isMy(hadc);
  }

  template<typename T>
  static void adc_client_callback(void * context, ADC_HandleTypeDef * hadc)
  {
    static_cast<T *>(context)->adcConvCpltCallback(hadc);
  }

  template<typename T>
  static bool cdc_client_matches(void * context, uint8_t chan)
  {
    return static_cast<T *>(context)->isMy(chan);
  }

  template<typename T>
  static void cdc_receive_client_callback(void * context, uint8_t * buffer, uint16_t size)
  {
    static_cast<T *>(context)->cdcReceiveCallback(buffer, size);
  }

  template<typename T>
  static void cdc_transmit_cplt_client_callback(void * context)
  {
    static_cast<T *>(context)->cdcTransmitCpltCallback();
  }

  template<typename T>
  static bool sd_client_matches(void * context, SD_HandleTypeDef * hsd)
  {
    return static_cast<T *>(context)->isMy(hsd);
  }

  template<typename T>
  static void sd_txcplt_client_callback(void * context)
  {
    static_cast<T *>(context)->sdTxCpltCallback();
  }

  template<typename T>
  static void sd_rxcplt_client_callback(void * context)
  {
    static_cast<T *>(context)->sdRxCpltCallback();
  }

  template<typename T>
  static bool uart_client_matches(void * context, UART_HandleTypeDef * huart)
  {
    return static_cast<T *>(context)->isMy(huart);
  }

  template<typename T>
  static void uart_rxcplt_client_callback(void * context)
  {
    static_cast<T *>(context)->uartRxCpltCallback();
  }

  template<typename T>
  static void uart_rxisr_client_callback(void * context)
  {
    static_cast<T *>(context)->uartRxIsrCallback();
  }

  template<typename T>
  static void uart_txcplt_client_callback(void * context)
  {
    static_cast<T *>(context)->uartTxCpltCallback();
  }

  void register_poll_client(
    void * context, void (*callback)(void * context, uint64_t poll_counter), int32_t phase_offset);
  void register_exti_client(
    void * context, bool (*matches)(void * context, uint16_t exti_pin), void (*callback)(void * context));
  void register_spi_client(
    void * context, bool (*matches)(void * context, SPI_HandleTypeDef * hspi), void (*callback)(void * context));
  void register_i2c_client(
    void * context, bool (*matches)(void * context, I2C_HandleTypeDef * hi2c), void (*callback)(void * context));
  void register_adc_client(
    void * context, bool (*matches)(void * context, ADC_HandleTypeDef * hadc),
    void (*callback)(void * context, ADC_HandleTypeDef * hadc));
  void register_cdc_client(
    void * context, bool (*matches)(void * context, uint8_t chan),
    void (*receive_callback)(void * context, uint8_t * buffer, uint16_t size),
    void (*transmit_cplt_callback)(void * context));
  void register_sd_client(
    void * context, bool (*matches)(void * context, SD_HandleTypeDef * hsd), void (*txcplt_callback)(void * context),
    void (*rxcplt_callback)(void * context));
  void register_uart_rxcplt_client(
    void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context));
  void register_uart_rxisr_client(
    void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context));
  void register_uart_txcplt_client(
    void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context));

public:
  STM32H7Board() {};

  INTERFACE_LIST

  Status * status(uint32_t n) { return status_list_[n]; }
  uint32_t status_len(void) { return status_len_; }
  void clear_poll_clients();
  void clear_exti_clients();
  void clear_spi_clients();
  void clear_i2c_clients();
  void clear_adc_clients();
  void clear_cdc_clients();
  void clear_sd_clients();
  void clear_uart_rxcplt_clients();
  void clear_uart_rxisr_clients();
  void clear_uart_txcplt_clients();

  template<typename T>
  void register_poll_client(T * driver, int32_t phase_offset = 0)
  {
    register_poll_client(static_cast<void *>(driver), &poll_client_callback<T>, phase_offset);
  }

  template<typename T>
  void register_exti_client(T * driver)
  {
    register_exti_client(static_cast<void *>(driver), &exti_client_matches<T>, &exti_client_callback<T>);
  }

  template<typename T>
  void register_spi_client(T * driver)
  {
    register_spi_client(static_cast<void *>(driver), &spi_client_matches<T>, &spi_client_callback<T>);
  }

  template<typename T>
  void register_i2c_client(T * driver)
  {
    register_i2c_client(static_cast<void *>(driver), &i2c_client_matches<T>, &i2c_client_callback<T>);
  }

  template<typename T>
  void register_adc_client(T * driver)
  {
    register_adc_client(static_cast<void *>(driver), &adc_client_matches<T>, &adc_client_callback<T>);
  }

  template<typename T>
  void register_cdc_client(T * driver)
  {
    register_cdc_client(static_cast<void *>(driver), &cdc_client_matches<T>, &cdc_receive_client_callback<T>,
                        &cdc_transmit_cplt_client_callback<T>);
  }


  template<typename T>
  void register_sd_client(T * driver)
  {
    register_sd_client(static_cast<void *>(driver), &sd_client_matches<T>, &sd_txcplt_client_callback<T>,
                       &sd_rxcplt_client_callback<T>);
  }
  template<typename T>
  void register_uart_rxcplt_client(T * driver)
  {
    register_uart_rxcplt_client(static_cast<void *>(driver), &uart_client_matches<T>, &uart_rxcplt_client_callback<T>);
  }

  template<typename T>
  void register_uart_rxisr_client(T * driver)
  {
    register_uart_rxisr_client(static_cast<void *>(driver), &uart_client_matches<T>, &uart_rxisr_client_callback<T>);
  }

  template<typename T>
  void register_uart_txcplt_client(T * driver)
  {
    register_uart_txcplt_client(static_cast<void *>(driver), &uart_client_matches<T>, &uart_txcplt_client_callback<T>);
  }

  void dispatch_poll(uint64_t poll_counter);
  void dispatch_exti(uint16_t exti_pin);
  void dispatch_spi(SPI_HandleTypeDef * hspi);
  void dispatch_i2c(I2C_HandleTypeDef * hi2c);
  void dispatch_adc(ADC_HandleTypeDef * hadc);
  void dispatch_cdc_receive(uint8_t chan, uint8_t * buffer, uint16_t size);
  void dispatch_cdc_transmit_cplt(uint8_t chan);
  void dispatch_sd_txcplt(SD_HandleTypeDef * hsd);
  void dispatch_sd_rxcplt(SD_HandleTypeDef * hsd);
  void dispatch_uart_rxcplt(UART_HandleTypeDef * huart);
  void dispatch_uart_rxisr(UART_HandleTypeDef * huart);
  void dispatch_uart_txcplt(UART_HandleTypeDef * huart);
  ////////////////////////////////////////////////////////////////////////////////
  // Required ROSflight Board HAL functions:

  // setup

  void init_board(void) override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // serial
  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t * src, size_t len, uint8_t qos) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

  // sensors
  void sensors_init() override;
  uint16_t sensors_errors_count() override;
  uint16_t sensors_init_message_count() override;
  uint16_t sensors_init_message(char * message, uint16_t size, uint16_t i) override;
  bool sensors_init_message_good(uint16_t i) override;

  bool imu_read(rosflight_firmware::ImuStruct * imu) override;

  bool mag_read(rosflight_firmware::MagStruct * mag) override;

  bool baro_read(rosflight_firmware::PressureStruct * baro) override;

  bool diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure) override;

  bool range_read(rosflight_firmware::RangeStruct * range) override;

  // Battery
  bool battery_read(rosflight_firmware::BatteryStruct * bat) override;
  void battery_voltage_set_multiplier(double multiplier) override;
  void battery_current_set_multiplier(double multiplier) override;

  // GNSS
  bool gnss_read(rosflight_firmware::GnssStruct * gnss) override;

  // RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_read(rosflight_firmware::RcStruct * rc) override;

  // PWM
  void pwm_init(const float * rate, uint32_t channels) override;
  void pwm_disable() override;
  void pwm_write(float * value, uint32_t channels) override;

  // non-volatile memory
  void memory_init() override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;

  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

  // Backup memory
  void backup_memory_init() override;
  bool backup_memory_read(void * dest, size_t len) override;
  void backup_memory_write(const void * src, size_t len) override;
  void backup_memory_clear(size_t len) override;
};

#endif /* STM32_H7_HPP_ */
