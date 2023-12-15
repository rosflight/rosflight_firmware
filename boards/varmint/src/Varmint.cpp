/**
 ******************************************************************************
 * File     : varmint.cpp
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
#include <BoardConfig.h>
#include <Callbacks.h>
#include <CubeMX.h>
#include <Spi.h>
#include <Time64.h>
#include <Varmint.h>
#include <mavlink.h>
#include <misc.h>
#include <rosflight.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>
#include <util.h>

#include <ctime>

bool verbose = true;

// #ifdef __cplusplus
// extern "C" {
// #endif
//
// void __stack_chk_fail(void);
//
// #ifdef __cplusplus
// }
// #endif
//
//
// void *__stack_chk_guard = (void *)0xDEADD00D;
//
// void __stack_chk_fail(void)
//{
//   misc_printf("Stack smashing detected.\n");
//	while(1){}
// }

Varmint varmint;
Time64 time64;

static uint32_t init_poll_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t channel);

uint32_t init_poll_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t channel)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = 199;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = POLLING_PERIOD_US;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    return DRIVER_HAL_ERROR;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    return DRIVER_HAL_ERROR;

  HAL_TIM_PWM_Start(htim, channel); // (10kHz) to service polling routines
  HAL_TIM_Base_Start_IT(htim);
  return DRIVER_OK;
}

////////////////////////////////////////////////////////////////////////////////////////
//
// Varmint Board
//
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @fn void init_board(void)
 * @brief Board Initialization
 *
 */

void Varmint::init_board(void)
{
  uint32_t init_status;

  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();
  MPU_Config();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C1_Init(); // PITOT, POT, EEPROM
  // MX_I2C2_Init(); // not used
  MX_SPI1_Init(); // BMI IMU
  MX_SPI2_Init(); // MAG
  MX_SPI3_Init(); // Baro
  MX_SPI4_Init(); // ADIS IMU
                  //  MX_TIM1_Init(); // PWM
                  //  MX_TIM3_Init(); // PWM
                  //  MX_TIM4_Init(); // PWM

  // MX_TIM5_Init(); // Time64, initialized elsewhere
  // MX_TIM8_Init(); // Time64, initialized elsewhere
  // MX_TIM7_Init(); // Poll, initialized elsewhere
  // MX_TIM12_Init(); // ADIS16500 Ext Clock, initialized elsewhere
  // MX_USART1_UART_Init(); // uBlox, initialized elsewhere
  MX_USART2_UART_Init(); // Telem &| Console, re-initialized elsewhere
                         // MX_USART3_UART_Init(); // S.Bus, initialized elsewhere
                         // MX_ADC1_Init(); // Battery and power supplies, initialized elsewhere
                         // MX_ADC3_Init(); // STM Temperature, initialized elsewhere
                         // MX_USB_DEVICE_Init(); // initialized elsewhere
                         // MX_FDCAN1_Init(); // not used
                         // MX_SDMMC1_SD_Init(); // initialized elsewhere
  // MX_RTC_Init(); // not used
  // MX_RNG_Init(); // not used
  // MX_CRC_Init(); // not used

  //// Startup Chained Timestamp Timers 1us rolls over in 8.9 years.
  misc_printf("\n\rStart Timestamp Timer\n\r");
  time64.init(HTIM_LOW, HTIM_LOW_INSTANCE, HTIM_HIGH, HTIM_HIGH_INSTANCE);

#define ASCII_ESC 27
  misc_printf("%c[H", ASCII_ESC);  // home
  misc_printf("%c[2J", ASCII_ESC); // clear screen

  misc_printf("\n\rStarted Timestamp Timer\n\r");

  // Zero GPIO outputs used as probes. // TODO remove these eventually and return the pins to their normal functions.
  PE5_LO;  // J000_JETSON_DRDY
  PE3_LO;  // J105_2_SPI4_EXT_CS
  PB0_LO;  // TP5
  PC7_LO;  // TP6
  PH1_LO;  // J105 pin 18 RST
  PB1_HI;  // J105 pin 23/25 Sync Bus Tx
  PB15_LO; // J105_2_SPI4_EXT_CLK

  misc_printf("\n\r\n\rADIS165xx (imu0) Initialization\n\r");
  init_status = imu0_.init(IMU0_HZ, IMU0_DRDY_PORT, IMU0_DRDY_PIN,                            // Driver
                           IMU0_SPI, IMU0_CS_PORT, IMU0_CS_PIN,                               // SPI
                           IMU0_RESET_PORT, IMU0_RESET_PIN,                                   // Reset Pin
                           IMU0_HTIM, IMU0_TIM_INSTANCE, IMU0_TIM_CHANNEL, IMU0_TIM_PERIOD_US // ADIS external clock
  );
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rBMI088 (imu1) Initialization\n\r");
  init_status = imu1_.init(IMU1_HZ, IMU1_DRDY_PORT, IMU1_DRDY_PIN, IMU1_SPI, IMU1_CS_PORT_A, IMU1_CS_PIN_A,
                           IMU1_CS_PORT_G, IMU1_CS_PIN_G, IMU1_RANGE_A, IMU1_RANGE_G);
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rDLHRL20G (pitot) Initialization\n\r");
  init_status = pitot_.init(PITOT_HZ, PITOT_DRDY_PORT, PITOT_DRDY_PIN, // Driver
                            PITOT_I2C, PITOT_I2C_ADDRESS               // I2C
  );
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rDPS310 (bar0) Initialization\n\r");
  init_status = baro_.init(BARO_HZ, BARO_DRDY_PORT, BARO_DRDY_PIN, // Driver
                           BARO_SPI, BARO_CS_PORT, BARO_CS_PIN     // SPI
  );
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rIIS2MDC (mag) Initialization\n\r");
  init_status = mag_.init(MAG_HZ, MAG_DRDY_PORT, MAG_DRDY_PIN, // Driver
                          MAG_SPI, MAG_CS_PORT, MAG_CS_PIN     // SPI
  );
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rUbx (gps) Initialization\n\r");
  init_status = gps_.init(GPS_HZ, GPS_PPS_PORT, GPS_PPS_PIN, GPS_UART, GPS_UART_INSTANCE, GPS_UART_DMA, GPS_BAUD);
  misc_exit_status(init_status);

  misc_printf("\n\r\nS.Bus (rc) Initialization\n\r");
  init_status = rc_.init(RC_HZ, RC_UART, RC_UART_INSTANCE, RC_UART_DMA, RC_BAUD);
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rAdc (adc) Initialization\n\r");
  init_status =
      adc_.init(ADC_HZ, ADC_ADC_EXTERNAL, ADC_ADC_INSTANCE_EXTERNAL, ADC_ADC_INTERNAL, ADC_ADC_INSTANCE_INTERNAL);
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rVcp (vcp) Initialization\n\r");
  init_status = vcp_.init(VCP_HZ);
  misc_exit_status(init_status);

  misc_printf("\n\r\n\rTelem (telem) Initialization\n\r");
  init_status = telem_.init(TELEM_HZ, TELEM_UART, TELEM_UART_INSTANCE, TELEM_UART_DMA, TELEM_BAUD, RxIsrCallback);
  misc_exit_status(init_status);

  //// Initialize PWM Timers
  misc_printf("\n\r\n\rPWM (PWM) Initialization\n\r");
  init_status = pwm_init_timers(SERVO_PWM_PERIOD);
  init_status |= pwm_[0].init(PWM_HTIM_0, PWM_CHAN_0, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[1].init(PWM_HTIM_1, PWM_CHAN_1, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[2].init(PWM_HTIM_2, PWM_CHAN_2, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[3].init(PWM_HTIM_3, PWM_CHAN_3, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[4].init(PWM_HTIM_4, PWM_CHAN_4, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[5].init(PWM_HTIM_5, PWM_CHAN_5, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[6].init(PWM_HTIM_6, PWM_CHAN_6, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[7].init(PWM_HTIM_7, PWM_CHAN_7, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[8].init(PWM_HTIM_8, PWM_CHAN_8, PWM_MIN, PWM_CENTER, PWM_MAX);
  init_status |= pwm_[9].init(PWM_HTIM_9, PWM_CHAN_9, PWM_MIN, PWM_CENTER, PWM_MAX);
  misc_exit_status(init_status);

  ////// Initialize SD Card
  misc_printf("\n\r\n\rSDMMC Initialization\n\r");
  init_status = sd_.init(SD_HSD, SD_HSD_INSTANCE);
  misc_exit_status(init_status);

  //// Start the Periodic Polling Timer
  // High Rate Polling Timer
  misc_printf("\n\r\n\rPolling Timer Initialization\n\r");
  init_status = init_poll_timer(POLL_HTIM, POLL_HTIM_INSTANCE, POLL_TIM_CHANNEL);
  misc_exit_status(init_status);

  //// Enable EXTI IRQ's
  misc_printf("\n\r\n\rSet-up EXTI IRQ's\n\r");

  verbose = false;

  HAL_NVIC_EnableIRQ(EXTI3_IRQn);     // uBlox GPS PPS
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);   // ADIS IMU DRDY
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Bosh IMU DRDY

  __HAL_UART_ENABLE_IT(gps_.huart(), UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(rc_.huart(), UART_IT_IDLE);

  telem_.rxStart(); // Also enables interrupts.
}

void Varmint::board_reset(bool bootloader)
{
  //	pwm_disable();
  //	HAL_NVIC_SystemReset();
}

// clock
uint32_t Varmint::clock_millis()
{
  return time64.Us() / 1000;
}
uint64_t Varmint::clock_micros()
{
  return time64.Us();
}
void Varmint::clock_delay(uint32_t ms)
{
  time64.dMs(ms);
}

// serial comms to the Companion computer
void Varmint::serial_init(uint32_t baud_rate, uint32_t dev)
{
  serial_device_ = 1; // dev;// 1 = serial, otherwise = VCP
  if (dev == 1)
    telem_.reset_baud(baud_rate);
}

void Varmint::serial_write(const uint8_t *src, size_t len)
{
  SerialTxPacket p;
  p.timestamp = time64.Us();
  p.payloadSize = len;

  if (len > (256 + 8))
    len = (256 + 8);
  memcpy(p.payload, src, len);

  uint8_t message_id = p.payload[5]; // Assumes Mavlink v1

  switch (message_id)
  {
  // Real-time
  case MAVLINK_MSG_ID_SMALL_IMU:
    p.qos = 0x00;
    break; // Start of Timeframe
  case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
  case MAVLINK_MSG_ID_TIMESYNC:
    p.qos = 0x01;
    break;
  // Secondary real-time
  case MAVLINK_MSG_ID_HEARTBEAT:
  case MAVLINK_MSG_ID_SMALL_BARO:
  case MAVLINK_MSG_ID_DIFF_PRESSURE:
  case MAVLINK_MSG_ID_ROSFLIGHT_GNSS:
  case MAVLINK_MSG_ID_ROSFLIGHT_GNSS_FULL:
  case MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS:
  case MAVLINK_MSG_ID_SMALL_MAG:
  case MAVLINK_MSG_ID_RC_CHANNELS:
  case MAVLINK_MSG_ID_SMALL_RANGE:
  case MAVLINK_MSG_ID_ROSFLIGHT_OUTPUT_RAW:
    //		case MAVLINK_MSG_ID_ROSFLIGHT_STATUS:
  case MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD:
    p.qos = 0x02;
    break;
  // Otherwise send if there is bandwidth available
  default:
    p.qos = 0xFF;
  }

  if (serial_device_ == 1)
    telem_.writePacket(&p);
  else
    vcp_.writePacket(&p);
}

uint16_t Varmint::serial_bytes_available(void)
{
  if (serial_device_ == 1)
    return telem_.byteCount();
  else
    return vcp_.byteCount();
}

uint8_t Varmint::serial_read(void)
{
  uint8_t c = 0;
  if (serial_device_ == 1)
    telem_.readByte(&c);
  else
    vcp_.readByte(&c);
  return c;
}

void Varmint::serial_flush(void)
{
  // do nothing
}

// sensors

void Varmint::sensors_init() {}

uint16_t Varmint::num_sensor_errors()
{
  return 0;
}

// IMU
bool Varmint::imu_has_new_data()
{
  return imu0_.rxFifoCount() > 0;
}
bool Varmint::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
  ImuPacket p;
  if (imu0_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    accel[0] = p.accel[0];
    accel[1] = p.accel[1];
    accel[2] = p.accel[2];
    gyro[0] = p.gyro[0];
    gyro[1] = p.gyro[1];
    gyro[2] = p.gyro[2];
    *temperature = p.temperature;
    *time_us = p.drdy;
    return true;
  }
  return false;
}
void Varmint::imu_not_responding_error()
{
  // do nothing for now.
}

// MAG
bool Varmint::mag_present()
{
  return true;
}
bool Varmint::mag_has_new_data()
{
  return mag_.rxFifoCount() > 0;
}

bool Varmint::mag_read(float mag[3])
{
  MagPacket p;
  if (mag_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    mag[0] = p.flux[0];
    mag[1] = p.flux[1];
    mag[2] = p.flux[2];
    return true;
  }
  return false;
}

// Baro
bool Varmint::baro_present()
{
  return true;
}
bool Varmint::baro_has_new_data()
{
  return baro_.rxFifoCount() > 0;
}
bool Varmint::baro_read(float *pressure, float *temperature)
{
  BaroPacket p;
  if (baro_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    *pressure = p.pressure;
    *temperature = p.temperature;
    return true;
  }
  return false;
}

// Pitot
bool Varmint::diff_pressure_present()
{
  return true;
}
bool Varmint::diff_pressure_has_new_data()
{
  return pitot_.rxFifoCount() > 0;
}

bool Varmint::diff_pressure_read(float *diff_pressure, float *temperature)
{
  PitotPacket p;
  if (pitot_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    *diff_pressure = p.pressure;
    *temperature = p.temperature;
    return true;
  }
  return false;
}

// Sonar
bool Varmint::sonar_present()
{
  return false;
}
bool Varmint::sonar_has_new_data()
{
  return false;
}
bool Varmint::sonar_read(float *range)
{
  return false;
}

// Battery
bool Varmint::battery_present()
{
  return true;
}
bool Varmint::battery_has_new_data()
{
  return adc_.rxFifoCount() > 0;
}
bool Varmint::battery_read(float *voltage, float *current)
{
  AdcPacket p;
  if (adc_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    *current = p.volts_ext[ADC_BATTERY_CURR];
    *voltage = p.volts_ext[ADC_BATTERY_VOLTS];
    return true;
  }
  return false;
}
void Varmint::battery_voltage_set_multiplier(double multiplier)
{
  if (multiplier == 0)
    return;
  adc_.setScaleFactor(ADC_BATTERY_VOLTS, multiplier);
}
void Varmint::battery_current_set_multiplier(double multiplier)
{
  if (multiplier == 0)
    return;
  adc_.setScaleFactor(ADC_BATTERY_CURR, multiplier);
}

// GNSS
bool Varmint::gnss_present()
{
  return true;
}
bool Varmint::gnss_has_new_data()
{
  return gps_.rxFifoCount() > 0;
}

bool Varmint::gnss_read(rosflight_firmware::GNSSData *gnss, rosflight_firmware::GNSSFull *gnss_full)
{
  UbxPacket p;

  if (gps_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    gnss_full->time_of_week = p.pvt.iTOW;
    gnss_full->year = p.pvt.year;
    gnss_full->month = p.pvt.month;
    gnss_full->day = p.pvt.day;
    gnss_full->hour = p.pvt.hour;
    gnss_full->min = p.pvt.min;
    gnss_full->sec = p.pvt.sec;
    gnss_full->valid = p.pvt.valid;
    gnss_full->t_acc = p.pvt.tAcc;
    gnss_full->nano = p.pvt.nano;
    // gnss_full->fix_type 		= p.pvt.fixType; 	// This is the Ubx fix type, not rosflight
    if ((p.pvt.fixType > 1) && (p.pvt.fixType < 5))
      gnss_full->fix_type = rosflight_firmware::GNSS_FIX_TYPE_FIX;
    else
      gnss_full->fix_type = rosflight_firmware::GNSS_FIX_TYPE_NO_FIX;
    gnss_full->num_sat = p.pvt.numSV;
    gnss_full->lon = p.pvt.lon;
    gnss_full->lat = p.pvt.lat;
    gnss_full->height = p.pvt.height;
    gnss_full->height_msl = p.pvt.hMSL;
    gnss_full->h_acc = p.pvt.hAcc;
    gnss_full->v_acc = p.pvt.vAcc;
    gnss_full->vel_n = p.pvt.velN;
    gnss_full->vel_e = p.pvt.velE;
    gnss_full->vel_d = p.pvt.velD;
    gnss_full->g_speed = p.pvt.gSpeed;
    gnss_full->head_mot = p.pvt.headMot;
    gnss_full->s_acc = p.pvt.sAcc;
    gnss_full->head_acc = p.pvt.headAcc;
    gnss_full->p_dop = p.pvt.pDOP;
    gnss_full->rosflight_timestamp = p.drdy;

    gnss->time_of_week = p.nav.iTOW;
    gnss->nanos = p.nav.fTOW; // TODO: Is this supposed to be the nanoseconds of the TOW, unix time???
    // gnss->fix_type				= nav.gpsFix;
    if ((p.nav.fixType > 1) && (p.nav.fixType < 5))
      gnss->fix_type = rosflight_firmware::GNSS_FIX_TYPE_FIX;
    else
      gnss->fix_type = rosflight_firmware::GNSS_FIX_TYPE_NO_FIX;
    struct tm tm;
    tm.tm_sec = p.pvt.sec;
    tm.tm_min = p.pvt.min;
    tm.tm_hour = p.pvt.hour;
    tm.tm_mday = p.pvt.day;
    tm.tm_mon = p.pvt.month - 1;
    tm.tm_year = p.pvt.year - 1900;
    gnss->time = mktime(&tm);
    gnss->lat = p.pvt.lat;
    gnss->lon = p.pvt.lon;
    gnss->height = p.pvt.height;
    gnss->vel_n = p.pvt.velN;
    gnss->vel_e = p.pvt.velE;
    gnss->vel_d = p.pvt.velD;
    gnss->h_acc = p.pvt.hAcc;
    gnss->v_acc = p.pvt.vAcc;

    gnss->ecef.x = p.nav.ecefX;
    gnss->ecef.y = p.nav.ecefY;
    gnss->ecef.z = p.nav.ecefZ;
    gnss->ecef.p_acc = p.nav.pAcc;
    gnss->ecef.vx = p.nav.ecefVX;
    gnss->ecef.vy = p.nav.ecefVY;
    gnss->ecef.vz = p.nav.ecefVZ;
    gnss->ecef.s_acc = p.nav.sAcc;
    gnss->rosflight_timestamp = p.drdy;
    return true;
  }

  return false;
}

// RC
void Varmint::rc_init(rc_type_t rc_type){};
bool Varmint::rc_lost()
{
  return rc_.lol();
}

bool Varmint::rc_has_new_data()
{
  return rc_.rxFifoCount() > 0;
}

float Varmint::rc_read(uint8_t chan)
{
  static RcPacket p;
  static bool ever_read = false;

  if (rc_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
  {
    ever_read = true;
  }

  if ((chan < PWM_CHANNELS) & ever_read)
    return p.chan[chan];
  return -1; // out of range or no data in p
}

// PWM
void Varmint::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (int i = 0; i < PWM_CHANNELS; i++) pwm_[i].disable();
  for (int i = 0; i < PWM_CHANNELS; i++) pwm_[i].set_rate(refresh_rate);
  for (int i = 0; i < PWM_CHANNELS; i++) pwm_[i].enable();
}
void Varmint::pwm_disable(void)
{
  for (int i = 0; i < PWM_CHANNELS; i++) pwm_[i].disable();
}
void Varmint::pwm_write(uint8_t channel, float value)
{
  pwm_[channel].write(value);
}

// LEDs
void Varmint::led0_on() // Red LED
{
  RED_HI;
}
void Varmint::led0_off()
{
  RED_LO;
}
void Varmint::led0_toggle()
{
  RED_TOG;
}

void Varmint::led1_on()
{
  BLU_HI;
}
void Varmint::led1_off()
{
  BLU_LO;
}
void Varmint::led1_toggle()
{
  BLU_TOG;
}

// Backup Data (Register and SRAM)
// https://stackoverflow.com/questions/20667754/how-to-use-backup-sram-as-eeprom-in-stm32f4
// from \Drivers\CMSIS\Device\ST\STM32H7xx\Include\stm32h753xx.h(2141)
// #define D3_BKPSRAM_BASE           (0x38800000UL) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge */
// #define D3_SRAM_BASE              (0x38000000UL) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge

#define D3_BKPSRAM_BASE_LEN (4096U)
#define D3_SRAM_BASE_LEN (65536U)
void Varmint::backup_memory_init() {}
bool Varmint::backup_memory_read(void *dest, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy(dest, (void*)D3_BKPSRAM_BASE, len);
  //	HAL_PWR_DisableBkUpAccess();
  return true;
}
void Varmint::backup_memory_write(const void *src, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy((void*)D3_BKPSRAM_BASE, src, len);
  //	HAL_PWR_DisableBkUpAccess();
}
void Varmint::backup_memory_clear(size_t len)
{
  //	HAL_PWR_EnableBkUpAccess();
  //	memset((void*)D3_BKPSRAM_BASE, 0, D3_BKPSRAM_BASE_LEN);
  //	HAL_PWR_DisableBkUpAccess();
}

void Varmint::memory_init() {} // do nothing

bool Varmint::memory_read(void *dest, size_t len)
{
  return sd_.read((uint8_t *)dest, len);
}
bool Varmint::memory_write(const void *src, size_t len)
{
  return sd_.write((uint8_t *)src, len);
}

uint32_t Varmint::pwm_init_timers(uint32_t servo_pwm_period_us)
{
  {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = (SERVO_PWM_CLK_DIV);
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = servo_pwm_period_us;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (SERVO_PWM_CENTER);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    HAL_TIM_MspPostInit(&htim1);
  }
  {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (SERVO_PWM_CLK_DIV);
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = servo_pwm_period_us;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (SERVO_PWM_CENTER);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
      return DRIVER_HAL_ERROR;
    HAL_TIM_MspPostInit(&htim3);
  }
  {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (SERVO_PWM_CLK_DIV);
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = servo_pwm_period_us;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (SERVO_PWM_CENTER);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
      return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
      return DRIVER_HAL_ERROR;
    HAL_TIM_MspPostInit(&htim4);
  }
  return DRIVER_OK;
}
