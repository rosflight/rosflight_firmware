/**
 ******************************************************************************
 * File     : Adis165xx.cpp
 * Date     : Sep 20, 2023
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

#include <Adis165xx.h>
#include <Packets.h>
#include <Time64.h>
#include <misc.h>

#define ADIS_OK (0x0000)

#define SPI_WRITE 0x80
#define SPI_READ 0x00

#define ADIS_SPI_PAUSE_US 100 // 16->20 us between spi transactions

#define ADIS_BUFFBYTES32 34
#define ADIS_BUFFBYTES16 22

#define SPI_WRITE 0x80
#define SPI_READ 0x00

#define BURST_READ (0x68 | SPI_READ)

extern Time64 time64;

DMA_RAM uint8_t adis165xx_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t adis165xx_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t adis165xx_fifo_rx_buffer[ADIS165XX_FIFO_BUFFERS * sizeof(ImuPacket)];

uint32_t Adis165xx::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
  uint16_t drdy_pin,                                 // Reset GPIO Pin
  // SPI initializers
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Reset GPIO Port
  uint16_t cs_pin,                                  // Reset GPIO Pin
  // ADIS165xx initializers
  GPIO_TypeDef * reset_port, // Reset GPIO Port
  uint16_t reset_pin,        // Reset GPIO Pin
  TIM_HandleTypeDef * htim, TIM_TypeDef * htim_instance, uint32_t htim_channel, uint32_t htim_period_us)
{
  uint32_t status = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  spi_.init(hspi, adis165xx_dma_txbuf, adis165xx_dma_rxbuf, cs_port, cs_pin);

  timeoutMs_ = 100;

  resetPort_ = reset_port;
  resetPin_ = reset_pin;
  htim_ = htim;
  htimChannel_ = htim_channel;

  groupDelay_ = (uint64_t) 1510
    + (uint64_t) 500000 / sampleRateHz_; // us, Approximate, Accel is 1.57ms, Gyro x&y are 1.51ms, and Gyro z is 1.29ms.

  HAL_GPIO_WritePin(spi_.port_, spi_.pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(resetPort_, resetPin_, GPIO_PIN_SET);

  rxFifo_.init(ADIS165XX_FIFO_BUFFERS, sizeof(ImuPacket), adis165xx_fifo_rx_buffer);

  // Startup the external clock

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim_->Instance = htim_instance;
  htim_->Init.Prescaler = 199;
  htim_->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_->Init.Period = htim_period_us - 1;
  htim_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) return DRIVER_HAL_ERROR;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim_, &sMasterConfig) != HAL_OK) return DRIVER_HAL_ERROR;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) return DRIVER_HAL_ERROR;

  HAL_TIM_MspPostInit(htim_);

  HAL_TIM_PWM_Start(htim_, htimChannel_); //(2kHz) clock source for ADIS165xx
  time64.dUs(100);

  // Reset
  HAL_GPIO_WritePin(resetPort_, resetPin_, GPIO_PIN_RESET);
  time64.dUs(100); // was 16
  HAL_GPIO_WritePin(resetPort_, resetPin_, GPIO_PIN_SET);
  time64.dMs(350); // Data sheet specifies 255ms for power-on startup empirically 300 is required

#define ADIS16500_PROD_ID_ADDR 0x72
#define ADIS16500_PROD_ID 0x4074
  uint16_t prod_id = readRegister(ADIS16500_PROD_ID_ADDR);

  misc_printf("ADIS165xx Product ID = 0x%04X (0x4074) - ", prod_id);

  if (prod_id == ADIS16500_PROD_ID) {
    misc_printf("OK\n");
  } else {
    misc_printf("ERROR\n");
    status |= DRIVER_ID_MISMATCH;
    return status;
  }

#define ADIS16500_FILT_CTRL 0x5C // shift so we can or the data into the first 16 bit packet
  // [15:3] not used
  // [2:0] 0 no digital filter default)
  writeRegister(ADIS16500_FILT_CTRL, 0);

#define ADIS16500_DEC_RATE 0x64 // decimation
  // [15:11] don't care
  // [10:0] decimation rate minus 1, e.g., use 5-1 = 4

  uint16_t dec_rate = 2000 / sampleRateHz_ - 1;
  writeRegister(ADIS16500_DEC_RATE, dec_rate); // 00 for 2000 Hz, 2000/400-1 = 4 for 400 Hz.

  // Miscellaneous Control Register (MSC_CTRL)
#define ADIS16500_MSC_CTRL 0x60
  // [15:10] 0's unused
  // [9] 1 32-bit burst data (default = 0)
  // [8] 0 burst data has gyro and accel data (default = 0)

  // [7] 1 enable linear acceleration compensation for gyros (default  0)
  // [6] 0 point of percussion alignment
  // [5] 0 always zero
  // [4] 0 wide sensor bandwidth (default)

  // [3:2] 01 Direct Input Sync Mode
  // [1] 0 falling edge sync (default =0)
  // [0] 1 active high when data is valid (default is 0, low)
  // 0b0000 0010 1000 0101 = 0x0285

  if (sampleRateHz_ == 2000) // use 32-bit data mode
  {
    writeRegister(ADIS16500_MSC_CTRL, 0x0085); // values 0b0000 0000 1000 0101 = 0x0085
  } else                                       // use 16-bit data mode
  {
    writeRegister(ADIS16500_MSC_CTRL, 0x0285); // values 0b0000 0010 1000 0101 = 0x0285
  }

#define ADIS16500_DIAG_STAT 0x02
  uint16_t diag_stat = readRegister(ADIS16500_DIAG_STAT);
  misc_printf("ADIS165xx DIAG_STAT  = 0x%04X (0x%04X) - ", diag_stat, ADIS_OK);
  if (diag_stat == 0) {
    misc_printf("OK\n");
  } else {
    misc_printf("ERROR\n");
    status |= DRIVER_SELF_DIAG_ERROR;
  }

  return status;
}

inline double val(uint8_t * x)
{
  return (double) ((int32_t) x[0] << 8 | (int32_t) x[1] << 0 | (int32_t) x[2] << 24 | (int32_t) x[3] << 16)
    / ((double) (1 << 16));
}

bool Adis165xx::startDma(void) // called to start dma read
{
  HAL_StatusTypeDef hal_Status = HAL_OK;
  drdy_ = time64.Us();
  if (sampleRateHz_ == 2000) hal_Status = spi_.startDma(BURST_READ, ADIS_BUFFBYTES16);
  else hal_Status = spi_.startDma(BURST_READ, ADIS_BUFFBYTES32);
  return hal_Status == HAL_OK;
}

void Adis165xx::endDma(void) // called when DMA data is ready
{
  uint8_t * rx = spi_.endDma();
  if (sampleRateHz_ == 2000) {
    // compute checksum
    uint16_t sum = 0;
    for (int n = 2; n < (ADIS_BUFFBYTES16 - 2); n++) sum += (uint16_t) rx[n];

    int16_t data[ADIS_BUFFBYTES16 / 2];
    for (int i = 0; i < ADIS_BUFFBYTES16 / 2; i++)
      data[i] = (int16_t) rx[2 * i] << 8 | ((int16_t) rx[2 * i + 1] & 0x00FF);
    if (sum == data[10]) {
      ImuPacket p;
      p.timestamp = time64.Us();
      p.drdy = drdy_;
      p.groupDelay = groupDelay_;
      p.status = (uint16_t) data[1];
      p.gyro[0] = -(double) data[2] * 0.001745329251994; // rad/s, or use 0.1 deg/s
      p.gyro[1] = -(double) data[3] * 0.001745329251994; // rad/s, or use 0.1 deg/s
      p.gyro[2] = (double) data[4] * 0.001745329251994;  // rad/s, or use 0.1 deg/s
      p.accel[0] = -(double) data[5] * 0.01225;          // m/s^2
      p.accel[1] = -(double) data[6] * 0.01225;          // m/s^2
      p.accel[2] = (double) data[7] * 0.01225;           // m/s^2
      p.temperature = (double) data[8] * 0.1 + 273.15;   // K
      p.dataTime = (double) ((uint16_t) data[9]) / sampleRateHz_;
      if (p.status == ADIS_OK) rxFifo_.write((uint8_t *) &p, sizeof(p));
    }
  } else {
    // compute checksum
    uint16_t sum = 0;
    for (int n = 2; n < (ADIS_BUFFBYTES32 - 2); n++) sum += (uint16_t) rx[n];

    int16_t data[ADIS_BUFFBYTES32 / 2];
    for (int i = 0; i < ADIS_BUFFBYTES32 / 2; i++)
      data[i] = (int16_t) rx[2 * i] << 8 | ((int16_t) rx[2 * i + 1] & 0x00FF);

    if (sum == data[16]) {
      ImuPacket p;
      p.timestamp = time64.Us();
      p.drdy = drdy_;
      p.groupDelay = groupDelay_;
      p.status = (uint16_t) data[1];
      p.gyro[0] = -val(rx + 4) * 0.001745329251994;     // rad/s, or use 0.1 deg/s
      p.gyro[1] = -val(rx + 8) * 0.001745329251994;     // rad/s, or use 0.1 deg/s
      p.gyro[2] = val(rx + 12) * 0.001745329251994;     // rad/s, or use 0.1 deg/s
      p.accel[0] = -val(rx + 16) * 0.01225;             // m/s^2
      p.accel[1] = -val(rx + 20) * 0.01225;             // m/s^2
      p.accel[2] = val(rx + 24) * 0.01225;              // m/s^2
      p.temperature = (double) data[14] * 0.1 + 273.15; // K
      p.dataTime = (double) ((uint16_t) data[15]) / sampleRateHz_;
      if (p.status == ADIS_OK) rxFifo_.write((uint8_t *) &p, sizeof(p));
    }
  }
}

void Adis165xx::writeRegister(uint8_t address, uint16_t value)
{
  uint8_t tx[2] = {0};
  tx[0] = (address) | SPI_WRITE;
  tx[1] = value & 0x00FF;

  spi_.tx(tx, 2, timeoutMs_);
  time64.dUs(ADIS_SPI_PAUSE_US);
  tx[0] = (++address) | SPI_WRITE;
  tx[1] = (value >> 8) & 0x00FF;
  spi_.tx(tx, 2, timeoutMs_);
  time64.dUs(ADIS_SPI_PAUSE_US);
}

uint16_t Adis165xx::readRegister(uint8_t address)
{
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};
  tx[0] = (address) | SPI_READ;
  spi_.rx(tx, rx, 2, timeoutMs_);
  time64.dUs(ADIS_SPI_PAUSE_US);
  tx[0] = (++address) | SPI_READ;
  spi_.rx(tx, rx, 2, timeoutMs_);
  time64.dUs(ADIS_SPI_PAUSE_US);
  return (uint16_t) rx[1] | (uint16_t) rx[0] << 8;
}

bool Adis165xx::display(void)
{
  ImuPacket p;
  char name[] = "Adis165xx (imu0)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f %10.3f %10.3f g    ", p.accel[0] / 9.80665, p.accel[1] / 9.80665, p.accel[2] / 9.80665);
    misc_printf(" | %10.3f %10.3f %10.3f deg/s", p.gyro[0] * 57.2958, p.gyro[1] * 57.2958, p.gyro[2] * 57.2958);
    misc_printf(" | %7.1f C", p.temperature - 273.15);
    misc_printf(" | %10.3f s | 0x%04X", p.dataTime, p.status);
    if (p.status == ADIS_OK) misc_printf(" - OK\n");
    else misc_printf(" - NOK\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }
  return true;
}
