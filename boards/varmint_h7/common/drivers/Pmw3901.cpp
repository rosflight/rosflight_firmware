/**
 ******************************************************************************
 * File     : Pmw3901.cpp
 * Date     : Dec 8, 2025
 ******************************************************************************
 *
 * Copyright (c) 2025, AeroVironment, Inc.
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

// References:
// https://px4.github.io/Firmware-Doxygen/df/db0/_p_m_w3901_8cpp_source.html
// https://github.com/bitcraze/Bitcraze_PMW3901/blob/master/src/Bitcraze_PMW3901.cpp#L149
// https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_OpticalFlow/AP_OpticalFlow_Pixart.cpp


#include "Pmw3901.h"
#include "Packets.h"
#include "Time64.h"
#include "Polling.h"
#include "misc.h"

#define PMW3901_OK 0xB0
#define PMW3901_STATE_IDLE 0x00
#define PMW3901_STATE_READ_MOTION 0x02
#define PMW3901_STATE_READ_XLO 0x03
#define PMW3901_STATE_READ_XHI 0x04
#define PMW3901_STATE_READ_YLO 0x05
#define PMW3901_STATE_READ_YHI 0x06
#define PMW3901_STATE_READ_QUAL 0x07
#define PMW3901_STATE_READ_SLO 0x0B
#define PMW3901_STATE_READ_SHI 0x0C

#define PMW3901_COUNTS_PER_RADIAN (500.0)

//#define PMW3901_STATE_READ_BURST 0x16

#define SPI_WRITE(a) ((a)| (1<<7))
#define SPI_READ(a)  ((a)&0x7F)

#define CHIP_ID         0x49  // 01001001
#define CHIP_ID_INVERSE 0xB6  // 10110110

extern Time64 time64;
extern Polling polling;

#ifndef PMW3901_DMA_RAM
  #define PMW3901_DMA_RAM DMA_RAM
#endif
PMW3901_DMA_RAM uint8_t pmw3901_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
PMW3901_DMA_RAM uint8_t pmw3901_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t pmw3901_double_buffer[2 * sizeof(OpticalFlowPacket)];

uint32_t Pmw3901::init(
   uint16_t reference_rate_hz, uint16_t sample_rate_hz, uint16_t delay_us,
   SPI_HandleTypeDef * hspi,
   gpio_t cs, //GPIO_TypeDef * cs_port, uint16_t cs_pin, // SPI
   gpio_t drdy, //
   gpio_t reset, // GPIO_TypeDef * reset_port, uint16_t reset_pin, // Reset
   TIM_HandleTypeDef * htim
 )
{
  HAL_GPIO_WritePin(reset.port, reset.pin, GPIO_PIN_RESET);
  time64.dMs(50);
  HAL_GPIO_WritePin(reset.port, reset.pin, GPIO_PIN_SET);
  time64.dMs(50);

  drdyPin_ = drdy.pin;
  decimation_ = (reference_rate_hz + sample_rate_hz -1)/sample_rate_hz; // round up
  sampleRateHz_ = (double)reference_rate_hz/(double)decimation_;
  delayUs_ = delay_us-1;
  htim_ = htim;

  return init(hspi, cs);
}

uint32_t Pmw3901::init(
   uint16_t sample_rate_hz,
   SPI_HandleTypeDef * hspi,
   gpio_t cs // SPI
)
{
  decimation_ = 1;
  sampleRateHz_ = sample_rate_hz;
  delayUs_ = 0;
  htim_ = nullptr;
  return init(hspi, cs);
}


uint32_t Pmw3901::init(
   SPI_HandleTypeDef * hspi, gpio_t cs // SPI
 )
 {
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Pmw3901");
  initializationStatus_ = DRIVER_OK;

  drdy_ = time64.Us();

  hspi_    = hspi;
  spiPort_ = cs.port;
  spiPin_  = cs.pin;

  if (htim_!=nullptr) HAL_TIM_Base_Stop_IT(htim_);

  double_buffer_.init(pmw3901_double_buffer, sizeof(pmw3901_double_buffer) );


  time64.dMs(100); // make sure we are at least 100ms from power ON.

// Make sure the SPI bus is reset
  HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
  time64.dMs(1);
  HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
  time64.dMs(1);
  HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
  time64.dMs(1);

  // Power on reset
  writeRegister(0x3A, 0x5A);
  time64.dMs(50);

  // Test the SPI communication, checking product ID (and inverse product ID)
  uint8_t product_id = readRegister(0x00);
  uint8_t not_product_id = readRegister(0x5F);

  misc_printf("PMW3901: PRODUCT ID = 0x%02X  (0x49) ~ 0x%02X  (0xB6) -", product_id, not_product_id);
  if (product_id != CHIP_ID || not_product_id != CHIP_ID_INVERSE) {
    initializationStatus_ |= DRIVER_ID_MISMATCH;
    misc_printf(" Not OK\n");
    return initializationStatus_;
  } else {
    misc_printf(" OK\n");
  }

  uint8_t pattern[16] = {
    SPI_READ(PMW3901_STATE_READ_MOTION),0,
    SPI_READ(PMW3901_STATE_READ_XLO),0,
    SPI_READ(PMW3901_STATE_READ_XHI),0,
    SPI_READ(PMW3901_STATE_READ_YLO),0,
    SPI_READ(PMW3901_STATE_READ_YHI),0,
    SPI_READ(PMW3901_STATE_READ_QUAL),0,
    SPI_READ(PMW3901_STATE_READ_SLO),0,
    SPI_READ(PMW3901_STATE_READ_SHI),0
  };
  rx_bytes_ = sizeof(pattern);

  memcpy(pmw3901_dma_txbuf, pattern, sizeof(pattern));


  // Reading the motion registers one time, I don't know why
//  readRegister(0x02);
//  readRegister(0x03);
//  readRegister(0x04);
//  readRegister(0x05);
//  readRegister(0x06);
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi_, pmw3901_dma_txbuf, pmw3901_dma_rxbuf, 10, 100);
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);

  time64.dMs(1);

  //  Data Sheet register settings:
  writeRegister(0x7F, 0x00);
  writeRegister(0x61, 0xAD);
  writeRegister(0x7F, 0x03);
  writeRegister(0x40, 0x00);
  writeRegister(0x7F, 0x05);
  writeRegister(0x41, 0xB3);
  writeRegister(0x43, 0xF1);
  writeRegister(0x45, 0x14);
  writeRegister(0x5B, 0x32);
  writeRegister(0x5F, 0x34);
  writeRegister(0x7B, 0x08);
  writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B);
  writeRegister(0x40, 0xBF);
  writeRegister(0x4E, 0x3F);

//  Additional Bitcraze register settings:
  writeRegister(0x7F, 0x08);
  writeRegister(0x65, 0x20);
  writeRegister(0x6A, 0x18);
  writeRegister(0x7F, 0x09);
  writeRegister(0x4F, 0xAF);
  writeRegister(0x5F, 0x40);
  writeRegister(0x48, 0x80);
  writeRegister(0x49, 0x80);
  writeRegister(0x57, 0x77);
  writeRegister(0x60, 0x78);
  writeRegister(0x61, 0x78);
  writeRegister(0x62, 0x08);
  writeRegister(0x63, 0x50);
  writeRegister(0x7F, 0x0A);
  writeRegister(0x45, 0x60);
  writeRegister(0x7F, 0x00);
  writeRegister(0x4D, 0x11);
  writeRegister(0x55, 0x80);
  writeRegister(0x74, 0x1F);
  writeRegister(0x75, 0x1F);
  writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78);
  writeRegister(0x44, 0x08);
  writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF);
  writeRegister(0x65, 0x1F);
  writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x60);
  writeRegister(0x66, 0x08);
  writeRegister(0x63, 0x78);
  writeRegister(0x7F, 0x15);
  writeRegister(0x48, 0x58);
  writeRegister(0x7F, 0x07);
  writeRegister(0x41, 0x0D);
  writeRegister(0x43, 0x14);
  writeRegister(0x4B, 0x0E);
  writeRegister(0x45, 0x0F);
  writeRegister(0x44, 0x42);
  writeRegister(0x4C, 0x80);
  writeRegister(0x7F, 0x10);
  writeRegister(0x5B, 0x02);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x41);
  writeRegister(0x70, 0x00);
  time64.dMs(100);
  writeRegister(0x32, 0x44);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06);
  writeRegister(0x62, 0xf0);
  writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D);
  writeRegister(0x48, 0xC0);
  writeRegister(0x6F, 0xd5);
  writeRegister(0x7F, 0x00);
  writeRegister(0x5B, 0xa0);
  writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50);
  writeRegister(0x40, 0x80);

  return initializationStatus_;
}

bool Pmw3901::poll(uint64_t poll_counter){

  uint64_t decimation = (double)1e6 / sampleRateHz_ / (double)polling.period_us();
  uint16_t poll_state = (uint16_t) (poll_counter % decimation);

  if( poll_state == 0 ) poll();

  return true;
}

bool Pmw3901::poll(void){
  if (htim_!=nullptr) HAL_TIM_Base_Stop_IT(htim_);

  // note pmw3901_dma_txbuf is set once during init.
  memset(pmw3901_dma_rxbuf,0,rx_bytes_);

  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);

  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_RESET);

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hspi_, pmw3901_dma_txbuf, pmw3901_dma_rxbuf, rx_bytes_);

  if (HAL_OK != status) {
    HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);
    return false;
  }
  drdy_ = time64.Us();
  return true;
}

void Pmw3901::trigger(void) {
  if (htim_==nullptr) return;

  static uint32_t count = 0;
  count %= decimation_;
  if (count==0) {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    __HAL_TIM_SET_AUTORELOAD(htim_, delayUs_); // 500us delay for ADIS 16500 at < 2000Hz
    HAL_TIM_Base_Start_IT(htim_);
  }
  count++;
}

void Pmw3901::endDma(void) {
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);

  OpticalFlowPacket p;
  p.header.timestamp = drdy_;
  p.header.complete = time64.Us();

  uint16_t motion = pmw3901_dma_rxbuf[1];
  int16_t ix = ((int16_t)pmw3901_dma_rxbuf[5] << 8) | pmw3901_dma_rxbuf[3];
  int16_t iy = ((int16_t)pmw3901_dma_rxbuf[9] << 8) | pmw3901_dma_rxbuf[7];
  uint16_t qual = pmw3901_dma_rxbuf[11];

  uint16_t shutter = ((uint16_t)pmw3901_dma_rxbuf[15] << 8) | pmw3901_dma_rxbuf[13];
  // if shutter is maxed out (8191?), it is very low light and likely unreliable.


  p.header.status = (motion<<8) | qual; // Motion & Qual


  p.rate[0] = (double)ix*sampleRateHz_/PMW3901_COUNTS_PER_RADIAN; // rad/s
  p.rate[1] = (double)iy*sampleRateHz_/PMW3901_COUNTS_PER_RADIAN;

  p.shutter = shutter;

  write((uint8_t *) &p, sizeof(p));
}

void Pmw3901::writeRegister(uint8_t address, uint8_t value)
{

  address = SPI_WRITE(address);

  uint8_t tx[2] = {address,value};
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi_, tx, sizeof(tx), 100);
  time64.dUs(27); // so total CS low time is >=50us
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);
  time64.dUs(50);
}

uint8_t Pmw3901::readRegister(uint8_t address)
{
  address = SPI_READ(address);

  uint8_t tx[2] = {address,0};
  uint8_t rx[2] = {0,0};

  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspi_, tx, rx, sizeof(tx), 100);
  time64.dUs(27); // so total CS low time is >=50us
  HAL_GPIO_WritePin(spiPort_, spiPin_, GPIO_PIN_SET);
  time64.dUs(200);

  return rx[1];
}


bool Pmw3901::display(void)
{
  OpticalFlowPacket p;

  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name_, p.header );
    misc_f32(nan(""), nan(""), p.rate[0]*1.2, "x", "%6.0f", "deg/s");
    misc_f32(nan(""), nan(""), p.rate[1]*1.2, "y", "%6.0f", "deg/s");
    //misc_x16(PMW3901_OK, p.header.status, "Status");
    misc_x16(PMW3901_OK, p.header.status>>8, "Motion");
    misc_x16(PMW3901_OK, p.header.status&0xFF, "Squal");
    misc_x16(PMW3901_OK, p.shutter, "Shutter");
    misc_printf("\n");
    return 1;
  } else {
    misc_printf("%s\n", name_);
  }
  return 0;
}


