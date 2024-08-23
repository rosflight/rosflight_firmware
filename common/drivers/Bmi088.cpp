/**
 ******************************************************************************
 * File     : Bmi088.cpp
 * Date     : Sep 28, 2023
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

#include <Bmi088.h>
#include <Bmi088_config.h>
#include <Packets.h>
#include <Time64.h>
#include <bmi08_defs.h>
#include <misc.h>

#define SPI_READ (uint8_t) 0x80
#define SPI_WRITE (uint8_t) 0x00

#define BMI_ACCEL_BYTES 20     // read bytes + 2
#define BMI_ACCEL_SYNC_BYTES 4 // read bytes + 2
#define BMI_GYRO_BYTES 7       // read bytes + 1
#define BMI_PAUSE_US 2         // us between spi transactions

#define BMI_ACCEL_CMD (0x12 | SPI_READ)
#define BMI_ACCEL_SYNC_CMD (0x27 | SPI_READ)
#define BMI_GYRO_CMD (0x02 | SPI_READ)

extern Time64 time64;

DMA_RAM uint8_t bmi088_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t bmi088_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t bmi088_fifo_rx_buffer[BMI088_FIFO_BUFFERS * sizeof(ImuPacket)];

uint32_t Bmi088::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // DRDY GPIO Port
  uint16_t drdy_pin,                                 // DRDY GPIO Pin
  // SPI initializers
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port_a, // Chip Select GPIO Port
  uint16_t cs_pin_a,                                  // Chip Select GPIO Pin
  GPIO_TypeDef * cs_port_g,                           // Chip Select GPIO Port
  uint16_t cs_pin_g,                                  // Chip Select GPIO Pin
  // Sensor Specific
  uint8_t range_a, // 0,1,2,3 --> 3,6,12,24g for BMI088; 2, 4, 8, 16g for BMI085
  uint8_t range_g  // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
)
{
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  spiA_.init(hspi, bmi088_dma_txbuf, bmi088_dma_rxbuf, cs_port_a, cs_pin_a);
  spiG_.init(hspi, bmi088_dma_txbuf, bmi088_dma_rxbuf, cs_port_g, cs_pin_g);

  seqCount_ = 0;
  timeoutMs_ = 1000;
  rangeA_ = range_a;
  rangeG_ = range_g;

  rxFifo_.init(BMI088_FIFO_BUFFERS, sizeof(ImuPacket), bmi088_fifo_rx_buffer);

  if (sampleRateHz_ <= 400) {
    sampleRateHz_ = 400;
    syncCfgMode_ = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;
    groupDelay_ = 7000;
  } else if (sampleRateHz_ <= 1000) {
    sampleRateHz_ = 1000;
    syncCfgMode_ = BMI08_ACCEL_DATA_SYNC_MODE_1000HZ;
    groupDelay_ = 2500;
  } else if (sampleRateHz_ <= 2000) {
    sampleRateHz_ = 2000;
    syncCfgMode_ = BMI08_ACCEL_DATA_SYNC_MODE_2000HZ;
    groupDelay_ = 1500;
  } else {
    sampleRateHz_ = 400;
    syncCfgMode_ = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;
    groupDelay_ = 7000;
  }

  time64.dMs(50); // Some time to ensure power-on completed.

  HAL_GPIO_WritePin(spiG_.port_, spiG_.pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(spiA_.port_, spiA_.pin_, GPIO_PIN_SET);
  time64.dUs(100);

  // Lock into SPI Mode
  HAL_GPIO_WritePin(spiA_.port_, spiA_.pin_, GPIO_PIN_RESET);
  time64.dUs(25);
  HAL_GPIO_WritePin(spiA_.port_, spiA_.pin_, GPIO_PIN_SET);
  time64.dUs(4);

  // Check Accel ID  (0x80)
  uint8_t accel_id = readRegisterA(BMI08_REG_ACCEL_CHIP_ID);
  misc_printf("BMI088 Accel ID = 0x%02X (0x1E/0x1F) - ", accel_id);
  if (accel_id == 0x1E) {
    misc_printf("OK - BMI088\n");
    accelRange_ = 3.0 * (double) ((0x0001) << (rangeA_)); // G's
  }                                                       // G's
  else if (accel_id == 0x1F) {
    misc_printf("OK - BMI085\n");
    accelRange_ = 2.0 * (double) ((0x0001) << (rangeA_)); // G's
  }

  else {
    misc_printf("FAIL\n");
    initializationStatus_ |= DRIVER_ID_MISMATCH;
  }

  // Check Gyro ID  (0x80)
  uint8_t gyro_id = readRegisterG(BMI08_REG_GYRO_CHIP_ID);
  misc_printf("BMI088 Gyro  ID = 0x%02X (0x0F) - ", gyro_id);
  if (gyro_id == 0x0F) misc_printf("OK\n");
  else {
    misc_printf("FAIL\n");
    initializationStatus_ |= DRIVER_ID_MISMATCH;
  }

  // Accel Soft Reset (0x7E, 0xB6)
  writeRegisterA(BMI08_REG_ACCEL_SOFTRESET, 0xB6);
  time64.dUs(1000); // required

  // Lock into SPI Mode (again after rest)
  HAL_GPIO_WritePin(spiA_.port_, spiA_.pin_, GPIO_PIN_RESET);
  time64.dUs(25);
  HAL_GPIO_WritePin(spiA_.port_, spiA_.pin_, GPIO_PIN_SET);

  // Why is this here in the Bosh examples?
  // time64.dUs(4900);

  // Activate Accel (0x7C,0x00)
  writeRegisterA(BMI08_REG_ACCEL_PWR_CONF, 0x00); // 0x00 active, 0x03 suspend.
  time64.dUs(450);                                // do we need this delay??

  // Disable "Config Loading" (0x59, 0x00)
  writeRegisterA(BMI08_REG_ACCEL_INIT_CTRL, 0X00);

#define BLOCK_SIZE 32
  for (uint8_t block = 0; block < 0xC0; block++) {
    writeRegisterA(BMI08_REG_ACCEL_RESERVED_5B, 0X00);  // (0x5B)
    writeRegisterA(BMI08_REG_ACCEL_RESERVED_5C, block); // (0x5C)

    uint8_t tx[BLOCK_SIZE + 1];
    tx[0] = BMI08_REG_ACCEL_FEATURE_CFG | SPI_WRITE; // (0x5E, data)
    memcpy(tx + 1, bmi_config + block * BLOCK_SIZE, BLOCK_SIZE);
    spiA_.tx(tx, sizeof(tx), 1000);
  }
  // Re-enable "Config Loading"
  writeRegisterA(BMI08_REG_ACCEL_INIT_CTRL, 0X01); // (0x59, 0x01)
  time64.dUs(150000);

  // Unknown status read (0xAA)
  //		uint8_t stat_a = readRegisterA(BMI08_REG_ACCEL_INTERNAL_STAT);
  readRegisterA(BMI08_REG_ACCEL_INTERNAL_STAT);
  // check something here??

  // Activate Accel, again (0x7C,0x00)
  writeRegisterA(BMI08_REG_ACCEL_PWR_CONF, 0X00);
  // time64.dUs(5000); // could this be 450us like above???
  time64.dUs(450); // do we need this delay??

  // Accelerometer ON (0x7D,0x04)
  writeRegisterA(BMI08_REG_ACCEL_PWR_CTRL, 0X04);
  time64.dUs(100);

  // Read/Write Low Power Mode
  readRegisterG(BMI08_REG_GYRO_LPM1);        // (0x91)
  writeRegisterG(BMI08_REG_GYRO_LPM1, 0x00); // (0x11,0x00) Set normal mode
  time64.dMs(30);

  // Read/Write Gyro Bandwidth
  uint8_t GyroBW;
  if (sampleRateHz_ >= 2000) GyroBW = BMI08_GYRO_BW_230_ODR_2000_HZ;
  else if (sampleRateHz_ >= 1000) GyroBW = BMI08_GYRO_BW_116_ODR_1000_HZ;
  else GyroBW = BMI08_GYRO_BW_47_ODR_400_HZ;

  readRegisterG(BMI08_REG_GYRO_BANDWIDTH);          //(0x90)
  writeRegisterG(BMI08_REG_GYRO_BANDWIDTH, GyroBW); // 0x10,0x83)

  // Read/Write Gyro Range
  readRegisterG(BMI08_REG_GYRO_RANGE);           //(0x8F)
  writeRegisterG(BMI08_REG_GYRO_RANGE, rangeG_); // (0x0F,0x02)
  time64.dUs(10000);                             // do we need this delay??

  // Read/Write BMI08_REG_ACCEL_FEATURE_CFG 7+1 bytes
  // This is undocumented, mimic Bosh code
  {
    uint8_t tx[8] = {0};
    uint8_t sync_cfg[8] = {0};
    tx[0] = BMI08_REG_ACCEL_FEATURE_CFG | SPI_READ; // (0xDE)
    spiA_.rx(tx, sync_cfg, 8, 1000);
    sync_cfg[1] = BMI08_REG_ACCEL_FEATURE_CFG | SPI_WRITE; // (0x5E)
    uint16_t reg_data = (syncCfgMode_ & BMI08_ACCEL_DATA_SYNC_MODE_MASK);
    sync_cfg[6] = reg_data & 0xFF; // low byte
    sync_cfg[7] = reg_data >> 8;   // high byte
    spiA_.tx(sync_cfg + 1, 7, 1000);
  }
  time64.dUs(100000); // delay of 100ms for data sync to take effect.

  readRegisterA(BMI08_REG_ACCEL_INT2_IO_CONF); //(0xD4)
  writeRegisterA(BMI08_REG_ACCEL_INT2_IO_CONF,
                 0x13);                           //(0x54, 0x13) Int2 as input, active high, reserved[0] = 1
  writeRegisterA(BMI08_REG_ACCEL_INT1_MAP, 0x01); // (0x56,0x01) undocumented.

  readRegisterA(BMI08_REG_ACCEL_INT1_IO_CONF);        // (0xD3)
  writeRegisterA(BMI08_REG_ACCEL_INT1_IO_CONF, 0x0A); // (0x53,0x0A) Int1 as output, active high

  //	readRegisterA(BMI08_REG_ACCEL_CONF); 		// (0x40)
  //	writeRegisterA(BMI08_REG_ACCEL_CONF, rangeA_);// (0x40,0x0A) Int1 as output, active high

  readRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_MAP); // (0x18)
  writeRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_MAP,
                 0x81);                                   //(0x18,0x80) 0x81 to map drdy to both int3 and int4
  readRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_CONF);        // (0x16)
  writeRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_CONF, 0x05); //(0x16,0x05)
  writeRegisterG(BMI08_REG_GYRO_INT_CTRL, 0x80);          //(0x15,0x80) Enable drdy interrupt on new data

  // why is this sequence here twice??
  readRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_MAP);         // (0x18)
  writeRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_MAP, 0x81);  //(0x18,0x80)
  readRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_CONF);        // (0x16)
  writeRegisterG(BMI08_REG_GYRO_INT3_INT4_IO_CONF, 0x05); //(0x16,0x05)
  writeRegisterG(BMI08_REG_GYRO_INT_CTRL, 0x80);          //(0x15,0x80)

  return initializationStatus_;
}

bool Bmi088::startDma(void)
{
  drdy_ = time64.Us();
  HAL_StatusTypeDef hal_status = spiA_.startDma(BMI_ACCEL_CMD, BMI_ACCEL_BYTES);
  if (hal_status == HAL_OK) seqCount_ = 1;
  return hal_status == HAL_OK;
}

void Bmi088::endDma(void)
{
  static ImuPacket p;

  double scale_factor;
  if (seqCount_ == 1) {
    memset(&p, 0, sizeof(p));
    uint8_t * rx = spiA_.endDma();
    scale_factor = (double) 9.80665 * accelRange_ / (double) (32768L) / 4.; // m/s^2

    p.dataTime = (double) (39.0625e-6 * (double) ((uint32_t) rx[8] | (uint32_t) rx[9] << 8 | (uint32_t) rx[10] << 16));
    p.temperature = (double) ((int16_t) rx[18] << 3 | (((int16_t) rx[19] >> 6) & 0x0003));
    if (p.temperature > 1023) p.temperature -= 2048.0;
    p.temperature *= 0.125;
    p.temperature += 23 + 273.15; // K

    int16_t data;
    data = (int16_t) rx[15] << 8 | (int16_t) rx[14];
    p.accel[0] = -scale_factor * (double) data;
    data = (int16_t) rx[17] << 8 | (int16_t) rx[16];
    p.accel[1] = -scale_factor * (double) data;

    // Launch the Accel read for az
    HAL_StatusTypeDef hal_status = spiA_.startDma(BMI_ACCEL_SYNC_CMD, BMI_ACCEL_SYNC_BYTES);
    if (hal_status == HAL_OK) seqCount_ = 2;
    else seqCount_ = 0;
  } else if (seqCount_ == 2) {
    uint8_t * rx = spiA_.endDma();
    scale_factor = (double) 9.80665 * accelRange_ / (double) (32768L) / 4.; // m/s^2

    int16_t az = (int16_t) rx[3] << 8 | (int16_t) rx[2];
    p.accel[2] = az * scale_factor;

    seqCount_ = 3;
    HAL_StatusTypeDef hal_status = spiG_.startDma(BMI_GYRO_CMD, BMI_GYRO_BYTES);
    if (hal_status == HAL_OK) seqCount_ = 3;
    else seqCount_ = 0;
  } else if (seqCount_ == 3) {
    uint8_t * rx = spiG_.endDma();
    // _gyro_range = 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
    scale_factor = (double) 1.0 / 8.192 / (double) (0x0001 << (rangeG_ + 1)) * 0.01745329252; // to rad/s

    int16_t data;
    data = (int16_t) rx[2] << 8 | (int16_t) rx[1];
    p.gyro[0] = -scale_factor * (double) data;
    data = (int16_t) rx[4] << 8 | (int16_t) rx[3];
    p.gyro[1] = -scale_factor * (double) data;
    data = (int16_t) rx[6] << 8 | (int16_t) rx[5];
    p.gyro[2] = scale_factor * (double) data;

    p.drdy = drdy_;
    p.groupDelay = groupDelay_;

    p.timestamp = time64.Us();

    rxFifo_.write((uint8_t *) &p, sizeof(p));

    seqCount_ = 0;
  } else {
    seqCount_ = 0;
  }
}

uint8_t Bmi088::readRegisterA(uint8_t reg)
{
  uint8_t tx[3] = {0};
  tx[0] = reg | SPI_READ;
  uint8_t rx[3] = {0};
  spiA_.rx(tx, rx, 3, timeoutMs_); // Ignore status
  return rx[2];
}

uint8_t Bmi088::readRegisterG(uint8_t reg)
{
  uint8_t tx[2] = {0, 0};
  tx[0] = reg | SPI_READ;
  uint8_t rx[2] = {0};
  spiG_.rx(tx, rx, 2, timeoutMs_); // Ignore status
  return rx[1];
}

void Bmi088::writeRegisterA(uint8_t reg, uint8_t data)
{
  uint8_t tx[2];
  tx[0] = reg | SPI_WRITE;
  tx[1] = data;
  spiA_.tx(tx, 2, timeoutMs_);
}

void Bmi088::writeRegisterG(uint8_t reg, uint8_t data)
{
  uint8_t tx[2];
  tx[0] = reg | SPI_WRITE;
  tx[1] = data;
  spiG_.tx(tx, 2, timeoutMs_);
}

bool Bmi088::display(void)
{
  ImuPacket p;
  char name[] = "Bmi088 (imu)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f %10.3f %10.3f g    ", p.accel[0] / 9.80665, p.accel[1] / 9.80665, p.accel[2] / 9.80665);
    misc_printf(" | %10.3f %10.3f %10.3f deg/s", p.gyro[0] * 57.2958, p.gyro[1] * 57.2958, p.gyro[2] * 57.2958);
    misc_printf(" | %7.1f C", p.temperature - 273.15);
    misc_printf(" | %10.3f s\n", p.dataTime);
    return 1;
  } else {
    misc_printf("%s\n", name);
  }
  return true;
}
