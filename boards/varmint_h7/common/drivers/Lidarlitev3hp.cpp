/**
 ******************************************************************************
 * File     : Lidarlitev3hp.cpp
 * Date     : Nov 17, 2025
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

#include "Lidarlitev3hp.h"
#include "Packets.h"
#include "Time64.h"
#include "misc.h"

extern Time64 time64;

#define ROLLOVER_US 10000

#define LIDARLITE_STATE_IDLE       0
#define LIDARLITE_STATE_POLLING           1
#define LIDARLITE_STATE_STATUS_CMD        2
#define LIDARLITE_STATE_STATUS_READ       3
#define LIDARLITE_STATE_DATA_CMD          4
#define LIDARLITE_STATE_DATA_READ         5
#define LIDARLITE_STATE_MEASURE_CMD       6
#define LIDARLITE_STATE_ERROR             7

// Control Register List - Address Definitions
#define ACQ_COMMAND            0x00  // Device command
#define STATUS                 0x01  // System status
#define SIG_COUNT_VAL          0x02  // Maximum acquisition count
#define ACQ_CONFIG_REG         0x04  // Acquisition mode control
#define LEGACY_RESET_EN        0x06  // Enables unit reset
#define SIGNAL_STRENGTH        0x0E  // Received signal strength
#define FULL_DELAY_HIGH        0x0F  // Distance measurement high byte
#define FULL_DELAY_LOW         0x10  // Distance measurement low byte
#define REF_COUNT_VAL          0x12  // Reference acquisition count
#define UNIT_ID_HIGH           0x16  // Serial number high byte
#define UNIT_ID_LOW            0x17  // Serial number low byte
#define I2C_ID_HIGH            0x18  // Write serial number high byte for I2C address unlock
#define I2C_ID_LOW             0x19  // Write serial number low byte for I2C address unlock
#define I2C_SEC_ADDR           0x1A  // Write new I2C address after unlock
#define THRESHOLD_BYPASS       0x1C  // Peak detection threshold bypass
#define I2C_CONFIG             0x1E  // Default address response control
#define PEAK_STACK_HIGH_BYTE   0x26  // Registers read successive values from the peak stack register (high byte)
#define PEAK_STACK_LOW_BYTE    0x27  // Registers read successive values from the peak stack register (low byte)
#define COMMAND                0x40  // State command
#define HEALTH_STATUS          0x48  // Used to diagnose major hardware issues at initialization
#define CORR_DATA              0x52  // Correlation record data low byte
#define CORR_DATA_SIGN         0x53  // Correlation record data high byte
#define POWER_CONTROL          0x65  // Power state control


DMA_RAM uint8_t lidarlite_i2c_dma_buf[I2C_DMA_MAX_BUFFER_SIZE];
DTCM_RAM uint8_t lidarlite_double_buffer[2 * sizeof(RangePacket)];

uint32_t Lidarlitev3hp::init(
  // Driver initializers
  uint16_t sample_rate_hz,
  // I2C initializers
  I2C_HandleTypeDef * hi2c, uint16_t i2c_address )
{
   snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Lidarlite");
   initializationStatus_ = DRIVER_OK;
   sampleRateHz_ = sample_rate_hz;

   hi2c_ = hi2c;
   address_ = i2c_address << 1;

   i2cState_ = LIDARLITE_STATE_IDLE;

   double_buffer_.init(lidarlite_double_buffer, sizeof(lidarlite_double_buffer));

   drdy_ = 0;

   dtMs_ = 1000. / (double) sampleRateHz_;


   // Check System Status Register

   status_ = 0;
   if (HAL_I2C_Mem_Read(hi2c_, address_, STATUS, I2C_MEMADD_SIZE_8BIT, &status_, 1, 1000) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }
   misc_printf("LidarLite Status = 0x%02X & 0x30 = 0x%02X\n", status_, status_ & 0x30);

   if ((status_ & 0x30) != 0x30) {
     initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
     return initializationStatus_;
   }

   // Check Health Register

   uint8_t health = 0;
   if (HAL_I2C_Mem_Read(hi2c_, address_, HEALTH_STATUS, I2C_MEMADD_SIZE_8BIT, &health, 1, 1000) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }
   misc_printf("LidarLite Health = 0x%02X & 0x17 = 0x%02X\n", health, health & 0x17);

   if ((health & 0x17) != 0x17) {
     initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
     return initializationStatus_;
   }

   uint8_t sigCountMax     = 0x80; // Default
   uint8_t acqConfigReg    = 0x08; // Default
   uint8_t refCountMax     = 0x05; // Default
   uint8_t thresholdBypass = 0x00; // Default

   // taken from https://github.com/garmin/LIDARLite_Arduino_Library/blob/master/src/LIDARLite_v3HP.cpp#L60C2-L60C3
   uint8_t configuration = 0;
   switch (configuration)
   {
       case 0: // Default mode, balanced performance
           sigCountMax     = 0x80; // Default
           acqConfigReg    = 0x08; // Default
           refCountMax     = 0x05; // Default
           thresholdBypass = 0x00; // Default
           break;

       case 1: // Short range, high speed
           sigCountMax     = 0x1d;
           acqConfigReg    = 0x08; // Default
           refCountMax     = 0x03;
           thresholdBypass = 0x00; // Default
           break;

       case 2: // Default range, higher speed short range
           sigCountMax     = 0x80; // Default
           acqConfigReg    = 0x00;
           refCountMax     = 0x03;
           thresholdBypass = 0x00; // Default
           break;

       case 3: // Maximum range
           sigCountMax     = 0xff;
           acqConfigReg    = 0x08; // Default
           refCountMax     = 0x05; // Default
           thresholdBypass = 0x00; // Default
           break;

       case 4: // High sensitivity detection, high erroneous measurements
           sigCountMax     = 0x80; // Default
           acqConfigReg    = 0x08; // Default
           refCountMax     = 0x05; // Default
           thresholdBypass = 0x80;
           break;

       case 5: // Low sensitivity detection, low erroneous measurements
           sigCountMax     = 0x80; // Default
           acqConfigReg    = 0x08; // Default
           refCountMax     = 0x05; // Default
           thresholdBypass = 0xb0;
           break;

       case 6: // Short range, high speed, higher error
           sigCountMax     = 0x04;
           acqConfigReg    = 0x01; // turn off short_sig, mode pin = status output mode
           refCountMax     = 0x03;
           thresholdBypass = 0x00;
           break;
   }

   if (HAL_I2C_Mem_Write(hi2c_, address_, SIG_COUNT_VAL,    I2C_MEMADD_SIZE_8BIT, &sigCountMax,     1, 1000) != HAL_OK) { initializationStatus_ |= DRIVER_HAL_ERROR; return initializationStatus_;}
   if (HAL_I2C_Mem_Write(hi2c_, address_, ACQ_CONFIG_REG,   I2C_MEMADD_SIZE_8BIT, &acqConfigReg,    1, 1000) != HAL_OK) { initializationStatus_ |= DRIVER_HAL_ERROR; return initializationStatus_;}
   if (HAL_I2C_Mem_Write(hi2c_, address_, REF_COUNT_VAL,    I2C_MEMADD_SIZE_8BIT, &refCountMax,     1, 1000) != HAL_OK) { initializationStatus_ |= DRIVER_HAL_ERROR; return initializationStatus_;}
   if (HAL_I2C_Mem_Write(hi2c_, address_, THRESHOLD_BYPASS, I2C_MEMADD_SIZE_8BIT, &thresholdBypass, 1, 1000) != HAL_OK) { initializationStatus_ |= DRIVER_HAL_ERROR; return initializationStatus_;}

   return initializationStatus_;
}


bool Lidarlitev3hp::poll(uint64_t poll_counter)
{
  PollingState poll_state = (PollingState) (poll_counter % (ROLLOVER_US / POLLING_PERIOD_US));

  if( poll_state == 0) {
    i2cState_ = LIDARLITE_STATE_POLLING;
    stateMachine();
  }

  return true;
}

void Lidarlitev3hp::stateMachine(void)
{
  switch(i2cState_)
  {
    case LIDARLITE_STATE_IDLE:
    {
      i2cState_ = LIDARLITE_STATE_ERROR;
      break; // pedantic.
    }
    case LIDARLITE_STATE_POLLING:
  {
      lidarlite_i2c_dma_buf[0] = STATUS;
      if (HAL_I2C_Master_Transmit_DMA(hi2c_, address_, lidarlite_i2c_dma_buf, 1)==HAL_OK) {
        i2cState_ = LIDARLITE_STATE_STATUS_CMD;
  } else {
    i2cState_ = LIDARLITE_STATE_ERROR;
  }
      break;
}
    case LIDARLITE_STATE_STATUS_CMD:
{
      // Start read of Status
      lidarlite_i2c_dma_buf[0] = 0;
      if (HAL_I2C_Master_Receive_DMA(hi2c_, address_, lidarlite_i2c_dma_buf,1) == HAL_OK) {
        i2cState_ = LIDARLITE_STATE_STATUS_READ;
      } else {
        i2cState_ = LIDARLITE_STATE_ERROR;
      }
      break;
    }
    case LIDARLITE_STATE_STATUS_READ:
  {
      // Check status
    status_ = lidarlite_i2c_dma_buf[0];
      if ((status_ & 0x01)==0x00)
      {
      drdy_ = time64.Us();
      // status good, start a read
        lidarlite_i2c_dma_buf[0] = FULL_DELAY_HIGH;

        if (HAL_I2C_Master_Transmit_DMA(hi2c_, address_, lidarlite_i2c_dma_buf,1) == HAL_OK) {
          i2cState_ = LIDARLITE_STATE_DATA_CMD;
        } else {
          i2cState_ = LIDARLITE_STATE_ERROR;
        }
      } else {
        // Give up and start a new measurement
        lidarlite_i2c_dma_buf[0] = ACQ_COMMAND;
        lidarlite_i2c_dma_buf[1] = 0x04;
        if (HAL_I2C_Master_Transmit_DMA(hi2c_, address_, lidarlite_i2c_dma_buf, 2) == HAL_OK) {
          i2cState_ = LIDARLITE_STATE_MEASURE_CMD;
        } else {
          i2cState_ = LIDARLITE_STATE_ERROR;
        }
      }
      break;
    }
    case LIDARLITE_STATE_DATA_CMD:
    {
      // status good, start a read
      lidarlite_i2c_dma_buf[0] = 0;
      lidarlite_i2c_dma_buf[1] = 0;
      if (HAL_I2C_Master_Receive_DMA(hi2c_, address_, lidarlite_i2c_dma_buf,2) == HAL_OK) {
        i2cState_ = LIDARLITE_STATE_DATA_READ;
    } else {
        i2cState_ = LIDARLITE_STATE_ERROR;
      }
      break;
    }
    case LIDARLITE_STATE_DATA_READ:
    {
    // Reading range data complete
    uint16_t urange = (lidarlite_i2c_dma_buf[0]<<8) | lidarlite_i2c_dma_buf[1]; // cm

    RangePacket p;
    p.header.timestamp = drdy_;
    p.header.complete = time64.Us();
    p.header.status = status_;
    p.range = ((float)urange)/100.0;
    p.min_range = 0.0;
    p.max_range = 40.0;
    p.type = rosflight_firmware::SensorRangeType::ROSFLIGHT_RANGE_LIDAR;
    // no specific check for status because we did that before starting the read.
    write((uint8_t *) &p, sizeof(p));

      // Start Next Measurement
      lidarlite_i2c_dma_buf[0] = ACQ_COMMAND;
      lidarlite_i2c_dma_buf[1] = 0x04;
      if (HAL_I2C_Master_Transmit_DMA(hi2c_, address_, lidarlite_i2c_dma_buf, 2)==HAL_OK) {
        i2cState_ = LIDARLITE_STATE_MEASURE_CMD;
  } else {
    i2cState_ = LIDARLITE_STATE_ERROR;
  }
      break;
}
    case LIDARLITE_STATE_MEASURE_CMD:
  {
      i2cState_ = LIDARLITE_STATE_IDLE;
      break;
  }
    case LIDARLITE_STATE_ERROR:
    {
      break;
}
    default:
{
    i2cState_ = LIDARLITE_STATE_ERROR;
      break; // pedantic.
  }
}
  }

bool Lidarlitev3hp::display(void)
{
  RangePacket p;
  char name[] = "LL_V3hp (range)";
  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header );
    misc_f32(p.min_range, p.max_range, p.range, "Range", "%6.2f", "m");
    misc_x16(0, p.header.status&0x01, "Status");
    misc_printf("\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }
  return true;
}















