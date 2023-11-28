/**
* Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi08_defs.h
* @date       2022-04-10
* @version    v1.6.0
*
*/

#ifndef BMI08_DEFS_H_
#define BMI08_DEFS_H_

/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#endif

/*********************************************************************/
/** \name       Common macros                   */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)               S8_C(x)
#define UINT8_C(x)              U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)              S16_C(x)
#define UINT16_C(x)             U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)              S32_C(x)
#define UINT32_C(x)             U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)              S64_C(x)
#define UINT64_C(x)             U64_C(x)
#endif

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                    0
#else
#define NULL                    ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                    UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE                   UINT8_C(0)
#endif

/**
 * BMI08_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMI08_INTF_RET_TYPE
#define BMI08_INTF_RET_TYPE     int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMI08_INTF_RET_SUCCESS
#define BMI08_INTF_RET_SUCCESS  INT8_C(0)
#endif

/*************************** BMI08 Accelerometer Macros *****************************/

/** Register map */
/* Accel registers */

/**\name    Accel Chip Id register */
#define BMI08_REG_ACCEL_CHIP_ID                 UINT8_C(0x00)

/**\name    Accel Error condition register */
#define BMI08_REG_ACCEL_ERR                     UINT8_C(0x02)

/**\name    Accel Status flag register */
#define BMI08_REG_ACCEL_STATUS                  UINT8_C(0x03)

/**\name    Accel X LSB data register */
#define BMI08_REG_ACCEL_X_LSB                   UINT8_C(0x12)

/**\name    Accel X MSB data register */
#define BMI08_REG_ACCEL_X_MSB                   UINT8_C(0x13)

/**\name    Accel Y LSB data register */
#define BMI08_REG_ACCEL_Y_LSB                   UINT8_C(0x14)

/**\name    Accel Y MSB data register */
#define BMI08_REG_ACCEL_Y_MSB                   UINT8_C(0x15)

/**\name    Accel Z LSB data register */
#define BMI08_REG_ACCEL_Z_LSB                   UINT8_C(0x16)

/**\name    Accel Z MSB data register */
#define BMI08_REG_ACCEL_Z_MSB                   UINT8_C(0x17)

/**\name    Sensor time byte 0 register */
#define BMI08_REG_ACCEL_SENSORTIME_0            UINT8_C(0x18)

/**\name    Sensor time byte 1 register */
#define BMI08_REG_ACCEL_SENSORTIME_1            UINT8_C(0x19)

/**\name    Sensor time byte 2 register */
#define BMI08_REG_ACCEL_SENSORTIME_2            UINT8_C(0x1A)

/**\name    Accel Interrupt status0 register */
#define BMI08_REG_ACCEL_INT_STAT_0              UINT8_C(0x1C)

/**\name    Accel Interrupt status1 register */
#define BMI08_REG_ACCEL_INT_STAT_1              UINT8_C(0x1D)

/**\name    Accel general purpose register 0*/
#define BMI08_REG_ACCEL_GP_0                    UINT8_C(0x1E)

/**\name    Sensor temperature MSB data register */
#define BMI08_REG_TEMP_MSB                      UINT8_C(0x22)

/**\name    Sensor temperature LSB data register */
#define BMI08_REG_TEMP_LSB                      UINT8_C(0x23)

/**\name    Accel general purpose register 4*/
#define BMI08_REG_ACCEL_GP_4                    UINT8_C(0x27)

/**\name    Accel Internal status register */
#define BMI08_REG_ACCEL_INTERNAL_STAT           UINT8_C(0x2A)

/**\name    Accel configuration register */
#define BMI08_REG_ACCEL_CONF                    UINT8_C(0x40)

/**\name    Accel range setting register */
#define BMI08_REG_ACCEL_RANGE                   UINT8_C(0x41)

/**\name    Accel Interrupt pin 1 configuration register */
#define BMI08_REG_ACCEL_INT1_IO_CONF            UINT8_C(0x53)

/**\name    Accel Interrupt pin 2 configuration register */
#define BMI08_REG_ACCEL_INT2_IO_CONF            UINT8_C(0x54)

/**\name    Accel Interrupt latch configuration register */
#define BMI08_REG_ACCEL_INT_LATCH_CONF          UINT8_C(0x55)

/**\name    Accel Interrupt pin1 mapping register */
#define BMI08_REG_ACCEL_INT1_MAP                UINT8_C(0x56)

/**\name    Accel Interrupt pin2 mapping register */
#define BMI08_REG_ACCEL_INT2_MAP                UINT8_C(0x57)

/**\name    Accel Interrupt map register */
#define BMI08_REG_ACCEL_INT1_INT2_MAP_DATA      UINT8_C(0x58)

/**\name    Accel Init control register */
#define BMI08_REG_ACCEL_INIT_CTRL               UINT8_C(0x59)

/**\name    Accel Self test register */
#define BMI08_REG_ACCEL_SELF_TEST               UINT8_C(0x6D)

/**\name    Accel Power mode configuration register */
#define BMI08_REG_ACCEL_PWR_CONF                UINT8_C(0x7C)

/**\name    Accel Power control (switch on or off ) register */
#define BMI08_REG_ACCEL_PWR_CTRL                UINT8_C(0x7D)

/**\name    Accel Soft reset register */
#define BMI08_REG_ACCEL_SOFTRESET               UINT8_C(0x7E)

/**\name    Feature Config related Registers */
#define BMI08_REG_ACCEL_RESERVED_5B             UINT8_C(0x5B)
#define BMI08_REG_ACCEL_RESERVED_5C             UINT8_C(0x5C)
#define BMI08_REG_ACCEL_FEATURE_CFG             UINT8_C(0x5E)

/**\name    Accel I2C slave address */
#define BMI08_ACCEL_I2C_ADDR_PRIMARY            UINT8_C(0x18)
#define BMI08_ACCEL_I2C_ADDR_SECONDARY          UINT8_C(0x19)

/**\name    Interrupt masks */
#define BMI08_ACCEL_DATA_READY_INT              UINT8_C(0x80)
#define BMI08_ACCEL_FIFO_WM_INT                 UINT8_C(0x02)
#define BMI08_ACCEL_FIFO_FULL_INT               UINT8_C(0x01)

#define BMI08_GYRO_DATA_READY_INT               UINT8_C(0x80)
#define BMI08_GYRO_FIFO_WM_INT                  UINT8_C(0x10)
#define BMI08_GYRO_FIFO_FULL_INT                UINT8_C(0x10)

/**\name    Initialization status */
#define BMI08_INIT_NOT_OK                       UINT8_C(0x00)
#define BMI08_INIT_OK                           UINT8_C(0x01)

/**\name    Accel Bandwidth */
#define BMI08_ACCEL_BW_OSR4                     UINT8_C(0x08)
#define BMI08_ACCEL_BW_OSR2                     UINT8_C(0x09)
#define BMI08_ACCEL_BW_NORMAL                   UINT8_C(0x0A)

/**\name    Accel Output data rate */
#define BMI08_ACCEL_ODR_12_5_HZ                 UINT8_C(0x05)
#define BMI08_ACCEL_ODR_25_HZ                   UINT8_C(0x06)
#define BMI08_ACCEL_ODR_50_HZ                   UINT8_C(0x07)
#define BMI08_ACCEL_ODR_100_HZ                  UINT8_C(0x08)
#define BMI08_ACCEL_ODR_200_HZ                  UINT8_C(0x09)
#define BMI08_ACCEL_ODR_400_HZ                  UINT8_C(0x0A)
#define BMI08_ACCEL_ODR_800_HZ                  UINT8_C(0x0B)
#define BMI08_ACCEL_ODR_1600_HZ                 UINT8_C(0x0C)

/**\name    Accel Self test */
#define BMI08_ACCEL_SWITCH_OFF_SELF_TEST        UINT8_C(0x00)
#define BMI08_ACCEL_POSITIVE_SELF_TEST          UINT8_C(0x0D)
#define BMI08_ACCEL_NEGATIVE_SELF_TEST          UINT8_C(0x09)

/**\name    Accel Power mode */
#define BMI08_ACCEL_PM_ACTIVE                   UINT8_C(0x00)
#define BMI08_ACCEL_PM_SUSPEND                  UINT8_C(0x03)

/**\name    Accel Power control settings */
#define BMI08_ACCEL_POWER_DISABLE               UINT8_C(0x00)
#define BMI08_ACCEL_POWER_ENABLE                UINT8_C(0x04)

/**\name    Accel internal interrupt pin mapping */
#define BMI08_ACCEL_DATA_SYNC_INT_DISABLE       UINT8_C(0x00)
#define BMI08_ACCEL_DATA_SYNC_INT_ENABLE        UINT8_C(0x01)

/**\name    Accel Soft reset delay */
#define BMI08_ACCEL_SOFTRESET_DELAY_MS          UINT8_C(1)

/**\name    Mask definitions for ACCEL_ERR_REG register */
#define BMI08_FATAL_ERR_MASK                    UINT8_C(0x01)
#define BMI08_ERR_CODE_MASK                     UINT8_C(0x1C)

/**\name    Position definitions for ACCEL_ERR_REG register */
#define BMI08_CMD_ERR_POS                       UINT8_C(1)
#define BMI08_ERR_CODE_POS                      UINT8_C(2)

/**\name    Mask definition for ACCEL_STATUS_REG register */
#define BMI08_ACCEL_STATUS_MASK                 UINT8_C(0x80)

/**\name    Position definitions for ACCEL_STATUS_REG  */
#define BMI08_ACCEL_STATUS_POS                  UINT8_C(7)

/**\name    Mask definitions for odr, bandwidth and range */
#define BMI08_ACCEL_ODR_MASK                    UINT8_C(0x0F)
#define BMI08_ACCEL_BW_MASK                     UINT8_C(0xF0)
#define BMI08_ACCEL_RANGE_MASK                  UINT8_C(0x03)

/**\name    Position definitions for odr, bandwidth and range */
#define BMI08_ACCEL_BW_POS                      UINT8_C(4)

/**\name    Mask definitions for INT1_IO_CONF register */
#define BMI08_ACCEL_INT_EDGE_MASK               UINT8_C(0x01)
#define BMI08_ACCEL_INT_LVL_MASK                UINT8_C(0x02)
#define BMI08_ACCEL_INT_OD_MASK                 UINT8_C(0x04)
#define BMI08_ACCEL_INT_IO_MASK                 UINT8_C(0x08)
#define BMI08_ACCEL_INT_IN_MASK                 UINT8_C(0x10)

/**\name    Position definitions for INT1_IO_CONF register */
#define BMI08_ACCEL_INT_EDGE_POS                UINT8_C(0)
#define BMI08_ACCEL_INT_LVL_POS                 UINT8_C(1)
#define BMI08_ACCEL_INT_OD_POS                  UINT8_C(2)
#define BMI08_ACCEL_INT_IO_POS                  UINT8_C(3)
#define BMI08_ACCEL_INT_IN_POS                  UINT8_C(4)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI08_ACCEL_MAP_INTA_MASK               UINT8_C(0x01)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define BMI08_ACCEL_MAP_INTA_POS                UINT8_C(0x00)

/**\name    Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI08_ACCEL_INT1_DRDY_MASK              UINT8_C(0x04)
#define BMI08_ACCEL_INT2_DRDY_MASK              UINT8_C(0x40)

/**\name    Position definitions for INT1_INT2_MAP_DATA register */
#define BMI08_ACCEL_INT1_DRDY_POS               UINT8_C(2)
#define BMI08_ACCEL_INT2_DRDY_POS               UINT8_C(6)

/**\name    Asic Initialization value */
#define BMI08_ASIC_INITIALIZED                  UINT8_C(0x01)

/*************************** BMI08 Gyroscope Macros *****************************/
/** Register map */
/* Gyro registers */

/**\name    Gyro Chip Id register */
#define BMI08_REG_GYRO_CHIP_ID                  UINT8_C(0x00)

/**\name    Gyro X LSB data register */
#define BMI08_REG_GYRO_X_LSB                    UINT8_C(0x02)

/**\name    Gyro X MSB data register */
#define BMI08_REG_GYRO_X_MSB                    UINT8_C(0x03)

/**\name    Gyro Y LSB data register */
#define BMI08_REG_GYRO_Y_LSB                    UINT8_C(0x04)

/**\name    Gyro Y MSB data register */
#define BMI08_REG_GYRO_Y_MSB                    UINT8_C(0x05)

/**\name    Gyro Z LSB data register */
#define BMI08_REG_GYRO_Z_LSB                    UINT8_C(0x06)

/**\name    Gyro Z MSB data register */
#define BMI08_REG_GYRO_Z_MSB                    UINT8_C(0x07)

/**\name    Gyro Interrupt status register */
#define BMI08_REG_GYRO_INT_STAT_1               UINT8_C(0x0A)

/**\name    Gyro FIFO status register */
#define BMI08_REG_GYRO_FIFO_STATUS              UINT8_C(0x0E)

/**\name    Gyro Range register */
#define BMI08_REG_GYRO_RANGE                    UINT8_C(0x0F)

/**\name    Gyro Bandwidth register */
#define BMI08_REG_GYRO_BANDWIDTH                UINT8_C(0x10)

/**\name    Gyro Power register */
#define BMI08_REG_GYRO_LPM1                     UINT8_C(0x11)

/**\name    Gyro Soft reset register */
#define BMI08_REG_GYRO_SOFTRESET                UINT8_C(0x14)

/**\name    Gyro Interrupt control register */
#define BMI08_REG_GYRO_INT_CTRL                 UINT8_C(0x15)

/**\name    Gyro Interrupt Pin configuration register */
#define BMI08_REG_GYRO_INT3_INT4_IO_CONF        UINT8_C(0x16)

/**\name    Gyro Interrupt Map register */
#define BMI08_REG_GYRO_INT3_INT4_IO_MAP         UINT8_C(0x18)

/**\name    Gyro FIFO watermark enable register */
#define BMI08_REG_GYRO_FIFO_WM_ENABLE           UINT8_C(0x1E)

/**\name    Gyro Self test register */
#define BMI08_REG_GYRO_SELF_TEST                UINT8_C(0x3C)

/**\name    Gyro Fifo Config 0 register */
#define BMI08_REG_GYRO_FIFO_CONFIG0             UINT8_C(0x3D)

/**\name    Gyro Fifo Config 1 register */
#define BMI08_REG_GYRO_FIFO_CONFIG1             UINT8_C(0x3E)

/**\name    Gyro Fifo Data register */
#define BMI08_REG_GYRO_FIFO_DATA                UINT8_C(0x3F)

/**\name    Gyro unique chip identifier */
#define BMI08_GYRO_CHIP_ID                      UINT8_C(0x0F)

/**\name    Gyro I2C slave address */
#define BMI08_GYRO_I2C_ADDR_PRIMARY             UINT8_C(0x68)
#define BMI08_GYRO_I2C_ADDR_SECONDARY           UINT8_C(0x69)

/**\name    Gyro Range */
#define BMI08_GYRO_RANGE_2000_DPS               UINT8_C(0x00)
#define BMI08_GYRO_RANGE_1000_DPS               UINT8_C(0x01)
#define BMI08_GYRO_RANGE_500_DPS                UINT8_C(0x02)
#define BMI08_GYRO_RANGE_250_DPS                UINT8_C(0x03)
#define BMI08_GYRO_RANGE_125_DPS                UINT8_C(0x04)

/**\name    Gyro Output data rate and bandwidth */
#define BMI08_GYRO_BW_532_ODR_2000_HZ           UINT8_C(0x00)
#define BMI08_GYRO_BW_230_ODR_2000_HZ           UINT8_C(0x01)
#define BMI08_GYRO_BW_116_ODR_1000_HZ           UINT8_C(0x02)
#define BMI08_GYRO_BW_47_ODR_400_HZ             UINT8_C(0x03)
#define BMI08_GYRO_BW_23_ODR_200_HZ             UINT8_C(0x04)
#define BMI08_GYRO_BW_12_ODR_100_HZ             UINT8_C(0x05)
#define BMI08_GYRO_BW_64_ODR_200_HZ             UINT8_C(0x06)
#define BMI08_GYRO_BW_32_ODR_100_HZ             UINT8_C(0x07)
#define BMI08_GYRO_ODR_RESET_VAL                UINT8_C(0x80)

/**\name    Gyro Power mode */
#define BMI08_GYRO_PM_NORMAL                    UINT8_C(0x00)
#define BMI08_GYRO_PM_DEEP_SUSPEND              UINT8_C(0x20)
#define BMI08_GYRO_PM_SUSPEND                   UINT8_C(0x80)

/**\name    Gyro data ready interrupt enable value */
#define BMI08_GYRO_DRDY_INT_DISABLE_VAL         UINT8_C(0x00)
#define BMI08_GYRO_DRDY_INT_ENABLE_VAL          UINT8_C(0x80)
#define BMI08_GYRO_FIFO_INT_DISABLE_VAL         UINT8_C(0x00)
#define BMI08_GYRO_FIFO_INT_ENABLE_VAL          UINT8_C(0x40)
#define BMI08_GYRO_FIFO_WM_ENABLE_VAL           UINT8_C(0x80)
#define BMI08_GYRO_FIFO_WM_DISABLE_VAL          UINT8_C(0x00)

/**\name    Gyro data ready map values */
#define BMI08_GYRO_MAP_DRDY_TO_INT3             UINT8_C(0x01)
#define BMI08_GYRO_MAP_DRDY_TO_INT4             UINT8_C(0x80)
#define BMI08_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4   UINT8_C(0x81)
#define BMI08_GYRO_MAP_FIFO_INT3                UINT8_C(0x04)
#define BMI08_GYRO_MAP_FIFO_INT4                UINT8_C(0x20)
#define BMI08_GYRO_MAP_FIFO_BOTH_INT3_INT4      UINT8_C(0x24)

/**\name    Gyro Soft reset delay */
#define BMI08_GYRO_SOFTRESET_DELAY              UINT8_C(30)

/**\name    Gyro power mode config delay */
#define BMI08_GYRO_POWER_MODE_CONFIG_DELAY      UINT8_C(30)

/** Mask definitions for range, bandwidth and power */
#define BMI08_GYRO_RANGE_MASK                   UINT8_C(0x07)
#define BMI08_GYRO_BW_MASK                      UINT8_C(0x0F)
#define BMI08_GYRO_POWER_MASK                   UINT8_C(0xA0)

/** Position definitions for range, bandwidth and power */
#define BMI08_GYRO_POWER_POS                    UINT8_C(5)

/**\name    Mask definitions for BMI08_GYRO_INT_CTRL_REG register */
#define BMI08_GYRO_DATA_EN_MASK                 UINT8_C(0x80)

/**\name    Position definitions for BMI08_GYRO_INT_CTRL_REG register */
#define BMI08_GYRO_DATA_EN_POS                  UINT8_C(7)

/**\name    Mask definitions for BMI08_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI08_GYRO_INT3_LVL_MASK                UINT8_C(0x01)
#define BMI08_GYRO_INT3_OD_MASK                 UINT8_C(0x02)
#define BMI08_GYRO_INT4_LVL_MASK                UINT8_C(0x04)
#define BMI08_GYRO_INT4_OD_MASK                 UINT8_C(0x08)

/**\name    Position definitions for BMI08_GYRO_INT3_INT4_IO_CONF_REG register */
#define BMI08_GYRO_INT3_OD_POS                  UINT8_C(1)
#define BMI08_GYRO_INT4_LVL_POS                 UINT8_C(2)
#define BMI08_GYRO_INT4_OD_POS                  UINT8_C(3)

/**\name    Mask definitions for BMI08_GYRO_INT_EN_REG register */
#define BMI08_GYRO_INT_EN_MASK                  UINT8_C(0x80)

/**\name    Position definitions for BMI08_GYRO_INT_EN_REG register */
#define BMI08_GYRO_INT_EN_POS                   UINT8_C(7)

/**\name    Mask definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI08_GYRO_INT3_MAP_MASK                UINT8_C(0x01)
#define BMI08_GYRO_INT4_MAP_MASK                UINT8_C(0x80)

/**\name    Position definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI08_GYRO_INT3_MAP_POS                 UINT8_C(0)
#define BMI08_GYRO_INT4_MAP_POS                 UINT8_C(7)

/**\name    Mask definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI088_GYRO_INT3_MAP_MASK               UINT8_C(0x01)
#define BMI088_GYRO_INT4_MAP_MASK               UINT8_C(0x80)

/**\name    Position definitions for BMI088_GYRO_INT_MAP_REG register */
#define BMI088_GYRO_INT3_MAP_POS                UINT8_C(0)
#define BMI088_GYRO_INT4_MAP_POS                UINT8_C(7)

/**\name    Mask definitions for GYRO_SELF_TEST register */
#define BMI08_GYRO_SELF_TEST_EN_MASK            UINT8_C(0x01)
#define BMI08_GYRO_SELF_TEST_RDY_MASK           UINT8_C(0x02)
#define BMI08_GYRO_SELF_TEST_RESULT_MASK        UINT8_C(0x04)
#define BMI08_GYRO_SELF_TEST_FUNCTION_MASK      UINT8_C(0x08)

/**\name    Position definitions for GYRO_SELF_TEST register */
#define BMI08_GYRO_SELF_TEST_RDY_POS            UINT8_C(1)
#define BMI08_GYRO_SELF_TEST_RESULT_POS         UINT8_C(2)
#define BMI08_GYRO_SELF_TEST_FUNCTION_POS       UINT8_C(3)

/**\name    Gyro Fifo configurations */
#define BMI08_GYRO_FIFO_OVERRUN_MASK            UINT8_C(0x80)
#define BMI08_GYRO_FIFO_OVERRUN_POS             UINT8_C(0x07)
#define BMI08_GYRO_FIFO_MODE_MASK               UINT8_C(0xC0)
#define BMI08_GYRO_FIFO_MODE_POS                UINT8_C(0x06)
#define BMI08_GYRO_FIFO_TAG_MASK                UINT8_C(0x80)
#define BMI08_GYRO_FIFO_TAG_POS                 UINT8_C(0x07)
#define BMI08_GYRO_FIFO_DATA_SELECT_MASK        UINT8_C(0x03)
#define BMI08_GYRO_FIFO_FRAME_COUNT_MASK        UINT8_C(0x7F)
#define BMI08_GYRO_FIFO_WM_LEVEL_MASK           UINT8_C(0x7F)

/*! @name Gyro Fifo interrupt map */
#define BMI08_GYRO_FIFO_INT3_MASK               UINT8_C(0x04)
#define BMI08_GYRO_FIFO_INT3_POS                UINT8_C(0x02)
#define BMI08_GYRO_FIFO_INT4_MASK               UINT8_C(0x20)
#define BMI08_GYRO_FIFO_INT4_POS                UINT8_C(0x05)

/**\name   Gyro FIFO definitions */
#define BMI08_GYRO_FIFO_TAG_ENABLED             UINT8_C(0x01)
#define BMI08_GYRO_FIFO_TAG_DISABLED            UINT8_C(0x00)
#define BMI08_GYRO_FIFO_MODE_BYPASS             UINT8_C(0x00)
#define BMI08_GYRO_FIFO_MODE                    UINT8_C(0x01)
#define BMI08_GYRO_FIFO_MODE_STREAM             UINT8_C(0x02)
#define BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED        UINT8_C(0x00)
#define BMI08_GYRO_FIFO_X_AXIS_ENABLED          UINT8_C(0x01)
#define BMI08_GYRO_FIFO_Y_AXIS_ENABLED          UINT8_C(0x02)
#define BMI08_GYRO_FIFO_Z_AXIS_ENABLED          UINT8_C(0x03)
#define BMI08_GYRO_FIFO_XYZ_AXIS_FRAME_SIZE     UINT8_C(0x06)
#define BMI08_GYRO_FIFO_SINGLE_AXIS_FRAME_SIZE  UINT8_C(0x02)
#define BMI08_GYRO_FIFO_1KB_BUFFER              UINT16_C(1024)

/*************************** Common Macros for both Accel and Gyro *****************************/
/**\name    SPI read/write mask to configure address */
#define BMI08_SPI_RD_MASK                       UINT8_C(0x80)
#define BMI08_SPI_WR_MASK                       UINT8_C(0x7F)

/**\name API success code */
#define BMI08_OK                                INT8_C(0)

/**\name API error codes */
#define BMI08_E_NULL_PTR                        INT8_C(-1)
#define BMI08_E_COM_FAIL                        INT8_C(-2)
#define BMI08_E_DEV_NOT_FOUND                   INT8_C(-3)
#define BMI08_E_OUT_OF_RANGE                    INT8_C(-4)
#define BMI08_E_INVALID_INPUT                   INT8_C(-5)
#define BMI08_E_CONFIG_STREAM_ERROR             INT8_C(-6)
#define BMI08_E_RD_WR_LENGTH_INVALID            INT8_C(-7)
#define BMI08_E_INVALID_CONFIG                  INT8_C(-8)
#define BMI08_E_FEATURE_NOT_SUPPORTED           INT8_C(-9)
#define BMI08_E_SELF_TEST_FAIL                  INT8_C(-10)

/***\name    Soft reset Value */
#define BMI08_SOFT_RESET_CMD                    UINT8_C(0xB6)

/**\name    Enable/disable macros */
#define BMI08_DISABLE                           UINT8_C(0)
#define BMI08_ENABLE                            UINT8_C(1)

/**\name  To define warnings for FIFO activity */
#define BMI08_W_FIFO_EMPTY                      INT8_C(1)
#define BMI08_W_PARTIAL_READ                    INT8_C(2)

/**\name  Maximum length to read */
#define BMI08_MAX_LEN                           UINT8_C(128)

/**\name  Sensortime resolution in seconds */
#define BMI08_SENSORTIME_RESOLUTION             0.0000390625f

/**\name    Constant values macros */
#define BMI08_SENSOR_DATA_SYNC_TIME_MS          UINT8_C(1)
#define BMI08_DELAY_BETWEEN_WRITES_MS           UINT8_C(1)
#define BMI08_SELF_TEST_DELAY_MS                UINT8_C(3)
#define BMI08_POWER_CONFIG_DELAY                UINT8_C(5)
#define BMI08_GYRO_SET_CONFIG_DELAY             UINT8_C(10)
#define BMI08_SENSOR_SETTLE_TIME_MS             UINT8_C(30)
#define BMI08_SET_ACCEL_CONF_DELAY              UINT8_C(40)
#define BMI08_SELF_TEST_DATA_READ_MS            UINT8_C(50)
#define BMI08_ASIC_INIT_TIME_MS                 UINT8_C(150)

#define BMI08_CONFIG_STREAM_SIZE                UINT16_C(6144)

/**\name    Sensor time array parameter definitions */
#define BMI08_SENSOR_TIME_MSB_BYTE              UINT8_C(2)
#define BMI08_SENSOR_TIME_XLSB_BYTE             UINT8_C(1)
#define BMI08_SENSOR_TIME_LSB_BYTE              UINT8_C(0)

/**\name   int pin active state */
#define BMI08_INT_ACTIVE_LOW                    UINT8_C(0)
#define BMI08_INT_ACTIVE_HIGH                   UINT8_C(1)

/**\name   interrupt pin output definition  */
#define BMI08_INT_MODE_PUSH_PULL                UINT8_C(0)
#define BMI08_INT_MODE_OPEN_DRAIN               UINT8_C(1)

/**\name    Sensor bit resolution */
#define BMI08_16_BIT_RESOLUTION                 UINT8_C(16)

/**\name    Convert milliseconds to microseconds */
#define BMI08_MS_TO_US(X)                       UINT32_C(X * 1000)

/*********************************BMI08 FIFO Macros**********************************/
/** Register map */
/*! @name FIFO Header Mask definitions */
#define BMI08_FIFO_HEADER_ACC_FRM               UINT8_C(0x84)
#define BMI08_FIFO_HEADER_ALL_FRM               UINT8_C(0x9C)
#define BMI08_FIFO_HEADER_SENS_TIME_FRM         UINT8_C(0x44)
#define BMI08_FIFO_HEADER_SKIP_FRM              UINT8_C(0x40)
#define BMI08_FIFO_HEADER_INPUT_CFG_FRM         UINT8_C(0x48)
#define BMI08_FIFO_HEAD_OVER_READ_MSB           UINT8_C(0x80)
#define BMI08_FIFO_SAMPLE_DROP_FRM              UINT8_C(0x50)

/* Accel registers */
#define BMI08_FIFO_LENGTH_0_ADDR                UINT8_C(0x24)
#define BMI08_FIFO_LENGTH_1_ADDR                UINT8_C(0x25)
#define BMI08_FIFO_DATA_ADDR                    UINT8_C(0x26)
#define BMI08_FIFO_DOWNS_ADDR                   UINT8_C(0x45)
#define BMI08_FIFO_WTM_0_ADDR                   UINT8_C(0x46)
#define BMI08_FIFO_WTM_1_ADDR                   UINT8_C(0x47)
#define BMI08_FIFO_CONFIG_0_ADDR                UINT8_C(0x48)
#define BMI08_FIFO_CONFIG_1_ADDR                UINT8_C(0x49)

/*! @name FIFO sensor data lengths */
#define BMI08_FIFO_ACCEL_LENGTH                 UINT8_C(6)
#define BMI08_FIFO_WTM_LENGTH                   UINT8_C(2)
#define BMI08_FIFO_LENGTH_MSB_BYTE              UINT8_C(1)
#define BMI08_FIFO_DATA_LENGTH                  UINT8_C(2)
#define BMI08_FIFO_CONFIG_LENGTH                UINT8_C(2)
#define BMI08_SENSOR_TIME_LENGTH                UINT8_C(3)
#define BMI08_FIFO_SKIP_FRM_LENGTH              UINT8_C(1)
#define BMI08_FIFO_INPUT_CFG_LENGTH             UINT8_C(1)

/*! @name FIFO byte counter mask definition */
#define BMI08_FIFO_BYTE_COUNTER_MSB_MASK        UINT8_C(0x3F)

/*! @name FIFO frame masks */
#define BMI08_FIFO_LSB_CONFIG_CHECK             UINT8_C(0x00)
#define BMI08_FIFO_MSB_CONFIG_CHECK             UINT8_C(0x80)
#define BMI08_FIFO_INTR_MASK                    UINT8_C(0x5C)

/*name FIFO config modes */
#define BMI08_ACC_STREAM_MODE                   UINT8_C(0x00)
#define BMI08_ACC_FIFO_MODE                     UINT8_C(0x01)

/*name Mask definitions for FIFO configuration modes */
#define BMI08_ACC_FIFO_MODE_CONFIG_MASK         UINT8_C(0x01)

/*! @name Mask definitions for FIFO_CONFIG_1 register */
#define BMI08_ACCEL_EN_MASK                     UINT8_C(0x40)
#define BMI08_ACCEL_INT1_EN_MASK                UINT8_C(0x08)
#define BMI08_ACCEL_INT2_EN_MASK                UINT8_C(0x04)

/*name Position definitions for FIFO_CONFIG_1 register */
#define BMI08_ACCEL_EN_POS                      UINT8_C(6)
#define BMI08_ACCEL_INT1_EN_POS                 UINT8_C(3)
#define BMI08_ACCEL_INT2_EN_POS                 UINT8_C(2)

/*! @name Position definitions for FIFO_DOWNS register */
#define BMI08_ACC_FIFO_DOWNS_MASK               UINT8_C(0xF0)

/*! @name FIFO down sampling bit positions */
#define BMI08_ACC_FIFO_DOWNS_POS                UINT8_C(0x04)

/*! @name FIFO down sampling user macros */
#define BMI08_ACC_FIFO_DOWN_SAMPLE_0            UINT8_C(0)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_1            UINT8_C(1)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_2            UINT8_C(2)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_3            UINT8_C(3)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_4            UINT8_C(4)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_5            UINT8_C(5)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_6            UINT8_C(6)
#define BMI08_ACC_FIFO_DOWN_SAMPLE_7            UINT8_C(7)

/*! @name Mask definitions for INT1_INT2_MAP_DATA register */
#define BMI08_ACCEL_INT2_FWM_MASK               UINT8_C(0x20)
#define BMI08_ACCEL_INT2_FFULL_MASK             UINT8_C(0x10)
#define BMI08_ACCEL_INT1_FWM_MASK               UINT8_C(0x02)
#define BMI08_ACCEL_INT1_FFULL_MASK             UINT8_C(0x01)

/*! @name Positions definitions for INT1_INT2_MAP_DATA register */
#define BMI08_ACCEL_INT1_FWM_POS                UINT8_C(1)
#define BMI08_ACCEL_INT2_FFULL_POS              UINT8_C(4)
#define BMI08_ACCEL_INT2_FWM_POS                UINT8_C(5)

/*! @name Accel Data sync */
#define BMI08_ACCEL_DATA_SYNC_ADR               UINT8_C(0x02)
#define BMI08_ACCEL_DATA_SYNC_LEN               UINT8_C(1)
#define BMI08_ACCEL_DATA_SYNC_MODE_MASK         UINT16_C(0x0003)
#define BMI08_ACCEL_DATA_SYNC_MODE_SHIFT        UINT8_C(0)

#define BMI08_ACCEL_DATA_SYNC_MODE_OFF          UINT8_C(0x00)
#define BMI08_ACCEL_DATA_SYNC_MODE_400HZ        UINT8_C(0x01)
#define BMI08_ACCEL_DATA_SYNC_MODE_1000HZ       UINT8_C(0x02)
#define BMI08_ACCEL_DATA_SYNC_MODE_2000HZ       UINT8_C(0x03)

/**\name    Absolute value */
#ifndef BMI08_ABS
#define BMI08_ABS(a)                            ((a) > 0 ? (a) : -(a))
#endif

/**\name    Utility Macros  */
#define BMI08_SET_LOW_BYTE                      UINT16_C(0x00FF)
#define BMI08_SET_HIGH_BYTE                     UINT16_C(0xFF00)
#define BMI08_SET_LOW_NIBBLE                    UINT8_C(0x0F)

/**\name Macro to SET and GET BITS of a register */
#define BMI08_SET_BITS(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     ((val << bitname##_POS) & bitname##_MASK))

#define BMI08_GET_BITS(reg_var, bitname)        ((reg_var & (bitname##_MASK)) >> \
                                                 (bitname##_POS))

#define BMI08_SET_BITS_POS_0(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     (val & bitname##_MASK))

#define BMI08_GET_BITS_POS_0(reg_var, bitname)  (reg_var & (bitname##_MASK))

#define BMI08_SET_BIT_VAL_0(reg_var, bitname)   (reg_var & ~(bitname##_MASK))

/**\name     Macro definition for difference between 2 values */
#define BMI08_GET_DIFF(x, y)                    ((x) - (y))

/**\name     Macro definition to get LSB of 16 bit variable */
#define BMI08_GET_LSB(var)                      (uint8_t)(var & BMI08_SET_LOW_BYTE)

/**\name     Macro definition to get MSB of 16 bit variable */
#define BMI08_GET_MSB(var)                      (uint8_t)((var & BMI08_SET_HIGH_BYTE) >> 8)

/*************************************************************************/

/*!
 * @brief Interface selection enums
 */
enum bmi08_intf {
    /*! I2C interface */
    BMI08_I2C_INTF,
    /*! SPI interface */
    BMI08_SPI_INTF
};

/*!
 * @brief Enum to define variants
 */
enum bmi08_variant {
    BMI085_VARIANT = 0,
    BMI088_VARIANT = 1
};

/*************************** Data structures *****************************/

/**\name    Typedef definitions */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[in]     reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef BMI08_INTF_RET_TYPE (*bmi08_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef BMI08_INTF_RET_TYPE (*bmi08_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                                  void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmi08_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/**\name    Structure Definitions */

/*!
 *  @brief Sensor XYZ data structure
 */
struct bmi08_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;
};

/*!
 *  @brief Sensor XYZ data structure in float representation
 */
struct bmi08_sensor_data_f
{
    /*! X-axis sensor data */
    float x;

    /*! Y-axis sensor data */
    float y;

    /*! Z-axis sensor data */
    float z;
};

/*!
 *  @brief Sensor configuration structure
 */
struct bmi08_cfg
{
    /*! power mode */
    uint8_t power;

    /*! range */
    uint8_t range;

    /*! bandwidth */
    uint8_t bw;

    /*! output data rate */
    uint8_t odr;
};

/*!
 *  @brief Error Status structure
 */
struct bmi08_err_reg
{
    /*! Indicates fatal error */
    uint8_t fatal_err;

    /*! Indicates error code */
    uint8_t err_code;
};

/*!
 *  @brief Data Sync config structure
 */
struct bmi08_data_sync_cfg
{
    /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    uint8_t mode;
};

/*!
 *  @brief Enum to select accelerometer Interrupt pins
 */
enum bmi08_accel_int_channel {
    /* interrupt Channel 1 for accel sensor */
    BMI08_INT_CHANNEL_1,
    /* interrupt Channel 2 for accel sensor */
    BMI08_INT_CHANNEL_2
};

/*!
 *  @brief Enum to select gyroscope Interrupt pins
 */
enum bmi08_gyro_int_channel {
    /* interrupt Channel 3 for gyro sensor */
    BMI08_INT_CHANNEL_3,
    /* interrupt Channel 4 for gyro sensor */
    BMI08_INT_CHANNEL_4
};

/*!
 *  @brief Enum to select accelerometer interrupts
 */
enum bmi08_accel_int_types {
    BMI08_ACCEL_INT_DATA_RDY,
    /* Accel data ready interrupt */
    BMI08_ACCEL_INT_SYNC_DATA_RDY,
    /* Accel synchronized data ready interrupt */
    BMI08_ACCEL_SYNC_INPUT,
    /* Accel FIFO watermark interrupt */
    BMI08_ACCEL_INT_FIFO_WM,
    /* Accel FIFO full interrupt */
    BMI08_ACCEL_INT_FIFO_FULL
};

/*!
 *  @brief Enum to select gyroscope interrupts
 */
enum bmi08_gyro_int_types {
    /* Gyro data ready interrupt */
    BMI08_GYRO_INT_DATA_RDY,
    /* Gyro FIFO watermark interrupt */
    BMI08_GYRO_INT_FIFO_WM,
    /* Gyro FIFO full interrupt */
    BMI08_GYRO_INT_FIFO_FULL
};

/*!
 *  @brief Interrupt pin configuration structure
 */
struct bmi08_int_pin_cfg
{
    /*! interrupt pin level configuration
     * Assignable macros :
     * - BMI08_INT_ACTIVE_LOW
     * - BMI08_INT_ACTIVE_HIGH
     */
    uint8_t lvl;

    /*! interrupt pin mode configuration
     * Assignable macros :
     * - BMI08_INT_MODE_PUSH_PULL
     * - BMI08_INT_MODE_OPEN_DRAIN
     */
    uint8_t output_mode;

    /*! Enable interrupt pin
     * Assignable Macros :
     * - BMI08_ENABLE
     * - BMI08_DISABLE
     */
    uint8_t enable_int_pin;
};

/*!
 *  @brief Interrupt channel structure for accel
 */
struct bmi08_accel_int_channel_cfg
{
    /*! Accel Interrupt channel */
    enum bmi08_accel_int_channel int_channel;

    /*! Select Accel Interrupt type */
    enum bmi08_accel_int_types int_type;

    /*! Structure to configure accel interrupt pins */
    struct bmi08_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt channel structure for gyro
 */
struct bmi08_gyro_int_channel_cfg
{
    /*! Gyro Interrupt channel */
    enum bmi08_gyro_int_channel int_channel;

    /*! Select Gyro Interrupt type */
    enum bmi08_gyro_int_types int_type;

    /*! Structure to configure gyro interrupt pins */
    struct bmi08_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt Configuration structure
 */
struct bmi08_int_cfg
{
    /*! Configuration of first accel interrupt channel */
    struct bmi08_accel_int_channel_cfg accel_int_config_1;

    /*! Configuration of second accel interrupt channel */
    struct bmi08_accel_int_channel_cfg accel_int_config_2;

    /*! Configuration of first gyro interrupt channel */
    struct bmi08_gyro_int_channel_cfg gyro_int_config_1;

    /*! Configuration of second gyro interrupt channel */
    struct bmi08_gyro_int_channel_cfg gyro_int_config_2;
};

/*!
 *  @brief Accel Fifo configurations
 */
struct bmi08_accel_fifo_config
{
    /*! Configure the fifo mode (0 = Stream mode, 1 = FIFO mode) */
    uint8_t mode;

    /*! To enable the accel */
    uint8_t accel_en;

    /*! To enable the interrupt_1 */
    uint8_t int1_en;

    /*! To enable the interrupt_2 */
    uint8_t int2_en;
};

/*!
 * @brief Gyro Fifo configurations
 */
struct bmi08_gyr_fifo_config
{
    /*! Configure the fifo mode (0 = Stream mode, 1 = FIFO mode) */
    uint8_t mode;

    /*! Selection of axis for data */
    uint8_t data_select;

    /*! Tag to include/exclude interrupt in FIFO data bytes */
    uint8_t tag;

    /*! Frame count of fifo data */
    uint8_t frame_count;

    /*! Water-mark level for FIFO */
    uint16_t wm_level;
};

/*! @name Structure to define FIFO frame configuration */
struct bmi08_fifo_frame
{
    /*! Pointer to FIFO data */
    uint8_t *data;

    /*! Number of user defined bytes of FIFO to be read */
    uint16_t length;

    /*! Enables type of data to be streamed - accelerometer */
    uint16_t acc_data_enable;

    /*! Enables type of data to be streamed - gyroscope */
    uint16_t gyr_data_enable;

    /*! To index accelerometer bytes */
    uint16_t acc_byte_start_idx;

    /*! To index gyroscope bytes */
    uint16_t gyr_byte_start_idx;

    /*! FIFO sensor time */
    uint32_t sensor_time;

    /*! Skipped frame count */
    uint8_t skipped_frame_count;

    /*! Type of data interrupt to be mapped */
    uint8_t data_int_map;

    /*! FIFO accelerometer configurations */
    struct bmi08_accel_fifo_config acc_fifo_conf;

    /*! FIFO gyroscope configurations */
    struct bmi08_gyr_fifo_config gyr_fifo_conf;
};

/*! @name Structure to store the value of re-mapped axis and its sign */
struct bmi08_axes_remap
{
    /*! Re-mapped x-axis */
    uint8_t x_axis;

    /*! Re-mapped y-axis */
    uint8_t y_axis;

    /*! Re-mapped z-axis */
    uint8_t z_axis;

    /*! Re-mapped x-axis sign */
    uint8_t x_axis_sign;

    /*! Re-mapped y-axis sign */
    uint8_t y_axis_sign;

    /*! Re-mapped z-axis sign */
    uint8_t z_axis_sign;
};

/*!
 *  @brief This structure holds all relevant information about BMI08
 */
struct bmi08_dev
{
    /*! Accel chip Id */
    uint8_t accel_chip_id;

    /*! Gyro chip Id */
    uint8_t gyro_chip_id;

    /*! Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr_accel;

    /*! Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr_gyro;

    /*! Interface Selection
     * For SPI, interface = BMI08_SPI_INTF
     * For I2C, interface = BMI08_I2C_INTF
     **/
    enum bmi08_intf intf;

    /*! Define the BMI08 variant BMI085 or BMI088 */
    enum bmi08_variant variant;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Structure to configure accel sensor  */
    struct bmi08_cfg accel_cfg;

    /*! Structure to configure gyro sensor  */
    struct bmi08_cfg gyro_cfg;

    /*! Structure to maintain a copy of the re-mapped axis */
    struct bmi08_axes_remap remap;

    /*! Config stream data buffer address will be assigned */
    const uint8_t *config_file_ptr;

    /*! Max read/write length
     * To be set by the user */
    uint8_t read_write_len;

    /*! Read function pointer */
    bmi08_read_fptr_t read;

    /*! Write function pointer */
    bmi08_write_fptr_t write;

    /*! Delay function pointer */
    bmi08_delay_us_fptr_t delay_us;

    /*! Variable to store result of read/write function */
    BMI08_INTF_RET_TYPE intf_rslt;
};

#endif /* BMI08_DEFS_H_ */
