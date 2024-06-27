/**
 ******************************************************************************
 * File     : MS4525.cpp
 * Date     : May 28, 2024
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
#include <Ms4525.h>
#include <Time64.h>
#include <misc.h>
extern Time64 time64;

#define MS4525_OK (0x0000)

__attribute__((section("my_dma_buffers")))
__attribute__((aligned(32))) static uint8_t ms4525_i2c_dma_buf[I2C_DMA_MAX_BUFFER_SIZE] = {0};
__attribute__((section("my_buffers")))
__attribute__((aligned(32))) static uint8_t ms4525_fifo_rx_buffer[MS4525_FIFO_BUFFERS * sizeof(PressurePacket)] = {0};

#define MS4525_I2C_DMA_SIZE (4)

uint32_t Ms4525::init(
    // Driver initializers
    uint16_t sample_rate_hz,
    // I2C initializers
    I2C_HandleTypeDef *hi2c, // The SPI handle
    uint16_t i2c_address     // Chip select Port
)
{
    uint32_t status = DRIVER_OK;
    sampleRateHz_ = sample_rate_hz;

    hi2c_ = hi2c;
    address_ = i2c_address << 1;
    launchUs_ = 0;
    // groupDelay_ = 0; //Computed later based on launchUs_ and drdy_ timestamps.

    rxFifo_.init(MS4525_FIFO_BUFFERS, sizeof(PressurePacket), ms4525_fifo_rx_buffer);

    dtMs_ = 1000. / (double)sampleRateHz_;

    // Read the status register
    uint8_t sensor_status[2];

    HAL_StatusTypeDef i2cstatus =
        HAL_I2C_Master_Receive(hi2c_, address_, sensor_status, 2, 1000); // Receive 1 bytes of data over I2C

    misc_printf("MS4525 Status = 0x%02X - ", (sensor_status[0] >> 6) & 0x0003);
    if (i2cstatus == HAL_OK)
        misc_printf("OK\n");
    else
    {
        misc_printf("ERROR\n");
        status |= DRIVER_SELF_DIAG_ERROR;
    }

    misc_printf("\n");

    return status;
}

PollingState Ms4525::state(uint64_t poll_counter)
{
    uint32_t rollover = 10000; // us (100 Hz) 0-99 slots at 10 kHz
    PollingStateStruct lut[] = // BARO at 50 Hz
        {
            // at 10kHz, each count is 100us. I2C is 90us pre byte
            {0, MS4525_CMDRX}, // addr + 4 bytes
        };
    return PollingStateLookup(lut, sizeof(lut) / sizeof(PollingStateStruct),
                              poll_counter % (rollover / POLLING_PERIOD_US));
}

bool Ms4525::poll(uint64_t poll_counter)
{
    PollingState poll_state = state(poll_counter);
    if (poll_state == MS4525_CMDRX)
    {
        launchUs_ = time64.Us();
        if ((dmaRunning_ =
                 (HAL_OK == HAL_I2C_Master_Receive_DMA(hi2c_, address_, ms4525_i2c_dma_buf,
                                                       MS4525_I2C_DMA_SIZE)))) // Receive 7 bytes of data over I2C
            i2cState_ = poll_state;
        else
            i2cState_ = MS4525_ERROR;
    }
    return dmaRunning_;
}

void Ms4525::endDma(void)
{
    if (i2cState_ == MS4525_CMDRX)
    {
        PressurePacket p;
        p.status = ms4525_i2c_dma_buf[0] & 0xC0;
        if (p.status == 0x00)
        {
            p.timestamp = time64.Us();
            p.drdy = p.timestamp;
            p.groupDelay = (p.timestamp - launchUs_);
            uint32_t i_pressure = (uint32_t)ms4525_i2c_dma_buf[0] << 8 | (uint32_t)ms4525_i2c_dma_buf[1];
            uint32_t i_temperature =
                ((uint32_t)ms4525_i2c_dma_buf[2] << 3 | (uint32_t)ms4525_i2c_dma_buf[3] >> 5) & 0x7FF;

            double pmax = 6894.76; // (=-pmin) Pa

            p.status = (ms4525_i2c_dma_buf[0] >> 6) & 0x0003;
            p.pressure = ((double)i_pressure / 8192.0 - 1.0) * 1.25 * pmax;         // Pa
            p.temperature = (double)i_temperature * 200.0 / 2047.0 - 50.0 + 273.15; // K

            if (p.status == MS4525_OK)
                rxFifo_.write((uint8_t *)&p, sizeof(p));
        }
    }
    i2cState_ = IDLE_STATE;
    dmaRunning_ = false;
}

bool Ms4525::display(void)
{
    PressurePacket p;
    char name[] = "MS4525 (pitot)";
    if (rxFifo_.readMostRecent((uint8_t *)&p, sizeof(p)))
    {
        misc_header(name, p.drdy, p.timestamp, p.groupDelay);
        misc_printf("%10.3f Pa                          |                                        | %7.1f C |           "
                    "   | 0x%04X\n",
                    p.pressure, p.temperature - 273.15, p.status);
        return 1;
    }
    else
    {
        misc_printf("%s\n", name);
    }
    return true;
}
