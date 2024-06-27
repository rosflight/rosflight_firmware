/**
 ******************************************************************************
 * File     : Mcp4017.h
 * Date     : Mar 18, 2024
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

#ifndef DRIVERS_MCP4017_H_
#define DRIVERS_MCP4017_H_

#define MCP4017_I2C_ADDRESS (0x2F)

#include <misc.h>

/*
 *
 */
class Mcp4017
{
    /**
     * \brief
     *
     *
     */
  public:
    uint32_t init(I2C_HandleTypeDef *hi2c, // The SPI handle
                  uint16_t i2c_address,    // Chip select Port
                  double v                 // voltage
    )
    {
        hi2c_ = hi2c;
        address_ = i2c_address << 1;

        uint32_t status = set(v);
        if (status == DRIVER_OK)
            misc_printf("Servo voltage set to %.1fV, %u counts\n", v_, pot_);
        else
            misc_printf("Servo voltage fail at %.1fV, %u counts\n", v_, pot_);
        return status;
    }

    uint32_t set(double v)
    {
        if (v < 4.8)
            v = 4.8;
        else if (v > 8.2)
            v = 8.2;
        v_ = v;
        double pot = ((-0.82601912 * v_ + 21.03465037) * v_ - 193.55466392) * v_ + 628.13007632;
        if (pot < 0)
            pot = 0;
        else if (pot > 93)
            pot = 93;
        pot_ = pot;
        uint8_t pot_tx = pot_;
        uint8_t pot_rx = ~pot_tx;
        HAL_I2C_Master_Transmit(hi2c_, address_, &pot_tx, 1, 100); // Write 1 bytes of data over I2C
        HAL_I2C_Master_Receive(hi2c_, address_, &pot_rx, 1, 100);  // Write 1 bytes of data over I2C
        if (pot_tx != pot_rx)
            return VOLTAGE_SET_FAIL;
        return DRIVER_OK;
    }

    double v(void)
    {
        return v_;
    }
    uint8_t pot(void)
    {
        return pot_;
    }

  private:
    double v_; // Voltage
    uint8_t pot_;
    I2C_HandleTypeDef *hi2c_; // The SPI handle
    uint16_t address_;        // Chip select Port
};

#endif /* DRIVERS_MCP4017_H_ */
