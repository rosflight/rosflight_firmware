/**
 ******************************************************************************
 * File     : Ubx.h
 * Date     : Oct 2, 2023
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

#ifndef UBX_H_
#define UBX_H_

#include <BoardConfig.h>
#include <Driver.h>

typedef enum
{
    UBX_M8,
    UBX_M9
} UbxProtocol;

// Class 0x01, ID 0x07
typedef struct __attribute__((__packed__)) // This matches the Ubx packet, do not modify
{
    uint32_t iTOW;                      // ms, GPS time of week
    uint16_t year;                      // UTC
    uint8_t month, day, hour, min, sec; // UTC
    uint8_t valid;                      // validity flags
                                        //		uint8_t :4;
                                        //		uint8_t validMag :1;
                                        //		uint8_t fullyResolved :1;
                                        //		uint8_t validTime :1;
                                        //		uint8_t validDate :1;
    uint32_t tAcc;                      // ns, time accuracy estimate
    int32_t nano;                       // ns, Fraction of second -1e9 to 1e9 (UTC)
    uint8_t fixType; // 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNS+dead reckoning combined, 5 time only fix
    uint8_t flags;
    //		uint8_t carrSoln :2;
    //		uint8_t headVehValid :1;
    //		uint8_t psmState:3;
    //		uint8_t diffSoln:1;
    //		uint8_t gnssFixOK :1;
    uint8_t flags2;
    //		uint8_t confirmedTime:1;
    //		uint8_t confirmedDate:1;
    //		uint8_t confirmedAvai:1;
    //		uint8_t :5;
    uint8_t numSV;                    // satellites used in solution
    int32_t lon, lat;                 // degx10^-7
    int32_t height, hMSL;             // mm
    uint32_t hAcc, vAcc;              // mm
    int32_t velN, velE, velD, gSpeed; // mm/s veloxity
    int32_t headMot;                  // degx10^-5
    uint32_t sAcc;                    // mm/s speed accuracy estimate
    uint32_t headAcc;                 // degx10^-5
    uint16_t pDOP;                    // 0.01 (percent)
                                      //	uint16_t flags3;
    uint16_t : 11;
    uint16_t lastCorrectionAge : 4;
    uint16_t invalidLlh : 1;
    uint8_t reserved1;
    int32_t headVeh; // degx10^-5, vehicle heading
    int16_t magDec;  // degx 10^-2
    uint16_t magAcc; // degx 10^-2
} UbxPvt;

// Class 0x01, ID 0x20
typedef struct __attribute__((__packed__)) // This matches the Ubx packet, do not modify
{
    uint32_t iTOW; // ms, GPS time of week
    int32_t fTOW;  // ns, (iTOW * 1e-3) + (fTOW * 1e-9) seconds
    int16_t week;  // GPS week number
    int8_t leapS;
    uint8_t valid;
    uint32_t tAcc;
} UbxTime;

// Class 0x01, ID 0x01
typedef struct __attribute__((__packed__)) // This matches the Ubx packet, do not modify
{
    uint32_t iTOW; // ms, GPS time of week
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
    uint32_t pAcc;
} UbxEcefPos;

// Class 0x01, ID 0x11
typedef struct __attribute__((__packed__)) // This matches the Ubx packet, do not modify
{
    uint32_t iTOW; // ms, GPS time of week
    int32_t ecefVX;
    int32_t ecefVY;
    int32_t ecefVZ;
    uint32_t sAcc;
} UbxEcefVel;

#define UBX_MAX_PAYLOAD_BYTES (256)
typedef struct __attribute__((__packed__))
{
    uint8_t cl, id;
    uint16_t length;
    uint8_t A, B;
    uint8_t payload[UBX_MAX_PAYLOAD_BYTES];
} UbxFrame;

typedef struct __attribute__((__packed__)) // This matches the Ubx packet, do not modify
{
    uint64_t timestamp;
    uint64_t drdy;
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    uint64_t pps;
    UbxPvt pvt;
    //	UbxNav 		nav;
    UbxTime time;
    UbxEcefPos ecefp;
    UbxEcefVel ecefv;
} UbxPacket;

/**
 * @class Ubx
 * @brief
 *
 */

class Ubx : public Driver
{
    /**
     * \brief
     *
     *
     */
  public:
    uint32_t init(
        // Driver initializers
        uint16_t sample_rate_hz, GPIO_TypeDef *drdy_port, uint16_t drdy_pin, bool has_pps,
        // UART initializers
        UART_HandleTypeDef *huart, USART_TypeDef *huart_instance, DMA_HandleTypeDef *hdma_uart_rx,
        uint32_t baud_desired, UbxProtocol ubx_protocol);

    bool poll(void);
    void endDma(void);
    bool startDma(void);
    bool display(void) override;
    bool parseByte(uint8_t c, UbxFrame *p);
    UART_HandleTypeDef *huart(void)
    {
        return huart_;
    }

    bool isMy(uint16_t exti_pin)
    {
        return drdyPin_ == exti_pin;
    }
    bool isMy(UART_HandleTypeDef *huart)
    {
        return huart_ == huart;
    }

    void pps(uint64_t pps_timestamp);

  private:
    UbxPacket ubx_;
    uint64_t gotPvt_, gotTime_, gotEcefP_, gotEcefV_; //, gotNav_;
    uint64_t dtimeout_;
    UART_HandleTypeDef *huart_;
    DMA_HandleTypeDef *hdmaUartRx_;
    uint32_t baud_, baud_initial_;
    UbxProtocol ubxProtocol_;
    bool hasPps_;

    void checksum(uint8_t *buffer);
    void header(uint8_t *buffer, uint8_t cl, uint8_t id, uint16_t length);
    HAL_StatusTypeDef tx(uint8_t *buffer, uint32_t length);

    uint16_t cfgMsg(uint8_t cl, uint8_t id, uint8_t decimation_rate);

    // M8 Variants Depricated in M9
    uint16_t cfgPrt(uint32_t baud);
    uint16_t cfgRate(uint32_t hz);
    uint16_t cfgTp5(uint32_t hz);
    uint16_t cfgNav5(void);
    uint32_t pollBaud(void);

    // M9 Variants
    uint16_t cfgM9(uint32_t baud, uint16_t sampleRateHz);
    uint32_t pollBaudM9(void);
};

#endif /* UBX_H_ */
