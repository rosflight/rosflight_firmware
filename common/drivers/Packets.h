/**
 ******************************************************************************
 * File     : DriverPackets.h
 * Date     : Sep 21, 2023
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

#ifndef DRIVERPACKETS_H_
#define DRIVERPACKETS_H_

#include <Ubx.h>
#include <stdint.h>

#define SERIAL_MAX_PAYLOAD_SIZE (256 + 8) // for MAVLINK1, really 255+8, added 1 byte to make it an even multiple of 8
typedef struct __attribute__((__packed__))
{
    uint64_t timestamp; // us, time of data read complete
    uint16_t qos;
    uint16_t packetSize;
    uint16_t payloadSize;
    uint16_t reserved;
    uint8_t payload[SERIAL_MAX_PAYLOAD_SIZE];
} SerialTxPacket;

typedef struct __attribute__((__packed__))
{
    uint64_t timestamp;  // us, time of data read complete
    uint64_t drdy;       // us, time of drdy signal (group delay is often known relative to this time)
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    double temperature;
    double vBku;
    double vRef;
    double volts[ADC_CHANNELS];
} AdcPacket;

typedef struct __attribute__((__packed__))
{
    uint64_t timestamp;  // us, time of data read complete
    uint64_t drdy;       // us, time of drdy signal (group delay is often known relative to this time)
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    uint16_t status;     // sensor specific
                         //
    double gyro[3];      // rad/s
    double accel[3];     // rad/s
    double temperature;  // K
    double dataTime;     // s
} ImuPacket;

typedef struct __attribute__((__packed__))
{
    uint64_t timestamp;  // us, time of data read complete
    uint64_t drdy;       // us, time of drdy signal (group delay is often known relative to this time)
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    uint16_t status;     // sensor specific
                         //
    double pressure;     // Pa
    double temperature;  // K
} PressurePacket;

typedef struct __attribute__((packed))
{
    uint64_t timestamp;  // us, time of data read complete
    uint64_t drdy;       // us, time of drdy signal (group delay is often known relative to this time)
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    uint16_t status;     // sensor specific
                         //
    double flux[3];      // T, magnetic flux density
    double temperature;  // K
} MagPacket;

#define RC_PACKET_CHANNELS 24 // 16 analog + 8 digital
typedef struct __attribute__((__packed__))
{
    uint64_t timestamp;  // us, time of data read complete
    uint64_t drdy;       // us, time of drdy signal (group delay is often known relative to this time)
    uint64_t groupDelay; // us, time from measurement to drdy, (approximate!)
    uint16_t status;     // sensor specific
                         //
    uint8_t nChan;
    float chan[RC_PACKET_CHANNELS];
    bool frameLost;
    bool failsafeActivated;
} RcPacket;

#endif /* DRIVERPACKETS_H_ */
