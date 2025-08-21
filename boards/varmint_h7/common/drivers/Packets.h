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

//#include "Ubx.h"
#include <stdint.h>

#include "board.h"

#define SERIAL_MAX_PAYLOAD_SIZE (256 + 8) // for MAVLINK1, really 255+8, added 1 byte to make it an even multiple of 8
typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  uint16_t qos;
  uint16_t packetSize;
  uint16_t payloadSize;
  uint16_t reserved;
  uint8_t payload[SERIAL_MAX_PAYLOAD_SIZE];
} SerialTxPacket;

typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  double temperature;
  double vBku;
  double vRef;
  double volts[ADC_CHANNELS];
} AdcPacket;

typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  double gyro[3];      // rad/s
  double accel[3];     // rad/s
  double temperature;  // K
  double dataTime;     // s
} ImuPacket;

typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  double pressure;     // Pa
  double temperature;  // K
} PressurePacket;

typedef struct //__attribute__((packed))
{
  rosflight_firmware::PacketHeader header;
  double flux[3];      // T, magnetic flux density
  double temperature;  // K
} MagPacket;

#define RC_PACKET_CHANNELS 24 // 16 analog + 8 digital
typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  uint8_t nChan;
  float chan[RC_PACKET_CHANNELS];
  bool frameLost;
  bool failsafeActivated;
} RcPacket;


//typedef struct GnssStruct GnssPacket;

typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  uint64_t pps;           // most recent pps timestamp (us)
  int64_t unix_seconds;   // Unix time, in seconds
  int32_t unix_nanos;
  uint8_t fix_type;
  uint8_t num_sat;
  double lon;
  double lat;
  float height_msl;
  float vel_n;
  float vel_e;
  float vel_d;
  float h_acc;
  float v_acc;
  float speed_accy;
} GnssPacket;


#endif /* DRIVERPACKETS_H_ */
