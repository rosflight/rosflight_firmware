/**
 ******************************************************************************
 * File     : rosflight_structs.h
 * Date     : Nov 20, 2024
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

#ifndef INCLUDE_INTERFACE_ROSFLIGHT_STRUCTS_H_
#define INCLUDE_INTERFACE_ROSFLIGHT_STRUCTS_H_

#include <cstdint>

namespace rosflight_firmware
{

typedef struct __attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  uint16_t status;    // device dependent
} PacketHeader;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float voltage;
  float current;
  float temperature; // STM32 temperature, not batter temperature
} BatteryStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float accel[3];    // rad/s
  float gyro[3];     // rad/s
  float temperature; // K
} ImuStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float pressure;    // Pa
  float temperature; // K
  union
  {
    float altitude;
    float ias; //speed;
  };
} PressureStruct;

enum class SensorRangeType // c.f., ROSFLIGHT_RANGE_TYPE
{
  ROSFLIGHT_RANGE_SONAR = 0, /*  | */
  ROSFLIGHT_RANGE_LIDAR = 1, /*  | */
  END = 2,                   /*  | */
};

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float range;          // m
  float min_range;      // m
  float max_range;      // m
  SensorRangeType type; // ROSFLIGHT_RANGE_SONAR, ROSFLIGHT_RANGE_SONAR
} RangeStruct;

typedef struct //__attribute__((packed))
{
  PacketHeader header; //
  float flux[3];       // T, magnetic flux density
  float temperature;   // K
} MagStruct;

enum class GNSSFixType // quality from GGA
{
  GNSS_FIX_TYPE_NO_FIX = 0,
  GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1,
  GNSS_FIX_TYPE_2D_FIX = 2,
  GNSS_FIX_TYPE_3D_FIX = 3,
  GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4,
  GNSS_FIX_TYPE_TIME_FIX_ONLY = 5,
  GNSS_FIX_RTK_FLOAT = 6,
  GNSS_FIX_RTK_FIXED = 7,
  END = 8
};

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;    //
  uint64_t pps;           // most recent pps timestamp (us)
  int64_t unix_seconds;   // Unix time, in seconds (redundant)
  int32_t unix_nanos;     // GGA RMC UTC Time (ms) / PVT nano (this and unix_seconds adjusted so unitx_nanos is positive)
  uint8_t fix_type;       // RMC (posmode), compute from GGA(quality) /PVT flags
  uint8_t num_sat;        // GGA
  double lon;            // GGA RMC / PVT
  double lat;            // GGA RMC / PVT
  float height_msl;     // GGA / PVT
  float vel_n;          // RMC / PVT (RMC compute from ground velocity)
  float vel_e;          // RMC / PVT (RMC compute from ground velocity)
  float vel_d;          // no / PVT
  float h_acc;         // GST (lat and lon) / PVT hAcc
  float v_acc;         // GST / PVT vAcc
  float speed_accy;    // no /PVT (sACC) speed accuracy
} GnssStruct;

typedef struct
{
  bool imu;
  bool gnss;
  bool baro;
  bool mag;
  bool diff_pressure;
  bool sonar;
  bool battery;
} got_flags;

// 16 analog + 8 digital MUST BE > 14 (Mavlink message size is hardware to 14)
#define RC_STRUCT_CHANNELS 24
typedef struct //__attribute__((packed))
{
  PacketHeader header;
  uint8_t nChan;
  float chan[RC_STRUCT_CHANNELS];
  bool frameLost;
  bool failsafeActivated;
} RcStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float q[4]; // quaternions
  float rate[3];
} AttitudeStruct;

} // namespace rosflight_firmware

#endif /* INCLUDE_INTERFACE_ROSFLIGHT_STRUCTS_H_ */
