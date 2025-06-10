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
  PacketHeader header; //
  uint64_t pps;        // most recent pps timestamp
  int64_t unix_seconds;       // Unix time, in seconds (redundant)
  // GPS Time
  uint32_t time_of_week; //     / PVT
  uint16_t year;         // RMC / PVT
  uint8_t month;         // RMC / PVT
  uint8_t day;           // RMC / PVT
  uint8_t hour;          // GGA RMC UTC Time / PVT
  uint8_t min;           // GGA RMC UTC Time / PVT
  uint8_t sec;           // GGA RMC UTC Time / PVT
  int32_t nano;         // GGA RMC UTC Time (ms) / PVT nano
  uint32_t t_acc;
  int32_t lon;              // GGA RMC / PVT
  int32_t lat;              // GGA RMC / PVT
  int32_t height_ellipsoid; //GGA RMC (computed) / PVT
  int32_t height_msl;       // GGA / PVT
  uint32_t h_acc;           // GST (lat and lon) / PVT hAcc
  uint32_t v_acc;           // GST / PVT vAcc
  int32_t ground_speed;     // RMC / PVT gSpeed
  int32_t course;           // RMC / PVT headMot
  int32_t course_accy;
  int32_t vel_n;       // no / PVT (RMC compute from ground velocity)
  int32_t vel_e;       // no / PVT (RMC compute from ground velocity)
  int32_t vel_d;       // no / PVT
  uint32_t speed_accy; // no /PVT (sACC) speed accuracy
  uint32_t mag_var;    // RMC / PVT
  // Fix
  uint8_t fix_type; // RMC (posmode), compute from GGA(quality) /PVT flags
  uint8_t valid;    // RMC (status), compute from GGA (0 or 6)
  uint8_t num_sat;  // GGA
  uint16_t dop;     // GGA RMC / PVT (pdop)
  struct
  {
    int32_t x;      // cm // not available on NMEA
    int32_t y;      // cm
    int32_t z;      // cm
    uint32_t p_acc; // cm
    int32_t vx;     // cm/s
    int32_t vy;     // cm/s
    int32_t vz;     // cm/s
    uint32_t s_acc; // cm/s
  } ecef;
} GnssStruct;

typedef GnssStruct GNSSData;
typedef GnssStruct GNSSFull;


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
