/**
 ******************************************************************************
 * File     : Varmint.cpp
 * Date     : Sep 27, 2023
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
#include <Varmint.h>

#include <Time64.h>

#include <ctime>

Varmint varmint;
rosflight_firmware::Board &board = varmint;

extern Time64 time64;

////////////////////////////////////////////////////////////////////////////////////////
//
// Rosflight HAL for Varmint Board
//
////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
// board init

// Note: void Varmint::init_board(void) is in VarmintInit.cpp

///////////////////////////////////////////////////////////////////////////////////////////////
// board reset
void Varmint::board_reset(bool bootloader)
{
    pwm_disable();
    //	HAL_NVIC_SystemReset();
}

///////////////////////////////////////////////////////////////////////////////////////////////
// us clock
uint32_t Varmint::clock_millis()
{
    return time64.Us() / 1000;
}
uint64_t Varmint::clock_micros()
{
    return time64.Us();
}
void Varmint::clock_delay(uint32_t ms)
{
    time64.dMs(ms);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// serial comms to the Companion computer
void Varmint::serial_init(uint32_t baud_rate, uint32_t dev)
{
    serial_device_ = USE_TELEM; // dev; // 1 = telem uart, otherwise = VCP
    if (serial_device_ == 1)
        telem_.reset_baud(baud_rate);
}

void Varmint::serial_write(const uint8_t *src, size_t len, uint8_t qos)
{
    SerialTxPacket p;
    p.timestamp = time64.Us();
    p.payloadSize = len;
    p.qos = qos;

    if (len > (256 + 8))
        len = (256 + 8);
    memcpy(p.payload, src, len);

    if (serial_device_ == 1)
        telem_.writePacket(&p);
    else
        vcp_.writePacket(&p);
}

uint16_t Varmint::serial_bytes_available(void)
{
    if (serial_device_ == 1)
        return telem_.byteCount();
    else
        return vcp_.byteCount();
}

uint8_t Varmint::serial_read(void)
{
    uint8_t c = 0;
    if (serial_device_ == 1)
        telem_.readByte(&c);
    else
        vcp_.readByte(&c);
    return c;
}

void Varmint::serial_flush(void)
{
    // do nothing
}

///////////////////////////////////////////////////////////////////////////////////////////////
//
// Sensors
//
//////////////////////////////////////////////////////////////////////////////////////////////////

void Varmint::sensors_init()
{}

uint16_t Varmint::num_sensor_errors()
{
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// IMU

bool Varmint::imu_has_new_data()
{
    return imu0_.rxFifoCount() > 0;
}
bool Varmint::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
    ImuPacket p;
    if (imu0_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        accel[0] = p.accel[0];
        accel[1] = p.accel[1];
        accel[2] = p.accel[2];
        gyro[0] = p.gyro[0];
        gyro[1] = p.gyro[1];
        gyro[2] = p.gyro[2];
        *temperature = p.temperature;
        *time_us = p.drdy;
        return true;
    }
    return false;
}
void Varmint::imu_not_responding_error(){};
// Do nothing for now

///////////////////////////////////////////////////////////////////////////////////////////////
// MAG
bool Varmint::mag_present()
{
    return true;
}
bool Varmint::mag_has_new_data()
{
    return mag_.rxFifoCount() > 0;
}
bool Varmint::mag_read(float mag[3])
{
    MagPacket p;
    if (mag_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        mag[0] = p.flux[0];
        mag[1] = p.flux[1];
        mag[2] = p.flux[2];
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Baro
bool Varmint::baro_present()
{
    return true;
}
bool Varmint::baro_has_new_data()
{
    return baro_.rxFifoCount() > 0;
}
bool Varmint::baro_read(float *pressure, float *temperature)
{
    PressurePacket p;
    if (baro_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        *pressure = p.pressure;
        *temperature = p.temperature;
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Pitot
bool Varmint::diff_pressure_present()
{
    return true;
}
bool Varmint::diff_pressure_has_new_data()
{
    return pitot_.rxFifoCount() > 0;
}
bool Varmint::diff_pressure_read(float *diff_pressure, float *temperature)
{
    PressurePacket p;
    if (pitot_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        *diff_pressure = p.pressure;
        *temperature = p.temperature;
        return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Sonar
bool Varmint::sonar_present()
{
    return false;
}
bool Varmint::sonar_has_new_data()
{
    return false;
}
bool Varmint::sonar_read(float *range)
{
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Battery
bool Varmint::battery_present()
{
    return true;
}
bool Varmint::battery_has_new_data()
{
    return adc_.rxFifoCount() > 0;
}
bool Varmint::battery_read(float *voltage, float *current)
{
    AdcPacket p;
    if (adc_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        *current = p.volts[ADC_BATTERY_CURRENT];
        *voltage = p.volts[ADC_BATTERY_VOLTS];
        return true;
    }
    return false;
}
void Varmint::battery_voltage_set_multiplier(double multiplier)
{
    if (multiplier == 0)
        return;
    adc_.setScaleFactor(ADC_BATTERY_VOLTS, multiplier);
}
void Varmint::battery_current_set_multiplier(double multiplier)
{
    if (multiplier == 0)
        return;
    adc_.setScaleFactor(ADC_BATTERY_CURRENT, multiplier);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// GNSS
bool Varmint::gnss_present()
{
    return true;
}
bool Varmint::gnss_has_new_data()
{
    return gps_.rxFifoCount() > 0;
}

bool Varmint::gnss_read(rosflight_firmware::GNSSData *gnss, rosflight_firmware::GNSSFull *gnss_full)
{
    UbxPacket p;

    if (gps_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        gnss_full->time_of_week = p.pvt.iTOW;
        gnss_full->year = p.pvt.year;
        gnss_full->month = p.pvt.month;
        gnss_full->day = p.pvt.day;
        gnss_full->hour = p.pvt.hour;
        gnss_full->min = p.pvt.min;
        gnss_full->sec = p.pvt.sec;
        gnss_full->valid = p.pvt.valid;
        gnss_full->t_acc = p.pvt.tAcc;
        gnss_full->nano = p.pvt.nano;
        gnss_full->fix_type = (rosflight_firmware::GNSSFixType)p.pvt.fixType;
        gnss_full->lon = p.pvt.lon;
        gnss_full->lat = p.pvt.lat;
        gnss_full->height = p.pvt.height;
        gnss_full->height_msl = p.pvt.hMSL;
        gnss_full->h_acc = p.pvt.hAcc;
        gnss_full->v_acc = p.pvt.vAcc;
        gnss_full->vel_n = p.pvt.velN;
        gnss_full->vel_e = p.pvt.velE;
        gnss_full->vel_d = p.pvt.velD;
        gnss_full->g_speed = p.pvt.gSpeed;
        gnss_full->head_mot = p.pvt.headMot;
        gnss_full->s_acc = p.pvt.sAcc;
        gnss_full->head_acc = p.pvt.headAcc;
        gnss_full->p_dop = p.pvt.pDOP;
        gnss_full->rosflight_timestamp = p.drdy;

        gnss->time_of_week = p.time.iTOW;
        gnss->nanos = p.time.fTOW;

        gnss->fix_type = (rosflight_firmware::GNSSFixType)p.pvt.fixType;

        struct tm tm;
        tm.tm_sec = p.pvt.sec;
        tm.tm_min = p.pvt.min;
        tm.tm_hour = p.pvt.hour;
        tm.tm_mday = p.pvt.day;
        tm.tm_mon = p.pvt.month - 1;
        tm.tm_year = p.pvt.year - 1900;
        gnss->time = mktime(&tm);
        gnss->lat = p.pvt.lat;
        gnss->lon = p.pvt.lon;
        gnss->height = p.pvt.height;
        gnss->vel_n = p.pvt.velN;
        gnss->vel_e = p.pvt.velE;
        gnss->vel_d = p.pvt.velD;
        gnss->h_acc = p.pvt.hAcc;
        gnss->v_acc = p.pvt.vAcc;

        gnss->ecef.x = p.ecefp.ecefX;
        gnss->ecef.y = p.ecefp.ecefY;
        gnss->ecef.z = p.ecefp.ecefZ;
        gnss->ecef.p_acc = p.ecefp.pAcc;

        gnss->ecef.vx = p.ecefv.ecefVX;
        gnss->ecef.vy = p.ecefv.ecefVY;
        gnss->ecef.vz = p.ecefv.ecefVZ;
        gnss->ecef.s_acc = p.ecefv.sAcc;
        gnss->rosflight_timestamp = p.drdy;
        return true;
    }

    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// RC
void Varmint::rc_init(rc_type_t rc_type){};
bool Varmint::rc_lost()
{
    return rc_.lol();
}

bool Varmint::rc_has_new_data()
{
    return rc_.rxFifoCount() > 0;
}

float Varmint::rc_read(uint8_t chan)
{
    static RcPacket p;
    static bool ever_read = false;

    if (rc_.rxFifoReadMostRecent((uint8_t *)&p, sizeof(p)))
    {
        ever_read = true;
    }

    if ((chan < RC_PACKET_CHANNELS) & ever_read)
        return p.chan[chan];
    return -1; // out of range or no data in p
}

///////////////////////////////////////////////////////////////////////////////////////////////
// PWM

// legacy, all channels are pwm and set to the same 'refresh_rate'
void Varmint::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
    for (int ch = 0; ch < PWM_CHANNELS; ch++)
        pwm_.disable(ch);
    for (int ch = 0; ch < PWM_CHANNELS; ch++)
    {
        pwm_.setRate(ch, refresh_rate);
        if(idle_pwm==0)  pwm_.writeUs(ch, 0); // OFF
        else pwm_.write(ch, 0.0);             // Channel minimum value
    }
    for (int ch = 0; ch < PWM_CHANNELS; ch++)
        pwm_.enable(ch);
}

void Varmint::pwm_init_multi(const float *rate, uint32_t channels)
{
	pwm_.updateConfig(rate,channels);
}

void Varmint::pwm_disable(void)
{
	for (int ch = 0; ch < PWM_CHANNELS; ch++)
        pwm_.disable(ch);
}
void Varmint::pwm_write(uint8_t ch, float value)
{
    pwm_.write(ch, value);
}

void Varmint::pwm_write_multi(float *value, uint32_t channels)
{
	pwm_.write(value, channels);
}



///////////////////////////////////////////////////////////////////////////////////////////////
// LEDs
void Varmint::led0_on() // Red LED
{
    RED_HI;
}
void Varmint::led0_off()
{
    RED_LO;
}
void Varmint::led0_toggle()
{
    RED_TOG;
}

void Varmint::led1_on()
{
    BLU_HI;
}
void Varmint::led1_off()
{
    BLU_LO;
}
void Varmint::led1_toggle()
{
    BLU_TOG;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Backup Data (Register and SRAM)
// https://stackoverflow.com/questions/20667754/how-to-use-backup-sram-as-eeprom-in-stm32f4
// from \Drivers\CMSIS\Device\ST\STM32H7xx\Include\stm32h753xx.h(2141)
//#define D3_BKPSRAM_BASE           (0x38800000UL) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge */
//#define D3_SRAM_BASE              (0x38000000UL) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge

#define D3_BKPSRAM_BASE_LEN (4096U)
#define D3_SRAM_BASE_LEN (65536U)
void Varmint::backup_memory_init()
{}
bool Varmint::backup_memory_read(void *dest, size_t len)
{
    //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
    //	HAL_PWR_EnableBkUpAccess();
    //	memcpy(dest, (void*)D3_BKPSRAM_BASE, len);
    //	HAL_PWR_DisableBkUpAccess();
    return true;
}
void Varmint::backup_memory_write(const void *src, size_t len)
{
    //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
    //	HAL_PWR_EnableBkUpAccess();
    //	memcpy((void*)D3_BKPSRAM_BASE, src, len);
    //	HAL_PWR_DisableBkUpAccess();
}
void Varmint::backup_memory_clear(size_t len)
{
    //	HAL_PWR_EnableBkUpAccess();
    //	memset((void*)D3_BKPSRAM_BASE, 0, D3_BKPSRAM_BASE_LEN);
    //	HAL_PWR_DisableBkUpAccess();
}

void Varmint::memory_init()
{} // do nothing

bool Varmint::memory_read(void *dest, size_t len)
{
    return sd_.read((uint8_t *)dest, len);
}
bool Varmint::memory_write(const void *src, size_t len)
{
    return sd_.write((uint8_t *)src, len);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions (not part of parent class)
