/**
 ******************************************************************************
 * File     : Ubx.cpp
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

#include "Ubx.h"

#include "Time64.h"

#include "Packets.h"
#include "misc.h"
#include <ctime>
extern Time64 time64;

#define SET(buf, data, type) *((type *) (buf)) = data

#define UBX_DMA_BUFFER_SIZE 16 * 2 // must be multiple of 16
DMA_RAM uint8_t ubx_dma_rxbuf[UBX_DMA_BUFFER_SIZE];

DTCM_RAM uint8_t ubx_double_buffer[2 * sizeof(GnssPacket)];

void Ubx::pps(uint64_t pps_timestamp)
{
  static bool first_time = true;
  if(!first_time) gnss_.pps = pps_timestamp;
  first_time = false;
}


uint32_t Ubx::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * pps_port, uint16_t pps_pin,
  // UART initializers
  UART_HandleTypeDef * huart, USART_TypeDef * huart_instance, DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Ubx");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  ppsPin_ = pps_pin;
  ppsHz_ = 1; // To match top of second.

  dtimeout_ = 1000000; // 1 seconds
  timeout_ = 0;

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx; // huart->hdmarx;

  baud_initial_ = 9600;
  baud_ = baud;

  gnss_.pps = 0;

  // gotNav_ = false;
  gotPvt_ = 0;

  // USART initialization
  huart_->Instance = huart_instance;
  huart_->Init.BaudRate = baud_initial_;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.StopBits = UART_STOPBITS_1;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_->Init.OverSampling = UART_OVERSAMPLING_16;
  huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  //  huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  //  huart_->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(huart_) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  if (HAL_UARTEx_SetTxFifoThreshold(huart_, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  if (HAL_UARTEx_SetRxFifoThreshold(huart_, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  //    if (HAL_UARTEx_DisableFifoMode (huart_) != HAL_OK)
  //        return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_EnableFifoMode(huart_) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  // New Sync Baudrate

   uint32_t bauds[] = {9600, 57600, 115200, 230400}; // { 9600, 19200, 38400, 57600, 115200, 230400, 460800};
   unsigned int i, retry;
   uint32_t ubx_baud = 0;

   uint8_t ack[10] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x37};
   for(retry=0;retry<2;retry++)
   {
     uint8_t offset = 0;
     for(i=0;i<sizeof(bauds)/sizeof(uint32_t);i++)
     {
       misc_printf("Sending with baud %u\n",bauds[i]);
       // Set STM Baud
         huart_->Init.BaudRate = bauds[i];
         if (HAL_UART_Init(huart_) != HAL_OK) {
           initializationStatus_ |= DRIVER_HAL_ERROR;
           return initializationStatus_;
         }
         // Send UBLOX Baud configuration message
         cfgPrt(baud_);

         // Look for Ack message
         offset = 0; // rest to start of packet
         for(int j=0;j<128;j++)
         {
           uint8_t ch = 0;
           //HAL_StatusTypeDef hal_status =
               HAL_UART_Receive(huart_, &ch, 1, 1);
           if(ch == ack[offset]) offset++;
           else offset = 0;

           if(offset==10) // we have a match
           {
             misc_printf("ubx characters read till match %u, baud tx %u\n",j+1,bauds[i]);
             ubx_baud = bauds[i];
             break;
           }
         }
         if(offset==10) break;
     }
     if(offset==10) break;
      misc_printf("retry %u\n",retry+1);
   }




  if (ubx_baud != baud_) {
    initializationStatus_ |= UBX_FAIL_BAUD_CHANGE;
    return initializationStatus_;
  }

  misc_printf("Baud Rate Set to   = %6lu, %u retries\n", 100000000 / huart_->Instance->BRR, retry);

  huart_->Init.BaudRate = baud_;
  if (HAL_UART_Init(huart_) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  double_buffer_.init(ubx_double_buffer, sizeof(ubx_double_buffer) );

  // Disable these messages to get rid of clutter
  uint16_t error = 0;

  // Disable these messages
  error |= (uint16_t) cfgMsg(0x0A, 0x09, 0); // MON-HW
  error |= (uint16_t) cfgMsg(0x0A, 0x0B, 0); // MON-HW2
  error |= (uint16_t) cfgMsg(0x01, 0x04, 0); // NAV-DOP
  error |= (uint16_t) cfgMsg(0x01, 0x03, 0); // NAV-STATUS
  error |= (uint16_t) cfgMsg(0x01, 0x35, 0); // NAV-SAT
  error |= (uint16_t) cfgMsg(0x01, 0x20, 0); // NAV-TIMEGPS
  error |= (uint16_t) cfgMsg(0x01, 0x01, 0); // NAV-POSECEF (length 20)
  error |= (uint16_t) cfgMsg(0x01, 0x11, 0); // NAV-VELECEF (length 20)
  error |= (uint16_t) cfgMsg(0x01, 0x20, 0); // NAV-TIMEGPS (length 16)
  error |= (uint16_t) cfgMsg(0x01, 0x01, 0); // NAV-POSECEF (length 20)
  error |= (uint16_t) cfgMsg(0x01, 0x11, 0); // NAV-VELECEF (length 20)

  // Enable these messages
  //error |= (uint16_t) cfgMsg(0x01, 0x20, 1); // NAV-TIMEGPS (length 16)
  error |= (uint16_t) cfgMsg(0x01, 0x07, 1); // NAV-PVT (length 92)
  //error |= (uint16_t) cfgMsg(0x01, 0x01, 1); // NAV-POSECEF (length 20)
  //error |= (uint16_t) cfgMsg(0x01, 0x11, 1); // NAV-VELECEF (length 20)

  // Set GPS Configuration (already done in pollCfgPrtM9() for UBX_M9)
  error |= (uint16_t) cfgRate(sampleRateHz_); // Nav rate 0x06 0x08
  error |= (uint16_t) cfgTp5(ppsHz_);  // PPS rate 0x06 0x31
  error |= (uint16_t) cfgNav5();              // airplane mode 0x06 0x24


  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);

  return initializationStatus_;
}

bool Ubx::startDma(void)
{
  timeout_ = time64.Us() + dtimeout_;
  HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(huart_, ubx_dma_rxbuf, UBX_DMA_BUFFER_SIZE);
  return HAL_OK == hal_status;
}

void Ubx::endDma(void)
{
  uint16_t bytes_in_dma_buffer = misc_bytes_in_dma(hdmaUartRx_, UBX_DMA_BUFFER_SIZE);

  static UbxFrame p;

  for (int i = 0; i < bytes_in_dma_buffer; i++) {
    bool found = parseByte(ubx_dma_rxbuf[i], &p);

    if (found) {
      if ((p.cl == 0x01) && (p.id == 0x07)) {
        UbxPvt pvt;
        memcpy((uint8_t *) &pvt, p.payload, sizeof(pvt));

        // Convert UbxPvt to GnssPacket
        struct tm tm;
        tm.tm_sec  = pvt.sec;
        tm.tm_min  = pvt.min;
        tm.tm_hour = pvt.hour;
        tm.tm_mday = pvt.day;
        tm.tm_mon  = pvt.month - 1;
        tm.tm_year = pvt.year - 1900;
        gnss_.unix_seconds = mktime(&tm);
        gnss_.unix_nanos = pvt.nano;

        if (pvt.nano<0)
        {
          gnss_.unix_seconds--;
          gnss_.unix_nanos += 1000000000;
        }

        gotPvt_ = time64.Us();
        if ( (gnss_.pps !=0) && (gnss_.pps < gotPvt_) && ((pvt.valid & 0x07)== 0x07) && ((pvt.flags & 0x01)==0x01) )
        {
           if(pvt.nano<0)
           {
             gnss_.header.timestamp  = gnss_.pps - (uint64_t)(-pvt.nano/1000);
           } else {
             gnss_.header.timestamp  = gnss_.pps + (uint64_t)(pvt.nano/1000);
          }
        } else {
          gnss_.header.timestamp = gotPvt_-22000; //
        }

        gnss_.header.complete = gotPvt_;

        // Convert Ubx-specific units to standard units
        gnss_.fix_type = pvt.fixType;
        gnss_.num_sat = pvt.numSV;
        gnss_.lon = (double)pvt.lon * 1e-7; // 100s of nanodegs to deg
        gnss_.lat = (double)pvt.lat * 1e-7; // 100s of nanodegs to deg
        gnss_.height_msl = (float)pvt.hMSL * 1e-3; // mm to m
        gnss_.vel_n = (float)pvt.velN * 1e-3; // mm/s to m/s
        gnss_.vel_e = (float)pvt.velE * 1e-3; // mm/s to m/s
        gnss_.vel_d = (float)pvt.velD * 1e-3; // mm/s to m/s
        gnss_.h_acc = (float)pvt.hAcc * 1e-3; // mm to m
        gnss_.v_acc = (float)pvt.vAcc * 1e-3; // mm to m
        gnss_.speed_accy = (float)pvt.sAcc * 1e-3; // mm/s to m/s

        write((uint8_t *) &gnss_, sizeof(gnss_));
        gotPvt_ = 0;
      }
    }
  }

  startDma(); // restart the DMA
}

bool Ubx::parseByte(uint8_t c, UbxFrame * p)
{
  static uint16_t n = 0;

  // special case where we get 0xB5 randomly duplicated at the start (DMA wierdness).
  if ((c == 0xB5) & (n == 1)) n = 0;

  if (n == 0) // header byte 1 "mu" character
  {
    if (c == 0xB5) n++;
    else n = 0;
    p->A = 0;
    p->B = 0;        // Reset the checksum calculation
  } else if (n == 1) // header byte 2
  {
    if (c == 0x62) n++;
    else if (c == 0xB5) n = 1; // repeated 'mu'
    else n = 0;
  } else if (n == 2) // Class
  {
    if ((c == 0x01) || (c == 0x05) || (c == 0x06)) // NAV, CFG, and ACK messages.
    {
      p->cl = c;
      n++;
      p->A += c;
      p->B += p->A;
    } else n = 0;
  } else if (n == 3) // ID, allow all
  {
    p->id = c;
    n++;
    p->A += c;
    p->B += p->A;

  } else if (n == 4) // length LSB
  {
    p->length = (uint16_t) c;
    n++;
    p->A += c;
    p->B += p->A;
  } else if (n == 5) // length MSB
  {
    p->length |= ((uint16_t) c) << 8;
    if (p->length > UBX_MAX_PAYLOAD_BYTES) n = 0;
    else {
      n++;
      p->A += c;
      p->B += p->A;
    }
  } else if (n < p->length + 6) // Packet Payload bytes and first byte of checksum.
  {
    p->payload[n - 6] = c;
    n++;
    p->A += c;
    p->B += p->A;
  } else if (n == p->length + 6) // Checksum A
  {
    if (p->A != c) n = 0;
    else n++;
  } else // if(n==p->length+7) // Checksum B (the end)
  {
    n = 0;
    if (p->B == c) { return true; }
  }
  return false;
}



///////////////////////////////////////////////////////////////////////////////
// Packet Stuff

void Ubx::checksum(uint8_t * buffer)
{
  uint8_t a = 0, b = 0;
  uint16_t n = ((((uint16_t) buffer[5]) << 8) | (uint16_t) buffer[4]) + 6;
  for (int16_t i = 2; i < n; i++) {
    a += buffer[i];
    b += a;
  }
  buffer[n] = a;
  buffer[n + 1] = b;
}

void Ubx::header(uint8_t * buffer, uint8_t cl, uint8_t id, uint16_t length)
{
  buffer[0] = 0xB5; // mu
  buffer[1] = 0x62; // b
  buffer[2] = cl;   // Class
  buffer[3] = id;   // ID
  SET(buffer + 4, length, uint16_t);
}

HAL_StatusTypeDef Ubx::tx(uint8_t * buffer, uint32_t length)
{
  checksum(buffer);
  HAL_StatusTypeDef hal_status = HAL_UART_Transmit(huart_, buffer, length + 8, 0xFFFFFFFF);
  return hal_status;
}

uint32_t Ubx::pollBaud(void)
{
#define POLL_BUFFER_SIZE 128
#define POLL_CFG_PRT_LENGTH 1
  // uint8_t message[CFG_PRT_LENGTH+8]={0};
  uint8_t message[POLL_BUFFER_SIZE] = {0};
  uint8_t * payload = message + 6;
  header(message, 0x06, 0x00, POLL_CFG_PRT_LENGTH);
  payload[0] = 0x01; // Port 1 is the UART

  tx(message, POLL_CFG_PRT_LENGTH);

  UbxFrame ubx;
  memset(&ubx, 0, sizeof(ubx));

  for (int i = 0; i < 256; i++) {
    uint8_t ch;
    HAL_StatusTypeDef hal_status = HAL_UART_Receive(huart_, &ch, 1, 1000);

    if (hal_status == HAL_OK)
      if (parseByte(ch, &ubx)) {
        if ((ubx.cl == 0x06) && (ubx.id == 0x00)) {
          uint32_t baud = (uint32_t) ubx.payload[11] << 24 | (uint32_t) ubx.payload[10] << 16
            | (uint32_t) ubx.payload[9] << 8 | (uint32_t) ubx.payload[8];
          return baud;
        } else {
          memset(&ubx, 0, sizeof(ubx));
        }
      }
  }
  return 0;
}

uint16_t Ubx::cfgPrt(uint32_t baud)
{
#define CFG_PRT_LENGTH 20
  uint8_t message[CFG_PRT_LENGTH + 8] = {0};
  uint8_t * payload = message + 6;

  header(message, 0x06, 0x00, CFG_PRT_LENGTH);
  payload[0] = 0x01; // Port 1 is the UART
  payload[1] = 0x00; // Reserved
  payload[2] = 0x00; // txReady
  payload[3] = 0x00; // txReady
  payload[4] = 0xC0; // mode 1100 0000 (8-bit character length)
  payload[5] = 0x08; // mode 0000 1000 (No parity, 1 stop bit)
  payload[6] = 0x00; // mode
  payload[7] = 0x00; // mode
  SET(payload + 8, baud,
      uint32_t);      // baudRate ( 115200 = 0x00 01 C2 00, 230400 = 0x00 03 84 00, 460800 = 0x00 07 08 00
  payload[12] = 0x01; // inProtoMask (ubx)
  payload[13] = 0x00; // inProtoMask
  payload[14] = 0x01; // outProtoMask (ubx)
  payload[15] = 0x00; // outProtoMask
  payload[16] = 0x00; // flags
  payload[17] = 0x00; // flags
  payload[18] = 0x00; // reserved2
  payload[19] = 0x00; // reserved2

  return tx(message, CFG_PRT_LENGTH);
}

uint16_t Ubx::cfgRate(uint32_t hz)
{
#define CFG_RATE_LENGTH 6
  uint8_t message[CFG_RATE_LENGTH + 8] = {0};
  uint8_t * payload = message + 6;
  uint32_t dt_ms = 1000 / hz;

  header(message, 0x06, 0x08, CFG_RATE_LENGTH);
  SET(payload, dt_ms, uint16_t);
  //	payload[2] = 0x01; // navRate 1 per measurement //10 Hz??
  //	payload[3] = 0x00; //
  SET(payload + 2, 0x001, uint16_t); // navRate 1 per measurement //10 Hz??
                                     //	payload[4] = 0x01; // timeRef = GPS time ref
                                     //	payload[5] = 0x00; //
  SET(payload + 4, 0x001, uint16_t); // timeRef = GPS time ref
  return tx(message, CFG_RATE_LENGTH);
}

uint16_t Ubx::cfgTp5(uint32_t hz)
{
#define SFG_TP5_LENGTH 32
  uint8_t message[SFG_TP5_LENGTH + 8] = {0};
  uint8_t * payload = message + 6;
//  uint32_t period_us = 1000000 / hz;
  uint32_t pulse_len_us = 1000;
  header(message, 0x06, 0x31, SFG_TP5_LENGTH);

  payload[0] = 0x00;                 // Timepulse pin 0
  payload[1] = 0x00;                 // Version 0
  payload[2] = 0x00;                 // reserved
  payload[3] = 0x00;                 // reserved
  SET(payload + 4, 0x0000, int16_t); // antenna delay
  SET(payload + 6, 0x0000, int16_t); // rf delay
  SET(payload + 8, hz, uint32_t); // frequency when not locked
  SET(payload + 12, hz, uint32_t); // frequency when locked
  SET(payload + 16, 0, uint32_t); // pulse length when not locked (1ms)
  SET(payload + 20, pulse_len_us, uint32_t); // pulse length when locked (1ms)
  SET(payload + 24, 0x0000, uint32_t); // delay
  SET(payload + 28, 0x007F, uint32_t); // 0x0111 1111

  return tx(message, SFG_TP5_LENGTH);
}
uint16_t Ubx::cfgNav5(void)
{
#define CFG_NAV5_LENGTH 36
  uint8_t message[CFG_NAV5_LENGTH + 8] = {0};
  uint8_t * payload = message + 6;

  header(message, 0x06, 0x24, CFG_NAV5_LENGTH);
  payload[0] = 5; // Parameters bitmask
  payload[1] = 8; // Airbourne navigatin < 4G's
  payload[3] = 3; // Auto 2d/3d fix mode
  // all the rest are zero
  return tx(message, CFG_NAV5_LENGTH);
}
uint16_t Ubx::cfgMsg(uint8_t cl, uint8_t id, uint8_t decimation_rate)
{
#define CFG_MSG_LENGTH 3
  uint8_t message[CFG_MSG_LENGTH + 8] = {0};
  uint8_t * payload = message + 6;

  header(message, 0x06, 0x01, CFG_MSG_LENGTH);
  payload[0] = cl;
  payload[1] = id;
  payload[2] = decimation_rate;

  return tx(message, CFG_MSG_LENGTH);
}


