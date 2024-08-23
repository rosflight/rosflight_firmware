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

#include <Ubx.h>

#include <Time64.h>

#include <Packets.h>
#include <misc.h>

extern Time64 time64;

#define SET(buf, data, type) *((type *) (buf)) = data

#define UBX_DMA_BUFFER_SIZE 16 * 2 // must be multiple of 16
DMA_RAM uint8_t ubx_dma_rxbuf[UBX_DMA_BUFFER_SIZE];

DTCM_RAM uint8_t ubx_fifo_rx_buffer[UBX_FIFO_BUFFERS * sizeof(UbxPacket)];

uint32_t Ubx::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, uint16_t drdy_pin, bool has_pps,
  // UART initializers
  UART_HandleTypeDef * huart, USART_TypeDef * huart_instance, DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud,
  UbxProtocol ubx_protocol)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Ubx");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;
  hasPps_ = has_pps;

  dtimeout_ = 1000000; // 1 seconds
  timeout_ = 0;

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx; // huart->hdmarx;

  baud_initial_ = 9600;
  baud_ = baud;

  ubxProtocol_ = ubx_protocol;

  groupDelay_ = 0;

  // gotNav_ = false;
  gotPvt_ = 0;
  gotTime_ = 0;
  gotEcefP_ = 0;
  gotEcefV_ = 0;

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
  //     baud_initial_ = 100000000/huart_->Instance->BRR;

  uint32_t bauds[] = {9600, 230400, 57600}; // { 9600, 19200, 38400, 57600, 115200, 230400};
  unsigned int i, retry;
  HAL_Delay(1000);

  uint32_t ubx_baud = 0;

  for (retry = 0; retry < 5; retry++) {
    for (i = 0; i < sizeof(bauds) / sizeof(uint32_t); i++) {
      // Set STM Baud
      huart_->Init.BaudRate = bauds[i];
      if (HAL_UART_Init(huart_) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }
      // Set UBLOX Baud
      if (ubxProtocol_ == UBX_M8) cfgPrt(baud_);
      else cfgM9(baud_, sampleRateHz_);
      HAL_Delay(2);
    }

    // Set STM Baud
    huart_->Init.BaudRate = baud_;
    if (HAL_UART_Init(huart_) != HAL_OK) {
      initializationStatus_ |= DRIVER_HAL_ERROR;
      return initializationStatus_;
    }
    HAL_Delay(100); // Give the UBLOX some time to get there

    // Check if we have acquired the baud rate
    if (ubxProtocol_ == UBX_M8) ubx_baud = pollBaud();
    else ubx_baud = pollBaudM9();

    if (ubx_baud == baud_) break;
    misc_printf("%6lu, %u retries\n", 100000000 / huart_->Instance->BRR, retry);
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

  rxFifo_.init(UBX_FIFO_BUFFERS, sizeof(UbxPacket), ubx_fifo_rx_buffer);

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

  // Enable these messages
  error |= (uint16_t) cfgMsg(0x01, 0x20, 1); // NAV-TIMEGPS (length 16)
  error |= (uint16_t) cfgMsg(0x01, 0x07, 1); // NAV-PVT (length 92)
  error |= (uint16_t) cfgMsg(0x01, 0x01, 1); // NAV-POSECEF (length 20)
  error |= (uint16_t) cfgMsg(0x01, 0x11, 1); // NAV-VELECEF (length 20)

  // Set GPS Configuration (already done in pollCfgPrtM9() for UBX_M9)
  if (ubxProtocol_ == UBX_M8) {
    error |= (uint16_t) cfgRate(sampleRateHz_); // Nav rate 0x06 0x08
    error |= (uint16_t) cfgTp5(sampleRateHz_);  // PPS rate 0x06 0x31
    error |= (uint16_t) cfgNav5();              // airplane mode 0x06 0x24
  }

  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);

  return initializationStatus_;
}

bool Ubx::poll(void)
{
  // Check if we are timed-out
  if (time64.Us() > timeout_) {
    if ((((DMA_Stream_TypeDef *) (huart_->hdmarx)->Instance)->CR & DMA_SxCR_EN) != DMA_SxCR_EN) {
      __HAL_UART_CLEAR_IDLEFLAG(huart_); // this may be redundant with call to HAL_UART_Abort()
      __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
      HAL_UART_Abort(huart_); // flush any leftover crumbs.
      startDma();
    }
  }
  return 0;
}

bool Ubx::startDma(void)
{
  timeout_ = time64.Us() + dtimeout_; // 1000000 for one second timeout
  HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(huart_, ubx_dma_rxbuf, UBX_DMA_BUFFER_SIZE); // start next read
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
        gotPvt_ = time64.Us();
        memcpy((uint8_t *) &(ubx_.pvt), p.payload, sizeof(ubx_.pvt));
      } else if ((p.cl == 0x01) && (p.id == 0x20)) {
        gotTime_ = time64.Us();
        memcpy((uint8_t *) (&ubx_.time), p.payload, sizeof(ubx_.time));
      } else if ((p.cl == 0x01) && (p.id == 0x01)) {
        gotEcefP_ = time64.Us();
        memcpy((uint8_t *) (&ubx_.ecefp), p.payload, sizeof(ubx_.ecefp));
      } else if ((p.cl == 0x01) && (p.id == 0x11)) {
        gotEcefV_ = time64.Us();
        memcpy((uint8_t *) (&ubx_.ecefv), p.payload, sizeof(ubx_.ecefv));
      }

      if (gotPvt_ && gotTime_ && gotEcefP_ && gotEcefV_) {
        ubx_.timestamp = time64.Us(); // usTime();

        ubx_.drdy = drdy_;
        if (!hasPps_) ubx_.pps = ubx_.drdy - 25000; // fake number if we don't have PPS hooked up

        if (ubx_.pps > ubx_.drdy) ubx_.pps -= 1000000 / sampleRateHz_;
        ubx_.groupDelay = ubx_.drdy - ubx_.pps;

        rxFifo_.write((uint8_t *) &ubx_, sizeof(ubx_));
        gotPvt_ = 0;
        gotTime_ = 0;
        gotEcefP_ = 0;
        gotEcefV_ = 0;
        drdy_ = 0;
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

    if ((p->cl == 0x01) && (p->id == 0x07)) drdy_ = time64.Us() - UBX_DMA_BUFFER_SIZE * 10000000 / baud_;
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

void Ubx::pps(uint64_t pps_timestamp)
{
  //    gotPvt_ = false;
  //    gotTime_ = false;
  //    gotEcefP_ = false;
  //    gotEcefV_ = false;
  ubx_.pps = pps_timestamp;
}

bool Ubx::display(void)
{
  UbxPacket p;

  char name_pvt[] = "Ubx (pvt)";
  char name_time[] = "Ubx (time)";
  char name_ecefp[] = "Ubx (ecefp)";
  char name_ecefv[] = "Ubx (ecefv)";

  if (rxFifo_.read((uint8_t *) &p, sizeof(p))) {
    misc_header(name_pvt, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double) (p.timestamp - p.pps) / 1000.);
    misc_printf(" iTOW %10u | ", p.pvt.iTOW);
    misc_printf("%02u/%02u/%04u ", p.pvt.month, p.pvt.day, p.pvt.year);
    misc_printf("%02u:%02u:%09.6f", p.pvt.hour, p.pvt.min, (double) p.pvt.sec + (double) p.pvt.nano * 1e-9);
    misc_printf("%14.8f deg %14.8f deg | ", (double) p.pvt.lat * 1e-7, (double) p.pvt.lon * 1e-7);
    misc_printf("numSV %02d | ", p.pvt.numSV);
    misc_printf("Fix %02d\n", p.pvt.fixType);

    misc_header(name_time, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double) (p.timestamp - p.pps) / 1000.);
    misc_printf(" iTOW %10u | ", p.time.iTOW);
    misc_printf("  TOW %14.3f ms | valid 0x%02X\n", (double) p.time.iTOW + (double) p.time.fTOW / 1000, p.time.valid);

    misc_header(name_ecefp, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double) (p.timestamp - p.pps) / 1000.);
    misc_printf(" iTOW %10u | ", p.ecefp.iTOW);
    misc_printf("  %10d %10d %10d %10u cm\n", p.ecefp.ecefX, p.ecefp.ecefY, p.ecefp.ecefZ, p.ecefp.pAcc);

    misc_header(name_ecefv, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double) (p.timestamp - p.pps) / 1000.);
    misc_printf(" iTOW %10u | ", p.ecefv.iTOW);
    misc_printf("  %10d %10d %10d %10u cm/s\n", p.ecefv.ecefVX, p.ecefv.ecefVY, p.ecefv.ecefVZ, p.ecefv.sAcc);
  } else {
    misc_printf("%s\n", name_pvt);
    misc_printf("%s\n", name_time);
    misc_printf("%s\n", name_ecefp);
    misc_printf("%s\n", name_ecefv);
  }

  return 1;
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
  uint32_t period_us = 1000000 / hz;
  uint32_t pulse_len_us = 1000;
  header(message, 0x06, 0x31, SFG_TP5_LENGTH);

  payload[0] = 0x00;                 // Timepulse pin 0
  payload[1] = 0x01;                 // Version 1
  payload[2] = 0x00;                 // reserved
  payload[3] = 0x00;                 // reserved
  SET(payload + 4, 0x0000, int16_t); // antenna delay
  SET(payload + 6, 0x0000, int16_t); // rf delay
  SET(payload + 8, period_us, uint32_t);
  SET(payload + 12, period_us / 2, uint32_t);
  SET(payload + 16, pulse_len_us, uint32_t); // no pulse when not locked =0 ?
  SET(payload + 20, pulse_len_us / 2, uint32_t);
  SET(payload + 24, 0x0000, uint32_t); // delay
  SET(payload + 28, 0x01F7, uint32_t); // 0001 1111 1111 0111 = 0x01F7

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

//////////////////////////////////////////////////////////////////////////////////////////
// M9 Protocol

//#define SET(buf,data,type) *((type*)(buf))=data

// M9 parameters setting functions
#define SETVAL(ptr, value, type)                                                                                       \
  {                                                                                                                    \
    *((type *) (ptr)) = value;                                                                                         \
    ptr += sizeof(type);                                                                                               \
  }

#define SETKV(ptr, key, value, type)                                                                                   \
  {                                                                                                                    \
    *(uint32_t *) (ptr) = key;                                                                                         \
    ptr += 4;                                                                                                          \
    *((type *) (ptr)) = value;                                                                                         \
    ptr += sizeof(type);                                                                                               \
  }

uint16_t Ubx::cfgM9(uint32_t baud, uint16_t sampleRateHz)
{
  uint8_t cfg_message[64] = {0};
  uint16_t length = 0;
  uint8_t * p = cfg_message;
  // Header
  SETVAL(p, 0xB5, uint8_t); // mu (1 bytes)
  SETVAL(p, 0x62, uint8_t); // b (1 bytes)
  SETVAL(p, 0x06, uint8_t); // class CFG (1 bytes)
  SETVAL(p, 0x8A, uint8_t); // id Write Value (1 bytes)
  p += 2;                   // space for length (2 bytes)
  // Message
  SETVAL(p, 0x00, uint8_t); // Message Version (1 bytes)
  SETVAL(p, 0x01, uint8_t); // Write to RAM bit 1 is ram, 2 is bbr layer, 3 is flash (1 bytes)
  p += 2;                   // reserved (2 bytes)
  // Key value pairs
  SETKV(p, 0x40520001, baud, uint32_t);                // baud rate (8 bytes)
  SETKV(p, 0x30210001, 1000 / sampleRateHz, uint16_t); // output rate in milliseconds (6 bytes)
  SETKV(p, 0x30210002, 1, uint16_t);                   // 1 data output per nav measurement (6 bytes)
  SETKV(p, 0x20110021, 8, uint8_t);                    // CFG-NAVSPG-DYNMODEL 8 = 4G Airborne (5 bytes)
  SETKV(p, 0x20110011, 3, uint8_t);                    // CFG-NAVSPG-FIXMODE 3 = Auto 2/3D (5 bytes)

  // compute and insert message length length
  length = p - cfg_message - 6;
  SET(cfg_message + 4, length, uint16_t);

  return tx(cfg_message, length);
}
uint32_t Ubx::pollBaudM9(void)
{
#define CFG_MESSAGE_SIZE 64
  uint8_t cfg_message[CFG_MESSAGE_SIZE] = {0};
  uint16_t length = 0;
  uint8_t * p = cfg_message;
  // Header
  SETVAL(p, 0xB5, uint8_t); // mu (1 bytes)
  SETVAL(p, 0x62, uint8_t); // b (1 bytes)
  SETVAL(p, 0x06, uint8_t); // class CFG (1 bytes)
  SETVAL(p, 0x8B, uint8_t); // id Write Value (1 bytes)
  p += 2;                   // space for length (2 bytes)
  // Message
  SETVAL(p, 0x00, uint8_t);    // Message Version (1 bytes)
  SETVAL(p, 0x00, uint8_t);    // Read to RAM
  SETVAL(p, 0x0000, uint16_t); // Offset (position)

  SETVAL(p, 0x40520001, uint32_t); // baud

  // compute and insert message length length
  length = p - cfg_message - 6;
  SET(cfg_message + 4, length, uint16_t);

  if (tx(cfg_message, length) != HAL_OK) return 0;

  UbxFrame ubx;
  memset(&ubx, 0, sizeof(ubx));

  for (int i = 0; 256; i++) {
    uint8_t ch;
    HAL_StatusTypeDef hal_status = HAL_UART_Receive(huart_, &ch, 1, 1000);

    if (hal_status == HAL_OK)
      if (parseByte(ch, &ubx)) {
        if ((ubx.cl == 0x06) && (ubx.id == 0x8B)) {
          uint32_t key = (uint32_t) ubx.payload[7] << 24 | (uint32_t) ubx.payload[6] << 16
            | (uint32_t) ubx.payload[5] << 8 | (uint32_t) ubx.payload[4];
          if (key == 0x40520001)
            return (uint32_t) ubx.payload[11] << 24 | (uint32_t) ubx.payload[10] << 16 | (uint32_t) ubx.payload[9] << 8
              | (uint32_t) ubx.payload[8];
          ; // baud key
        } else {
          memset(&ubx, 0, sizeof(ubx));
        }
      }
  }
  return 0;
}
