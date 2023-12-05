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

#include <Time64.h>
#include <Ubx.h>
#include <misc.h>

extern Time64 time64;

#define SET(buf, data, type) *((type *)(buf)) = data

#define UBX_DMA_BUFFER_SIZE 16 // must be multiple of 16
__attribute__((section("my_dma_buffers")))
__attribute__((aligned(32))) static uint8_t ubx_dma_rxbuf[UBX_DMA_BUFFER_SIZE] = {0};

__attribute__((section("my_buffers")))
__attribute__((aligned(32))) static uint8_t ubx_fifo_rx_buffer[UBX_FIFO_BUFFERS * sizeof(UbxPacket)] = {0};

uint32_t Ubx::init(
    // Driver initializers
    uint16_t sample_rate_hz,
    GPIO_TypeDef *drdy_port, // Reset GPIO Port
    uint16_t drdy_pin,       // Reset GPIO Pin
    // UART initializers
    UART_HandleTypeDef *huart,
    USART_TypeDef *huart_instance,
    DMA_HandleTypeDef *hdma_uart_rx,
    uint32_t baud)
{
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  dtimeout_ = 1000000; // 1 seconds
  timeout_ = 0;

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx;

  groupDelay_ = 0;

  gotNav_ = false;
  gotPvt_ = false;

  // USART initialization
  huart_->Instance = USART1;
  huart_->Init.BaudRate = 9600;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.StopBits = UART_STOPBITS_1;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_->Init.OverSampling = UART_OVERSAMPLING_16;
  huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(huart_) != HAL_OK)
    return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetTxFifoThreshold(huart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetRxFifoThreshold(huart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_DisableFifoMode(huart_) != HAL_OK)
    return DRIVER_HAL_ERROR;

  uint32_t ubx_baud = 0;
  uint16_t i;
  for (i = 0; i <= 100; i++)
    if ((ubx_baud = pollCfgPrt()))
      break;
  misc_printf("Baud Before = %lu, after %u retries.\n\r", ubx_baud, i);

  misc_printf("Setting baud to = %lu\n\r", baud);
  uint32_t j;
  for (j = 0; j < 10; j++)
  {
    huart_->Init.BaudRate = 9600;
    HAL_UART_Init(huart_);

    cfgPrt(baud); // 0x06 0x00, baud rate change message

    huart_->Init.BaudRate = baud;
    HAL_UART_Init(huart_);

    time64.dMs(30); // takes around 30ms to get an ack for the baud rate change.

    for (i = 0; i < 5; i++)
      if ((ubx_baud = pollCfgPrt()))
        break;
    if (baud == ubx_baud)
      break;
    misc_printf("Failed, retry %u\n\r", j + 1);
  }
  misc_printf("Baud After  = %lu, %u retries\n\r", ubx_baud, j);

  if (ubx_baud != baud)
    return UBX_FAIL_BAUD_CHANGE;

  rxFifo_.init(UBX_FIFO_BUFFERS, sizeof(UbxPacket), ubx_fifo_rx_buffer);

  // Disable these messages to get rid of clutter
  uint16_t error = 0;

  // Disable these messages
  error |= (uint16_t)cfgMsg(0x0A, 0x09, 0); // MON-HW
  error |= (uint16_t)cfgMsg(0x0A, 0x0B, 0); // MON-HW2
  error |= (uint16_t)cfgMsg(0x01, 0x04, 0); // NAV-DOP
  error |= (uint16_t)cfgMsg(0x01, 0x03, 0); // NAV-STATUS
  error |= (uint16_t)cfgMsg(0x01, 0x20, 0); // NAV-TIMEGPS
  error |= (uint16_t)cfgMsg(0x01, 0x35, 0); // NAV-SAT
  error |= (uint16_t)cfgMsg(0x01, 0x01, 0); // NAV-POSECEF (length 20)
  error |= (uint16_t)cfgMsg(0x01, 0x11, 0); // NAV-VELECEF (length 20)

  // Enable only these messages
  error |= (uint16_t)cfgMsg(0x01, 0x07, 1); // NAV-PVT (length 92)
  error |= (uint16_t)cfgMsg(0x01, 0x06, 1); // NAV-SOL
  // Set GPS Configuration
  error |= (uint16_t)cfgRate(sampleRateHz_); // Nav rate 0x06 0x08
  error |= (uint16_t)cfgTp5(sampleRateHz_);  // PPS rate 0x06 0x31
  error |= (uint16_t)cfgNav5();              // airplane mode 0x06 0x24

  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);

  return DRIVER_OK;
}

bool Ubx::poll(void)
{
  // Check if we are timed-out
  if (time64.Us() > timeout_)
  {
    if ((((DMA_Stream_TypeDef *)(hdmaUartRx_)->Instance)->CR & DMA_SxCR_EN) != DMA_SxCR_EN)
    {
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
  drdy_ = time64.Us();

#if USE_D_CACHE_MANAGEMENT_FUNCTIONS
  SCB_InvalidateDCache_by_Addr((uint32_t *)ubx_dma_rxbuf, UBX_DMA_BUFFER_SIZE); // Force read from SRAM
#endif

  uint16_t bytes_in_dma_buffer = misc_bytes_in_dma(hdmaUartRx_, UBX_DMA_BUFFER_SIZE);

  static UbxFrame p;

  for (int i = 0; i < bytes_in_dma_buffer; i++)
  {
    bool found = parseByte(ubx_dma_rxbuf[i], &p);

    if (found)
    {
      if ((p.cl == 0x01) && (p.id == 0x07))
      {
        gotPvt_ = true;
        memcpy((uint8_t *)&(ubx_.pvt), p.payload, sizeof(ubx_.pvt));
      }
      else if ((p.cl == 0x01) && (p.id == 0x06))
      {
        gotNav_ = true;
        memcpy((uint8_t *)(&ubx_.nav), p.payload, sizeof(ubx_.nav));
      }
      if (gotPvt_ && gotNav_)
      {
        ubx_.timestamp = time64.Us(); // usTime();
        ubx_.drdy = drdy_;
        ubx_.groupDelay = ubx_.drdy - ubx_.pps;
        rxFifo_.write((uint8_t *)&ubx_, sizeof(ubx_));
        gotPvt_ = false;
        gotNav_ = false;
      }
    }
  }
  startDma(); // restart the DMA
}

bool Ubx::parseByte(uint8_t c, UbxFrame *p)
{
  static uint16_t n = 0;

  // special case where we get 0xB5 randomly duplicated at the start (DMA wierdness).
  if ((c == 0xB5) & (n == 1))
    n = 0;

  if (n == 0) // header byte 1 "mu" character
  {
    if (c == 0xB5)
      n++;
    else
      n = 0;
    p->A = 0;
    p->B = 0; // Reset the checksum calculation
  }
  else if (n == 1) // header byte 2
  {
    if (c == 0x62)
      n++;
    else if (c == 0xB5)
      n = 1; // repeated 'mu'
    else
      n = 0;
  }
  else if (n == 2) // Class
  {
    if ((c == 0x01) || (c == 0x05) || (c == 0x06)) // NAV, CFG, and ACK messages.
    {
      p->cl = c;
      n++;
      p->A += c;
      p->B += p->A;
    }
    else
      n = 0;
  }
  else if (n == 3) // ID, allow all
  {
    p->id = c;
    n++;
    p->A += c;
    p->B += p->A;
  }
  else if (n == 4) // length LSB
  {
    p->length = (uint16_t)c;
    n++;
    p->A += c;
    p->B += p->A;
  }
  else if (n == 5) // length MSB
  {
    p->length |= ((uint16_t)c) << 8;
    if (p->length > UBX_MAX_PAYLOAD_BYTES)
      n = 0;
    else
    {
      n++;
      p->A += c;
      p->B += p->A;
    }
  }
  else if (n < p->length + 6) // Packet Payload bytes and first byte of checksum.
  {
    p->payload[n - 6] = c;
    n++;
    p->A += c;
    p->B += p->A;
  }
  else if (n == p->length + 6) // Checksum A
  {
    if (p->A != c)
      n = 0;
    else
      n++;
  }
  else // if(n==p->length+7) // Checksum B (the end)
  {
    n = 0;
    if (p->B == c)
    {
      return true;
    }
  }
  return false;
}

void Ubx::pps(uint64_t pps_timestamp)
{
  gotNav_ = false;
  gotPvt_ = false;
  ubx_.pps = pps_timestamp;
}

bool Ubx::display(void)
{
  UbxPacket p;

  char name_nav[] = "Ubx (nav)";
  char name_pvt[] = "Ubx (pvt)";

  if (rxFifo_.read((uint8_t *)&p, sizeof(p)))
  {
    misc_header(name_pvt, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double)(p.timestamp - p.pps) / 1000.);
    misc_printf(" iTOW %10u | ", p.pvt.iTOW);
    misc_printf("%02u/%02u/%04u ", p.pvt.month, p.pvt.day, p.pvt.year);
    misc_printf("%02u:%02u:%09.6f", p.pvt.hour, p.pvt.min, (double)p.pvt.sec + (double)p.pvt.nano * 1e-9);
    misc_printf("%14.8f deg %14.8f deg | ", (double)p.pvt.lat * 1e-7, (double)p.pvt.lon * 1e-7);
    misc_printf("numSV %02d\n\r", p.pvt.numSV);

    misc_header(name_nav, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f ms | ", (double)(p.timestamp - p.pps) / 1000.);
    misc_printf("  TOW %14.3f\n\r", (double)p.nav.iTOW + (double)p.nav.fTOW / 1000);
  }
  else
  {
    misc_printf("%s\n\r", name_pvt);
    misc_printf("%s\n\r", name_nav);
  }

  return 1;
}

void Ubx::checksum(uint8_t *buffer)
{
  uint8_t a = 0, b = 0;
  uint16_t n = ((((uint16_t)buffer[5]) << 8) | (uint16_t)buffer[4]) + 6;
  for (int16_t i = 2; i < n; i++)
  {
    a += buffer[i];
    b += a;
  }
  buffer[n] = a;
  buffer[n + 1] = b;
}

void Ubx::header(uint8_t *buffer, uint8_t cl, uint8_t id, uint16_t length)
{
  buffer[0] = 0xB5; // mu
  buffer[1] = 0x62; // b
  buffer[2] = cl;   // Class
  buffer[3] = id;   // ID
  SET(buffer + 4, length, uint16_t);
}

uint16_t Ubx::tx(uint8_t *buffer, uint32_t length)
{
  uint16_t err = 0;
  checksum(buffer);

  for (uint32_t n = 0; n < length + 8; n++)
  {
    HAL_UART_Transmit(huart_, (uint8_t *)buffer++, 1, 10);
  }
  //	HAL_UART_Abort(huart_); // flush any leftover crumbs.

  //	if(HAL_UART_Transmit(huart_, (uint8_t*)buffer, length+8, 1000)!=HAL_OK) err |= DRIVER_HAL_ERROR;
  // Unclear why the above does not work right, the following is a kludge.
  //	if(HAL_UART_Transmit_IT(huart_, (uint8_t*)buffer, length+8)!=HAL_OK) err |= DRIVER_HAL_ERROR;
  //	time64.dMs(10); // wait for transfer to complete.

  return err;
}

uint32_t Ubx::pollCfgPrt(void)
{
#define POLL_BUFFER_SIZE 128
#define POLL_CFG_PRT_LENGTH 1
  // uint8_t message[CFG_PRT_LENGTH+8]={0};
  uint8_t message[POLL_BUFFER_SIZE] = {0};
  // tx(message, POLL_CFG_PRT_LENGTH);
  // HAL_UART_Transmit(huart_, message, 1, 1);

  uint8_t *payload = message + 6;
  header(message, 0x06, 0x00, POLL_CFG_PRT_LENGTH);
  payload[0] = 0x01; // Port 1 is the UART

  //	__HAL_UART_CLEAR_IDLEFLAG(huart_);
  //	__HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
  //	HAL_UART_Abort(huart_); // flush any leftover crumbs.

  // uint16_t err =
  tx(message, POLL_CFG_PRT_LENGTH);
  memset(message, 0, POLL_CFG_PRT_LENGTH);
  HAL_StatusTypeDef status = HAL_UART_Receive(huart_, message, POLL_BUFFER_SIZE, 63);

  UbxFrame p;

  if ((status == HAL_TIMEOUT) || (status == HAL_OK))
  {
    for (int i = 0; i < POLL_BUFFER_SIZE; i++)
    {
      if (parseByte(message[i], &p))
      {
        if ((p.cl == 0x06) && (p.id == 0x00))
        {
          uint32_t baud = (uint32_t)p.payload[11] << 24 | (uint32_t)p.payload[10] << 16 | (uint32_t)p.payload[9] << 8
                          | (uint32_t)p.payload[8];
          // misc_printf("%lu b\n\r",baud);
          if ((baud != 9600) && (baud != 115200))
            return 0;
          return baud;
        }
      }
    }
  }
  //	misc_printf("Fail\n\r");
  return 0;
}

uint16_t Ubx::cfgPrt(uint32_t baud)
{
#define CFG_PRT_LENGTH 20
  uint8_t message[CFG_PRT_LENGTH + 8] = {0};
  uint8_t *payload = message + 6;

  header(message, 0x06, 0x00, CFG_PRT_LENGTH);
  payload[0] = 0x01;                // Port 1 is the UART
  payload[1] = 0x00;                // Reserved
  payload[2] = 0x00;                // txReady
  payload[3] = 0x00;                // txReady
  payload[4] = 0xC0;                // mode 1100 0000 (8-bit character length)
  payload[5] = 0x08;                // mode 0000 1000 (No parity, 1 stop bit)
  payload[6] = 0x00;                // mode
  payload[7] = 0x00;                // mode
  SET(payload + 8, baud, uint32_t); // baudRate ( 115200 = 0x00 01 C2 00, 230400 = 0x00 03 84 00, 460800 = 0x00 07 08 00
  payload[12] = 0x01;               // inProtoMask (ubx)
  payload[13] = 0x00;               // inProtoMask
  payload[14] = 0x01;               // outProtoMask (ubx)
  payload[15] = 0x00;               // outProtoMask
  payload[16] = 0x00;               // flags
  payload[17] = 0x00;               // flags
  payload[18] = 0x00;               // reserved2
  payload[19] = 0x00;               // reserved2

  return tx(message, CFG_PRT_LENGTH);
}
uint16_t Ubx::cfgRate(uint32_t hz)
{
#define CFG_RATE_LENGTH 6
  uint8_t message[CFG_RATE_LENGTH + 8] = {0};
  uint8_t *payload = message + 6;
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
  uint8_t *payload = message + 6;
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
  uint8_t *payload = message + 6;

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
  uint8_t *payload = message + 6;

  header(message, 0x06, 0x01, CFG_MSG_LENGTH);
  payload[0] = cl;
  payload[1] = id;
  payload[2] = decimation_rate;

  return tx(message, CFG_MSG_LENGTH);
}
