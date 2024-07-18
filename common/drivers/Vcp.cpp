/**
 ******************************************************************************
 * File     : Vcp.cpp
 * Date     : Oct 11, 2023
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

#include <BoardConfig.h>
#include <Packets.h>
#include <Vcp.h>
#include <usb_device.h>

#include <Time64.h>
#include <usbd_cdc_acm_if.h>

extern Time64 time64;

#define VCP_TX_FIFO_BUFFERS SERIAL_TX_FIFO_BUFFERS
DTCM_RAM uint8_t vcp_fifo_tx_buffer[VCP_TX_FIFO_BUFFERS * sizeof(SerialTxPacket)];

#define VCP_RX_FIFO_BUFFER_BYTES 4096
DTCM_RAM uint8_t vcp_fifo_rx_buffer[VCP_RX_FIFO_BUFFER_BYTES];

uint32_t Vcp::init(uint16_t sample_rate_hz)
{
  // Common initializations
  sampleRateHz_ = sample_rate_hz;

  txTimeout_ = 0;
  txDtimeout_ = 250; // 1000000/sampleRateHz_;

  txFifo_.init(VCP_TX_FIFO_BUFFERS, sizeof(SerialTxPacket), vcp_fifo_tx_buffer); // Packet Fifo
  rxFifo_.init(VCP_RX_FIFO_BUFFER_BYTES, vcp_fifo_rx_buffer);                    // byte Fifo

  txIdle_ = true;
  retry_ = 0;

  // VCP-specific
  // MX_USB_DEVICE_Init();

  return DRIVER_OK;
}

// typedef enum
//{
//   USBD_OK = 0U,
//   USBD_BUSY,
//   USBD_EMEM,
//   USBD_FAIL,
// } USBD_StatusTypeDef;

/**
 * @fn uint16_t writePacket(SerialTxPacket*)
 * @brief Comms Tx (to companion computer) Rosflight/VCP interface.
 *
 * @param p_new
 * @return
 */

uint16_t Vcp::writePacket(SerialTxPacket * p_new)
{
  return txFifo_.write((uint8_t *) p_new, sizeof(SerialTxPacket));
}

/**
 * @fn void poll(void)
 * @brief This (Polling loop) needs to be run at the same interrupt level as the CDC Tx Complete Callback.
 *
 */
void Vcp::poll(void)
{
  //	// TX
  if ((txFifo_.packetCountMax() > 0) && ((time64.Us() > txTimeout_))) txStart();
}
/**
 * @fn void txCdcCallback(void)
 * @brief This CDC Tx Complete Callback needs to be at the same interrupt level as the polling loop.
 *
 */
void Vcp::txCdcCallback(void) { txStart(); }

void Vcp::txStart()
{
  txTimeout_ = time64.Us() + txDtimeout_;

  static SerialTxPacket p;
  uint8_t status = !USBD_OK;

  if (retry_) {
    status = VCP_Transmit((uint8_t *) p.payload, p.payloadSize);
    if (status == USBD_OK) {
      retry_ = 0;
      txIdle_ = false;
    } else {
      retry_--;
      txIdle_ = true;
    }
    //		return;
  } else if (txFifo_.packetCount()) {
    if (txFifo_.read((uint8_t *) &p, sizeof(SerialTxPacket))) {
      status = VCP_Transmit((uint8_t *) p.payload, p.payloadSize);
      if (status == USBD_OK) {
        retry_ = 0;
        txIdle_ = false;
      } else {
        retry_ = 2;
        txIdle_ = true;
      } // try at most two more times
    } else {
      retry_ = 0;
    }
  }
}
