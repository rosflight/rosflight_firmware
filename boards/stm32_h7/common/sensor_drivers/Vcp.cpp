/**
 ******************************************************************************
 * File     : Vcp.cpp
 * Date     : Oct 11, 2023
 ******************************************************************************
 **/

#include "Vcp.h"
#include "stm32_h7.hpp"
#include "BoardConfig.h"
#include "Packets.h"

#include "usb_device.h"

#include "Time64.h"
#include "usbd_cdc_acm_if.h"

extern Time64 time64;

#define VCP_TX_FIFO_BUFFERS 256
DTCM_RAM uint8_t vcp_fifo_tx_buffer[VCP_TX_FIFO_BUFFERS * sizeof(SerialTxPacket)];

#define VCP_RX_FIFO_BUFFER_BYTES (8 * 1024)
static uint8_t vcp_fifo_rx_buffer[VCP_RX_FIFO_BUFFER_BYTES];

#define VCP_RX_PACKET_FIFO_BUFFERS 32
#define VCP_RX_PACKET_BYTES 128
static uint8_t vcp_rx_packet_buffer[VCP_RX_PACKET_FIFO_BUFFERS * VCP_RX_PACKET_BYTES];

uint32_t Vcp::init(uint16_t sample_rate_hz)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Vcp");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  txFifo_.init(VCP_TX_FIFO_BUFFERS, sizeof(SerialTxPacket), vcp_fifo_tx_buffer);
  rxPacketFifo_.init(VCP_RX_PACKET_FIFO_BUFFERS, VCP_RX_PACKET_BYTES, vcp_rx_packet_buffer);
  rxFifo_.init(VCP_RX_FIFO_BUFFER_BYTES, vcp_fifo_rx_buffer);

  txIdle_ = true;
  retry_ = 0;

  return initializationStatus_;
}

uint16_t Vcp::writePacket(SerialTxPacket * p_new)
{
  uint16_t size = txFifo_.write((uint8_t *) p_new, sizeof(SerialTxPacket));
  if (size != 0) {
    tx_signal_.trigger();
  }
  return size;
}

void Vcp::cdcReceiveCallback(uint8_t * buffer, uint16_t size)
{
  if ((size != 0U) && (rxPacketFifo_.write(buffer, size) != 0U)) {
    rx_signal_.trigger();
  }
}

AsyncTask<void> Vcp::rxRun()
{
  uint8_t packet[VCP_RX_PACKET_BYTES] = {};

  while (true) {
    co_await rx_signal_.wait_for_trigger();

    while (rxPacketFifo_.packetCount() > 0U) {
      const uint16_t size = rxPacketFifo_.read(packet, sizeof(packet));
      if (size == 0U) {
        break;
      }
      rxFifo_.writeBlock(packet, size);
    }
  }
}

AsyncTask<void> Vcp::txRun()
{
  while (true) {
    co_await tx_signal_.wait_for_trigger();

    if (!txIdle_) {
      continue;
    }

    while (txStart()) {
      co_await tx_signal_.wait_for_trigger();
    }
  }
}

void Vcp::cdcTransmitCpltCallback(void)
{
  txIdle_ = true;
  tx_signal_.trigger();
}

bool Vcp::txStart(void)
{
  txIdle_ = false;

  if (retry_ == 0U) {
    if (txFifo_.packetCount() == 0U) {
      txIdle_ = true;
      return false;
    }

    if (txFifo_.read((uint8_t *) &pendingTxPacket_, sizeof(SerialTxPacket)) == 0U) {
      txIdle_ = true;
      return false;
    }
  }

  const uint8_t status = VCP_Transmit((uint8_t *) pendingTxPacket_.payload, pendingTxPacket_.payloadSize);
  if (status == USBD_OK) {
    retry_ = 0;
    return true;
  }

  retry_ = 1;
  txIdle_ = true;
  return false;
}

void Vcp::start(STM32H7Board & board, int32_t poll_phase_offset)
{
  (void) poll_phase_offset;
  board.callbacks().register_cdc_client(this);
  rx_task_ = rxRun();
  tx_task_ = txRun();
}
