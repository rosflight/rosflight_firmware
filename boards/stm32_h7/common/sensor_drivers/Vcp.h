/**
 ******************************************************************************
 * File     : Vcp.h
 * Date     : Oct 4, 2023
 ******************************************************************************
 **/

#ifndef VCP_H_
#define VCP_H_

#include "Async.h"
#include "BoardConfig.h"
#include "ByteFifo.h"
#include "PacketFifo.h"
#include "Packets.h"

class STM32H7Board;

class Vcp : public Status
{
public:
  uint32_t init(uint16_t sample_rate_hz);
  void start(STM32H7Board & board, int32_t poll_phase_offset = 0);

  uint16_t writePacket(SerialTxPacket * p);
  bool isMy(uint8_t chan) { return channel_ == chan; }
  void cdcReceiveCallback(uint8_t * buffer, uint16_t size);
  void cdcTransmitCpltCallback(void);

  uint16_t byteCount(void) { return rxFifo_.byteCount(); }
  bool readByte(uint8_t * data) { return rxFifo_.read(data); }

private:
  AsyncTask<void> rxRun();
  AsyncTask<void> txRun();
  bool txStart(void);

  int16_t sampleRateHz_;
  PacketFifo txFifo_;
  PacketFifo rxPacketFifo_;
  bool txIdle_;
  uint16_t retry_;
  SerialTxPacket pendingTxPacket_ = {};

  ByteFifo rxFifo_;
  AcquisitionSignal rx_signal_;
  AcquisitionSignal tx_signal_;
  AsyncTask<void> rx_task_;
  AsyncTask<void> tx_task_;
  uint8_t channel_ = 0;
};
#endif /* VCP_H_ */
