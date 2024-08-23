/**
 ******************************************************************************
 * File     : Vcp.h
 * Date     : Oct 4, 2023
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

#ifndef VCP_H_
#define VCP_H_

#include <BoardConfig.h>
#include <ByteFifo.h>
#include <Packets.h>

/**
 * @class Vcp
 * @brief
 *
 */

class Vcp
{
public:
  Vcp() { initializationStatus_ = DRIVER_NOT_INITIALIZED;}
  bool initGood(void) { return initializationStatus_== DRIVER_OK;}

  uint32_t init(uint16_t sample_rate_hz);

  void poll(void);
  bool display(void) { return false; }

  uint16_t writePacket(SerialTxPacket * p);
  void txCdcCallback(void);
  void txStart(void);

  void rxCdcCallback(uint8_t * buffer, uint16_t size) { rxFifo_.writeBlock(buffer, size); }
  uint16_t byteCount(void) { return rxFifo_.byteCount(); }
  bool readByte(uint8_t * data) { return rxFifo_.read(data); }

private:
  int16_t sampleRateHz_;
  uint64_t txTimeout_ = 0;
  uint64_t txDtimeout_;

  PacketFifo txFifo_;
  bool txIdle_;
  uint16_t retry_;

  ByteFifo rxFifo_;
  uint32_t initializationStatus_ = DRIVER_NOT_INITIALIZED;
};
#endif /* VCP_H_ */
