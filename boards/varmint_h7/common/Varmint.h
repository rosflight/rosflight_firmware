/**
 ******************************************************************************
 * File     : Varmint.h
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

#ifndef VARMINT_H_
#define VARMINT_H_

#include "BoardConfig.h"

#include "Adc.h"
#include "Adis165xx.h"
#include "Auav.h"
#include "Bmi088.h"
#include "DlhrL20G.h"
#include "Dps310.h"
#include "Iis2mdc.h"
#include "Ist8308.h"
#include "Mcp4017.h"
#include "Ms4525.h"
#include "Pwm.h"
#include "Sbus.h"
#include "Sd.h"
#include "Telem.h"
#include "Ubx.h"
#include "Vcp.h"
#include "Lidarlitev3hp.h"
#include "Pmw3901.h"
#include "interface/board.h"

/*
 *
 */

class Varmint : public rosflight_firmware::Board
{
  /**
     * \brief
     *
     *
     */
private:
  uint32_t serial_device_;

  uint32_t status_errors_ = 0;
  uint32_t status_len_ = 0;
  Status * status_list_[STATUS_LIST_MAX_LEN];

  RcPacket rcPacket_;

public:
  Varmint(){};

  INTERFACE_LIST

  Status * status(uint32_t n) { return status_list_[n]; }
  uint32_t status_len(void) { return status_len_; }

  ////////////////////////////////////////////////////////////////////////////////
  // Required ROSflight Board HAL functions defined in board.h

  PLATFORM_OVERRIDES

};

#endif /* VARMINT_H_ */
