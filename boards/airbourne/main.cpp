/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 */

#include "airbourne_board.h"
#include "rosflight.h"
#include "mavlink.h"
#include "backup_sram.h"
#include "board.h"
#include "state_manager.h"

//#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Because this was unnecessary and annoying

uint32_t error_count_ = 0;//Used for counting resets
rosflight_firmware::ROSflight *rosflight=nullptr;//Used to access important variables in case of a hard fault
rosflight_firmware::StateManager::State get_state()//Used in case of a hard fault
{
  if (rosflight==nullptr)
  {
#pragma GCC diagnostic push //Ignore blank fields in struct
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    rosflight_firmware::StateManager::State ret = {0};
#pragma GCC diagnostic pop
    return ret;
  }
  return rosflight->state_manager_.state();
}

extern "C" {
  /* The prototype shows it is a naked function - in effect this is just an
  assembly function. */
  void HardFault_Handler(void) __attribute__((naked));

  /* The fault handler implementation calls a function called
  prvGetRegistersFromStack(). */
  void HardFault_Handler(void)
  {
    __asm volatile
    (
      " tst lr, #4                                                \n"
      " ite eq                                                    \n"
      " mrseq r0, msp                                             \n"
      " mrsne r0, psp                                             \n"
      " ldr r1, [r0, #24]                                         \n"
      " ldr r2, handler2_address_const                            \n"
      " bx r2                                                     \n"
      " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
  }

  void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
  {
    /* These are volatile to try and prevent the compiler/linker optimising them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr; /* Link register. */
    volatile uint32_t pc; /* Program counter. */
    volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    // avoid compiler warnings about unused variables
    (void) r0;
    (void) r1;
    (void) r2;
    (void) r3;
    (void) r12;
    (void) lr;
    (void) pc;
    (void) psr;

    /* When the following line is hit, the variables contain the register values. */

    //save crash information to backup SRAM
    rosflight_firmware::BackupData backup_data;
    backup_data.debug_info= {r0,r1,r2,r3,r12,lr,pc,psr};
    backup_data.reset_count=error_count_+1;
    backup_data.error_code=1;
    backup_data.state = get_state();
    if (backup_data.state.armed)
      backup_data.arm_status=rosflight_firmware::ARM_MAGIC;//magic number for extra certainty on rearm
    backup_data.checksum=generate_backup_checksum(backup_data);
    backup_sram_write(backup_data);

    NVIC_SystemReset();
  }

  void NMI_Handler()
  {
    while (1) {}
  }


  void MemManage_Handler()
  {
    while (1) {}
  }

  void BusFault_Handler()
  {
    while (1) {}
  }

  void UsageFault_Handler()
  {
    while (1) {}
  }

  void WWDG_IRQHandler()
  {
    while (1) {}
  }
}



int main(void)
{
  rosflight_firmware::AirbourneBoard board;
  rosflight_firmware::Mavlink mavlink(board);
  rosflight_firmware::ROSflight firmware(board, mavlink);
  rosflight = &firmware; //this allows crashes to grab some info
  board.init_board();
  firmware.init();
  //Because the USB driver breaks the backup sram, the backup sram must be initalized after
  backup_sram_init();
  rosflight_firmware::BackupData backup_data = backup_sram_read();
  error_count_ = backup_data.reset_count;

  while (true)
  {
    firmware.run();
  }
  return 0;
}
