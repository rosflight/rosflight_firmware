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

#ifndef ROSFLIGHT_FIRMWARE_COMMAND_MANAGER_H
#define ROSFLIGHT_FIRMWARE_COMMAND_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "interface/param_listener.h"

#include "rc.h"

namespace rosflight_firmware
{

class ROSflight;

typedef enum
{
  RATE,         // Channel is is in rate mode (mrad/s)
  ANGLE,        // Channel command is in angle mode (mrad)
  THROTTLE,     // Channel is direcly controlling throttle max/1000
  PASSTHROUGH,  // Channel directly passes PWM input to the mixer
} control_type_t;

typedef struct
{
  bool active;          // Whether or not the channel is active
  control_type_t type;  // What type the channel is
  float value;          // The value of the channel
} control_channel_t;

typedef struct
{
  uint32_t stamp_ms;
  control_channel_t x;
  control_channel_t y;
  control_channel_t z;
  control_channel_t F;
} control_t;

class CommandManager : public ParamListenerInterface
{

private:

  typedef struct
  {
    control_channel_t *rc;
    control_channel_t *onboard;
    control_channel_t *combined;
  } mux_t;

  mux_t muxes[4] =
  {
    {&rc_command_.x, &offboard_command_.x, &combined_command_.x},
    {&rc_command_.y, &offboard_command_.y, &combined_command_.y},
    {&rc_command_.z, &offboard_command_.z, &combined_command_.z},
    {&rc_command_.F, &offboard_command_.F, &combined_command_.F}
  };

  control_t rc_command_ =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };
  control_t offboard_command_ =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };
  control_t combined_command_ =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };

  control_t multirotor_failsafe_command_ =
  {
    0,
    {true, ANGLE, 0.0},
    {true, ANGLE, 0.0},
    {true, RATE, 0.0},
    {true, THROTTLE, 0.0}
  };
  control_t fixedwing_failsafe_command_ =
  {
    0,
    {true, PASSTHROUGH, 0.0},
    {true, PASSTHROUGH, 0.0},
    {true, PASSTHROUGH, 0.0},
    {true, THROTTLE, 0.0}
  };

  typedef enum
  {
    ATT_MODE_RATE,
    ATT_MODE_ANGLE
  } att_mode_t;

  enum MuxChannel
  {
    MUX_X,
    MUX_Y,
    MUX_Z,
    MUX_F,
  };
  enum RCOverrideReason: uint16_t
  {
    OVERRIDE_NO_OVERRIDE = 0x0,
    OVERRIDE_ATT_SWITCH = 0x1,
    OVERRIDE_THR_SWITCH = 0x2,
    OVERRIDE_X = 0x4,
    OVERRIDE_Y = 0x8,
    OVERRIDE_Z = 0x10,
    OVERRIDE_T = 0x20,
    OVERRIDE_OFFBOARD_X_INACTIVE = 0x40,
    OVERRIDE_OFFBOARD_Y_INACTIVE = 0x80,
    OVERRIDE_OFFBOARD_Z_INACTIVE = 0x100,
    OVERRIDE_OFFBOARD_T_INACTIVE = 0x200,
  };
  static constexpr uint16_t X_OVERRIDDEN{OVERRIDE_ATT_SWITCH | OVERRIDE_X | OVERRIDE_OFFBOARD_X_INACTIVE};
  static constexpr uint16_t Y_OVERRIDDEN{OVERRIDE_ATT_SWITCH | OVERRIDE_Y | OVERRIDE_OFFBOARD_Y_INACTIVE};
  static constexpr uint16_t Z_OVERRIDDEN{OVERRIDE_ATT_SWITCH | OVERRIDE_Z | OVERRIDE_OFFBOARD_Z_INACTIVE};
  static constexpr uint16_t T_OVERRIDDEN{OVERRIDE_THR_SWITCH | OVERRIDE_T | OVERRIDE_OFFBOARD_T_INACTIVE};

  typedef struct
  {
    RC::Stick rc_channel;
    uint32_t last_override_time;
    RCOverrideReason stick_override_reason;
    RCOverrideReason offboard_inactive_override_reason;
    uint16_t override_mask;
  } channel_override_t;

  channel_override_t channel_override_[4] =
  {
    { RC::STICK_X, 0, OVERRIDE_X, OVERRIDE_OFFBOARD_X_INACTIVE, X_OVERRIDDEN},
    { RC::STICK_Y, 0, OVERRIDE_Y, OVERRIDE_OFFBOARD_Y_INACTIVE, Y_OVERRIDDEN},
    { RC::STICK_Z, 0, OVERRIDE_Z, OVERRIDE_OFFBOARD_Z_INACTIVE, Z_OVERRIDDEN},
    { RC::STICK_F, 0, OVERRIDE_T, OVERRIDE_OFFBOARD_T_INACTIVE, T_OVERRIDDEN} // Note that throttle overriding works a bit differently
  };

  ROSflight &RF_;

  bool new_command_;
  uint16_t rc_override_;

  control_t &failsafe_command_;

  void param_change_callback(uint16_t param_id) override;
  void init_failsafe();

  /**
   * @brief Checks which channels are overridden
   * @details There are many reasons that a channel could be overriden. These reasons include:
   * 	- A stick is deviated
   * 	-	The commanded throttle is less than the RC throttle, and the MIN_THROTTLE parameter is set
   * 	-	The attitude or throttle override switch is flipped
   * 	-	The onboard computer has not sent any commands recently
   * The returned bitfield indicates which reasons have caused an override.
   * By anding with a constant such as X_OVERRIDDEN, you can check if a specific channel is overridden.
   * @return A bitfield, with overriden reasons indicated
   */
  uint16_t determine_override_status();
  void do_muxing(uint16_t rc_override);
  void do_channel_muxing(MuxChannel channel, uint16_t rc_override);

  void interpret_rc(void);
  bool stick_deviated(MuxChannel channel);

public:

  CommandManager(ROSflight &_rf);
  void init();
  bool run();
  bool rc_override_active();
  bool offboard_control_active();
  void set_new_offboard_command(control_t new_offboard_command);
  void set_new_rc_command(control_t new_rc_command);
  void override_combined_command_with_rc();
  inline const control_t &combined_control() const { return combined_command_; }
  inline const control_t &rc_control() const { return rc_command_; }
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMMAND_MANAGER_H
