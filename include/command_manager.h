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

#include "param_listener.h"

#include "rc.h"

#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
class ROSflight;

typedef enum
{
  RATE,        // Channel is is in rate mode (mrad/s)
  ANGLE,       // Channel command is in angle mode (mrad)
  THROTTLE,    // Channel is controlling throttle setting, which will be converted to force
  PASSTHROUGH, // Channel directly passes PWM input to the mixer
} control_type_t;

typedef struct
{
  bool active;         // Whether or not the channel is active
  control_type_t type; // What type the channel is
  float value;         // The value of the channel
} control_channel_t;

typedef enum
{
  X_AXIS,
  Y_AXIS,
  Z_AXIS,
} rc_f_axis_t;

typedef struct
{
  uint32_t stamp_ms;
  control_channel_t Qx;
  control_channel_t Qy;
  control_channel_t Qz;
  control_channel_t Fx;
  control_channel_t Fy;
  control_channel_t Fz;
} control_t;

class CommandManager : public ParamListenerInterface
{
private:
  typedef struct
  {
    control_channel_t * rc;
    control_channel_t * onboard;
    control_channel_t * combined;
  } mux_t;

  mux_t muxes[6] = {{&rc_command_.Qx, &offboard_command_.Qx, &combined_command_.Qx},
                    {&rc_command_.Qy, &offboard_command_.Qy, &combined_command_.Qy},
                    {&rc_command_.Qz, &offboard_command_.Qz, &combined_command_.Qz},
                    {&rc_command_.Fx, &offboard_command_.Fx, &combined_command_.Fx},
                    {&rc_command_.Fy, &offboard_command_.Fy, &combined_command_.Fy},
                    {&rc_command_.Fz, &offboard_command_.Fz, &combined_command_.Fz}};

  // clang-format off
  control_t rc_command_ = {0,
                           {false, ANGLE, 0.0},
                           {false, ANGLE, 0.0},
                           {false, RATE, 0.0},
                           {false, THROTTLE, 0.0},
                           {false, THROTTLE, 0.0},
                           {false, THROTTLE, 0.0}};
  control_t offboard_command_ = {0,
                                 {false, ANGLE, 0.0},
                                 {false, ANGLE, 0.0},
                                 {false, RATE, 0.0},
                                 {false, THROTTLE, 0.0},
                                 {false, THROTTLE, 0.0},
                                 {false, THROTTLE, 0.0}};
  control_t combined_command_ = {0,
                                 {false, ANGLE, 0.0},
                                 {false, ANGLE, 0.0},
                                 {false, RATE, 0.0},
                                 {false, THROTTLE, 0.0},
                                 {false, THROTTLE, 0.0},
                                 {false, THROTTLE, 0.0}};

  control_t multirotor_failsafe_command_ = {0,
                                            {true, ANGLE, 0.0},
                                            {true, ANGLE, 0.0},
                                            {true, RATE, 0.0},
                                            {true, THROTTLE, 0.0},
                                            {true, THROTTLE, 0.0},
                                            {true, THROTTLE, 0.0}};
  control_t fixedwing_failsafe_command_ = {0,
                                           {true, PASSTHROUGH, 0.0},
                                           {true, PASSTHROUGH, 0.0},
                                           {true, PASSTHROUGH, 0.0},
                                           {true, PASSTHROUGH, 0.0},
                                           {true, PASSTHROUGH, 0.0},
                                           {true, PASSTHROUGH, 0.0}};
  // clang-format on

  typedef enum
  {
    ATT_MODE_RATE,
    ATT_MODE_ANGLE
  } att_mode_t;

  enum MuxChannel
  {
    MUX_QX,
    MUX_QY,
    MUX_QZ,
    MUX_FX,
    MUX_FY,
    MUX_FZ,
  };

  typedef struct
  {
    RC::Stick rc_channel;
    uint32_t last_override_time;
  } rc_stick_override_t;

  rc_stick_override_t rc_stick_override_[3] = {{RC::STICK_X, 0},
                                               {RC::STICK_Y, 0},
                                               {RC::STICK_Z, 0}};

  ROSflight & RF_;

  bool new_command_;
  bool rc_throttle_override_;
  bool rc_attitude_override_;

  control_t & failsafe_command_;

  void param_change_callback(uint16_t param_id) override;
  void init_failsafe();

  bool do_roll_pitch_yaw_muxing(MuxChannel channel);
  bool do_throttle_muxing(void);
  void do_min_throttle_muxing();

  void interpret_rc(void);
  bool stick_deviated(MuxChannel channel);

public:
  CommandManager(ROSflight & _rf);
  void init();
  bool run();
  bool rc_override_active();
  bool rc_throttle_override_active();
  bool rc_attitude_override_active();
  bool offboard_control_active();
  void set_new_offboard_command(control_t new_offboard_command);
  void set_new_rc_command(control_t new_rc_command);
  void override_combined_command_with_rc();
  inline const control_t & combined_control() const { return combined_command_; }
  inline const control_t & rc_control() const { return rc_command_; }
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMMAND_MANAGER_H
