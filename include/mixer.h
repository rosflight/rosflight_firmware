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

#ifndef ROSFLIGHT_FIRMWARE_MIXER_H
#define ROSFLIGHT_FIRMWARE_MIXER_H

#include "interface/param_listener.h"
#include "controller.h"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
class ROSflight;

class Mixer : public ParamListenerInterface
{
public:
  static constexpr uint8_t NUM_TOTAL_OUTPUTS = 14;
  static constexpr uint8_t NUM_MIXER_OUTPUTS = 10;

  enum
  {
    ESC_CALIBRATION = 0,
    QUADCOPTER_PLUS = 1,
    QUADCOPTER_X = 2,
    HEX_PLUS = 3,
    HEX_X = 4,
    OCTO_PLUS = 5,
    OCTO_X = 6,
    Y6 = 7,
    X8 = 8,
    FIXEDWING = 9,
    INVERTED_VTAIL = 10,
    CUSTOM = 11,
    NUM_MIXERS,
    INVALID_MIXER = 255
  };

  typedef enum
  {
    NONE, // None
    S,    // Servo
    M,    // Motor
    G     // GPIO
  } output_type_t;

  typedef struct
  {
    output_type_t output_type[NUM_MIXER_OUTPUTS];
    float default_pwm_rate[NUM_MIXER_OUTPUTS];
    float Fx[NUM_MIXER_OUTPUTS];
    float Fy[NUM_MIXER_OUTPUTS];
    float Fz[NUM_MIXER_OUTPUTS];
    float Qx[NUM_MIXER_OUTPUTS];
    float Qy[NUM_MIXER_OUTPUTS];
    float Qz[NUM_MIXER_OUTPUTS];
  } mixer_t;

  typedef struct
  {
    output_type_t type;
    float value;
  } aux_channel_t;

  typedef struct
  {
    aux_channel_t channel[NUM_TOTAL_OUTPUTS];
  } aux_command_t;

private:
  ROSflight & RF_;

  float raw_outputs_[NUM_TOTAL_OUTPUTS];
  float outputs_[NUM_TOTAL_OUTPUTS];
  aux_command_t aux_command_;
  output_type_t combined_output_type_[NUM_TOTAL_OUTPUTS];

  void write_motor(uint8_t index, float value);
  void write_servo(uint8_t index, float value);
  void add_header_to_mixer(mixer_t* mixer);
  void load_primary_mixer_values();
  mixer_t invert_mixer(const mixer_t* mixer_to_invert);
  float mix_multirotor_with_motor_parameters(Controller::Output commands);
  float mix_multirotor_without_motor_parameters(Controller::Output commands);

  // clang-format off

  const mixer_t esc_calibration_mixing = {
    {   M,    M,    M,    M,    M,    M,    M,    M,    M,    M},  // output type
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_x Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_y Mix
    {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},  // F_z Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_x Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Q_z Mix

  const mixer_t quadcopter_plus_mixing = {
    {M, M, M, M, NONE, NONE, NONE, NONE, NONE, NONE},            // output type
    {490, 490, 490, 490, 50, 50, 50, 50, 50, 50},                // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0, 0, 0},  // F_y Mix
    {-0.2500f, -0.2500f, -0.2500f, -0.2500f, 0, 0, 0, 0, 0, 0},  // F_z Mix
    { 0.0000f, -1.0000f,  0.0000f,  1.0000f, 0, 0, 0, 0, 0, 0},  // Q_x Mix
    { 1.0000f,  0.0000f, -1.0000f,  0.0000f, 0, 0, 0, 0, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0, 0, 0, 0, 0}}; // Q_z Mix 

  const mixer_t quadcopter_x_mixing = {
    {M, M, M, M, NONE, NONE, NONE, NONE, NONE, NONE},             // output type
    {490, 490, 490, 490, 50, 50, 50, 50, 50, 50},                 // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0, 0, 0},   // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0, 0, 0},   // F_y Mix
    {-0.2500f, -0.2500f, -0.2500f, -0.2500f, 0, 0, 0, 0, 0, 0},   // F_z Mix
    {-0.7071f, -0.7071f,  0.7071f,  0.7071f, 0, 0, 0, 0, 0, 0},   // Q_x Mix
    { 0.7071f, -0.7071f, -0.7071f,  0.7071f, 0, 0, 0, 0, 0, 0},   // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0, 0, 0, 0, 0}};  // Q_z Mix

  const mixer_t hex_plus_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},                                // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                          // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_y Mix
    {-0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, 0, 0, 0, 0},  // F_z Mix
    { 0.0000f, -0.8660f, -0.8660f,  0.0000f,  0.8660f,  0.8660f, 0, 0, 0, 0},  // Q_x Mix
    { 1.0000f,  0.5000f, -0.5000f, -1.0000f, -0.5000f,  0.5000f, 0, 0, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0, 0, 0}}; // Q_z Mix

  const mixer_t hex_x_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},                                // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                          // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_y Mix
    {-0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, 0, 0, 0, 0},  // F_z Mix
    {-0.5000f, -1.0000f, -0.5000f,  0.5000f,  1.0000f,  0.5000f, 0, 0, 0, 0},  // Q_x Mix
    { 0.8660f,  0.0000f, -0.8660f, -0.8660f,  0.0000f,  0.8660f, 0, 0, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0, 0, 0}}; // Q_z Mix

  const mixer_t octocopter_plus_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},                                                    // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                                        // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_y Mix
    {-0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, 0, 0},  // F_z Mix
    { 0.0000f, -0.7071f, -1.0000f, -0.7071f,  0.0000f,  0.7071f,  1.0000f,  0.7071f, 0, 0},  // Q_x Mix
    { 1.0000f,  0.7071f,  0.0000f, -0.7071f, -1.0000f, -0.7071f,  0.0000f,  0.7071f, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0}}; // Q_z Mix

  const mixer_t octocopter_x_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},                                                    // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                                        // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_y Mix
    {-0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, 0, 0},  // F_z Mix
    {-0.3827f, -0.9239f, -0.9239f, -0.3827f,  0.3827f,  0.9239f,  0.9239f,  0.3827f, 0, 0},  // Q_x Mix
    { 0.9239f,  0.3827f, -0.3827f, -0.9239f, -0.9239f, -0.3827f,  0.3827f,  0.9239f, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0}}; // Q_z Mix

  const mixer_t Y6_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},                                // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                          // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0, 0, 0},  // F_y Mix
    {-0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, -0.1667f, 0, 0, 0, 0},  // F_z Mix
    {-0.8660f, -0.8660f,  0.0000f,  0.0000f,  0.8660f,  0.8660f, 0, 0, 0, 0},  // Q_x Mix
    { 0.5000f,  0.5000f, -1.0000f, -1.0000f,  0.5000f,  0.5000f, 0, 0, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0, 0, 0}}; // Q_z Mix

  const mixer_t X8_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},                                                    // output type
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},                                        // Rate (Hz)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_x Mix
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f, 0, 0},  // F_y Mix
    {-0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, -0.1250f, 0, 0},  // F_z Mix
    {-0.7071f, -0.7071f, -0.7071f, -0.7071f,  0.7071f,  0.7071f,  0.7071f,  0.7071f, 0, 0},  // Q_x Mix
    { 0.7071f,  0.7071f, -0.7071f, -0.7071f, -0.7071f, -0.7071f,  0.7071f,  0.7071f, 0, 0},  // Q_y Mix
    { 1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f,  1.0000f, -1.0000f, 0, 0}}; // Q_z Mix

  const mixer_t fixedwing_mixing = {
    {   S,    S,    S, NONE,    M, NONE, NONE, NONE, NONE, NONE},  // output type
    { 50,    50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_x Mix 
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_z Mix
    {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_x Mix 
    {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_y Mix 
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Q_z Mix 

  const mixer_t fixedwing_inverted_vtail_mixing = {
    {   S,     S,    S, NONE,    M, NONE, NONE, NONE, NONE, NONE},  // Ailerons, LRuddervator, RRuddervator, Motor
    {  50,    50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f,  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_x Mix 
    {0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_y Mix
    {0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_z Mix
    {1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_x Mix 
    {0.0f, -0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_y Mix 
    {0.0f,  0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Q_z Mix 

  const mixer_t custom_mixing = {
    {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},  // output type
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz or kHz)
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_x Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F_z Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_x Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Q_y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Q_z Mix

  mixer_t primary_mixer_;

  const mixer_t * mixer_to_use_;
  bool use_motor_parameters_;

  const mixer_t* array_of_mixers_[NUM_MIXERS] = {
    &esc_calibration_mixing,
    &quadcopter_plus_mixing,
    &quadcopter_x_mixing,
    &hex_plus_mixing,
    &hex_x_mixing,
    &octocopter_plus_mixing,
    &octocopter_x_mixing,
    &Y6_mixing,
    &X8_mixing,
    &fixedwing_mixing,
    &fixedwing_inverted_vtail_mixing,
    &custom_mixing,
  };

  // clang-format on

  // Define parameters for the mixer
  float R_;                      // Motor resistance
  float rho_;                    // Air density
  float K_V_;                    // Motor back-emf constant
  float K_Q_ = 0.01706;          // Motor torque constant
  float i_0_;                    // Motor no-load current
  float D_;                      // Propeller diameter
  float C_T_;                    // Thrust coefficient
  float C_Q_;                    // Torque coefficient
  int num_motors_;               // Number of motors
  float V_max_;                  // Maximum battery voltage
  float l_[NUM_MIXER_OUTPUTS];   // Radial distance from center of mass to motor
  float psi_[NUM_MIXER_OUTPUTS]; // Angle of motor from body x-axis

public:
  Mixer(ROSflight & _rf);
  void init();
  void init_PWM();
  void init_mixing();
  void update_parameters();
  void mix_output();
  void param_change_callback(uint16_t param_id) override;
  void set_new_aux_command(aux_command_t new_aux_command);
  inline const float * get_outputs() const { return raw_outputs_; }
  inline const bool use_motor_parameters() const { return use_motor_parameters_; }

  void calculate_mixer_values();
  void mix_multirotor();
  void mix_fixedwing();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MIXER_H
