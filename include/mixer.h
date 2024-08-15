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
    TRICOPTER = 9,
    FIXEDWING = 10,
    PASSTHROUGH = 11,
    INVERTED_VTAIL = 12,
    QUADPLANE = 13,
    CUSTOM = 14,
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
    float F[NUM_MIXER_OUTPUTS];
    float x[NUM_MIXER_OUTPUTS];
    float y[NUM_MIXER_OUTPUTS];
    float z[NUM_MIXER_OUTPUTS];
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

  // clang-format off

  const mixer_t esc_calibration_mixing = {
    {   M,    M,    M,    M,    M,    M, NONE, NONE, NONE, NONE},  // output type
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},  // F Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // X Mix

  const mixer_t quadcopter_plus_mixing = {
    {   M,     M,     M,     M, NONE, NONE, NONE, NONE, NONE, NONE},  // output_type
    { 490,   490,   490,   490,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {5098.031218379775, 5098.0312183797805, 5098.031218379778, 5098.0312183797805, 0, 0, 0, 0, 0, 0},
    {0.0, -40784.24974703961, 0.0, 40784.24974703687, 0, 0, 0, 0, 0, 0},
    {40784.249747038244, 0.0, -40784.24974703822, 0.0, 0, 0, 0, 0, 0, 0},
    {223010.98943043666, -223010.98943043654, 223010.98943043666, -223010.98943043657, 0, 0, 0, 0, 0, 0}};

  const mixer_t quadcopter_x_mixing = {
    {    M,     M,     M,     M, NONE, NONE, NONE, NONE, NONE, NONE},  // output_type
    {  490,   490,   490,   490,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {5098.031218379779, 5098.03121837978, 5098.031218379777, 5098.031218379782, 0, 0, 0, 0, 0, 0},
    {-28838.81956173643, -28838.819561736505, 28838.81956173648, 28838.819561736465, 0, 0, 0, 0, 0, 0},
    {28838.819561736495, -28838.819561736498, -28838.819561736433, 28838.81956173645, 0, 0, 0, 0, 0, 0},
    {223010.9894304366, -223010.9894304366, 223010.98943043666, -223010.9894304366, 0, 0, 0, 0, 0, 0}};

  const mixer_t hex_plus_mixing = {
    {   M,          M,          M,     M,         M,         M, NONE, NONE, NONE, NONE},  // output_type
    { 490,        490,        490,   490,       490,       490,   50,   50,   50,   50},  // Rate (Hz)
    {3398.687478919854, 3398.6874789198537, 3398.6874789198537, 3398.6874789198523, 3398.6874789198537, 3398.6874789198537, 0, 0, 0, 0},
    {0.0, -23546.797570149454, -23546.797570149447, 0.0, 23546.797570149443, 23546.79757014946, 0, 0, 0, 0},
    {27189.49983135882, 13594.749915679433, -13594.749915679424, -27189.49983135882, -13594.749915679417, 13594.749915679415, 0, 0, 0, 0},
    {148673.99295362437, -148673.99295362443, 148673.99295362437, -148673.99295362437, 148673.9929536245, -148673.9929536244, 0, 0, 0, 0}};

  const mixer_t hex_x_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 50, 50, 50, 50},
    {3398.687478919854, 3398.687478919853, 3398.687478919853, 3398.6874789198523, 3398.687478919854, 3398.687478919854, 0, 0, 0, 0},
    {-13594.749915679407, -27189.49983135883, -13594.74991567941, 13594.749915679384, 27189.49983135884, 13594.74991567942, 0, 0, 0, 0},
    {23546.797570149465, 0.0, -23546.797570149454, -23546.797570149472, 0.0, 23546.797570149465, 0, 0, 0, 0},
    {148673.99295362426, -148673.9929536243, 148673.99295362443, -148673.99295362443, 148673.99295362446, -148673.99295362437, 0, 0, 0, 0}};

  const mixer_t octocopter_plus_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},
    {2549.0156091898907, 2549.0156091898903, 2549.01560918989, 2549.0156091898893, 2549.015609189889, 2549.0156091898893, 2549.01560918989, 2549.0156091898903, 0, 0},
    {0.0, -14419.409780868224, -20392.124873519115, -14419.409780868233, 0.0, 14419.409780868235, 20392.124873519115, 14419.409780868242, 0, 0},
    {20392.124873519126, 14419.409780868244, 0.0, -14419.409780868249, -20392.124873519126, -14419.409780868249, 0.0, 14419.409780868238, 0, 0},
    {111505.49471521821, -111505.4947152183, 111505.49471521829, -111505.49471521829, 111505.49471521827, -111505.49471521826, 111505.49471521827, -111505.49471521827, 0, 0}};

  const mixer_t octocopter_x_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},
    {2549.0156091898893, 2549.01560918989, 2549.01560918989, 2549.0156091898903, 2549.0156091898903, 2549.01560918989, 2549.01560918989, 2549.0156091898893, 0, 0},
    {-7803.728339815821, -18839.866795058642, -18839.86679505863, -7803.728339815828, 7803.728339815823, 18839.86679505863, 18839.86679505864, 7803.728339815834, 0, 0},
    {18839.86679505862, 7803.728339815822, -7803.728339815816, -18839.866795058624, -18839.86679505863, -7803.728339815828, 7803.728339815817, 18839.86679505862, 0, 0},
    {111505.49471521824, -111505.49471521845, 111505.49471521832, -111505.49471521827, 111505.49471521837, -111505.49471521827, 111505.49471521833, -111505.49471521836, 0, 0}};

  const mixer_t Y6_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 50, 50, 50, 50},
    {3398.6874789198546, 3398.6874789198523, 3398.6874789198537, 3398.6874789198546, 3398.687478919853, 3398.6874789198528, 0, 0, 0, 0},
    {-23546.79757014947, 0.0, 23546.797570149454, -23546.79757014944, 0.0, 23546.79757014947, 0, 0, 0, 0},
    {13594.749915679397, -27189.499831358822, 13594.749915679413, 13594.749915679437, -27189.499831358844, 13594.749915679426, 0, 0, 0, 0},
    {148673.9929536246, -148673.9929536245, 148673.9929536245, -148673.99295362452, 148673.99295362446, -148673.9929536245, 0, 0, 0, 0}};

  const mixer_t X8_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 490, 490, 50, 50},
    {2549.01560918989, 2549.01560918989, 2549.0156091898893, 2549.0156091898893, 2549.01560918989, 2549.01560918989, 2549.0156091898903, 2549.0156091898903, 0, 0},
    {-14419.409780868245, -14419.409780868247, -14419.40978086824, -14419.409780868236, 14419.409780868244, 14419.409780868247, 14419.40978086824, 14419.409780868244, 0, 0},
    {14419.409780868236, 14419.40978086824, -14419.409780868247, -14419.409780868247, -14419.40978086824, -14419.409780868244, 14419.409780868249, 14419.409780868245, 0, 0},
    {111505.49471521829, -111505.49471521827, 111505.49471521823, -111505.49471521829, 111505.49471521827, -111505.49471521826, 111505.49471521829, -111505.49471521823, 0, 0}};

  const mixer_t tricopter_mixing = {
    {M, M, M, M, M, M, NONE, NONE, NONE, NONE},
    {490, 490, 490, 490, 490, 490, 50, 50, 50, 50},
    {3398.687478919855, 3398.687478919855, 3398.6874789198528, 3398.687478919852, 3398.6874789198537, 3398.6874789198528, 0, 0, 0, 0},
    {-23546.797570149458, -23546.797570149447, 0.0, 0.0, 23546.797570149454, 23546.797570149465, 0, 0, 0, 0},
    {13594.749915679417, 13594.74991567943, -27189.499831358844, -27189.49983135883, 13594.749915679406, 13594.749915679418, 0, 0, 0, 0},
    {148673.99295362437, -148673.99295362429, 148673.99295362443, -148673.9929536245, 148673.9929536244, -148673.99295362443, 0, 0, 0, 0}};

  const mixer_t fixedwing_mixing = {
    {   S,    S,    S, NONE,    M, NONE, NONE, NONE, NONE, NONE},  // output type
    { 50,    50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
    {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t fixedwing_inverted_vtail_mixing = {
    {   S, NONE, NONE,     S,    S,    M, NONE, NONE, NONE, NONE},  // Ailerons, LRuddervator, RRuddervator, Motor
    {  50,   50,   50,    50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
    {1.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, -0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f, 0.0f, 0.0f,  0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t quadplane_mixing = {
    {   S,     S,    S,    M,    M,     M,     M,     M, NONE, NONE},  // Ailerons, Rudder, Elevator, Tractor Motor, Quadrotors
    {  50,    50,   50,   50,  490,   490,   490,   490,   50,   50},  // Rate (Hz)
    {0.0f,  0.0f, 0.0f, 1.0f, 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f},  // F Mix
    {1.0f,  0.0f, 0.0f, 0.0f, 0.0f, -1.0f,  0.0f,  1.0f, 0.0f, 0.0f},  // X Mix
    {0.0f,  1.0f, 0.0f, 0.0f, 1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f,  0.0f, 1.0f, 0.0f, 1.0f, -1.0f,  1.0f, -1.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t passthrough_mixing = {
    {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz or kHz)
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t custom_mixing = {
    {NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},  // output type
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz or kHz)
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  mixer_t custom_mixer_;

  const mixer_t * mixer_to_use_;

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
    &tricopter_mixing,
    &fixedwing_mixing,
    &passthrough_mixing,
    &fixedwing_inverted_vtail_mixing,
    &quadplane_mixing,
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

  void calculate_mixer_values();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MIXER_H
