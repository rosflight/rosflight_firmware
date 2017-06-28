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

#include <stdint.h>
#include <stdbool.h>

namespace rosflight_firmware
{

class ROSflight;

class Mixer
{

public:
  enum
  {
    QUADCOPTER_PLUS,
    QUADCOPTER_X,
    Y6,
    X8,
    FIXEDWING,
    NUM_MIXERS,
    INVALID_MIXER = 255
  };

  typedef enum
  {
    NONE, // None
    S, // Servo
    M, // Motor
    G // GPIO
  } output_type_t;

  typedef struct
  {
    float F;
    float x;
    float y;
    float z;
  } command_t;

  typedef struct
  {
    output_type_t output_type[8];
    float F[8];
    float x[8];
    float y[8];
    float z[8];
  } mixer_t;

private:

  float _outputs[8];

  ROSflight& RF_;

  float prescaled_outputs[8];

  void write_motor(uint8_t index, float value);
  void write_servo(uint8_t index, float value);

  const mixer_t quadcopter_plus_mixing =
  {
    {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

    { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    { 0.0f, -1.0f,  1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    {-1.0f,  1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };


  const mixer_t quadcopter_x_mixing =
  {
    {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

    { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    {-1.0f,-1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-1.0f, 1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    {-1.0f, 1.0f, 1.0f,-1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t fixedwing_mixing =
  {
    {S, S, M, S, S, M, NONE, NONE},

    { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t X8_mixing =
  {
    {M, M, M, M, M, M, M, M},
    { 1.0f,   1.0f,    1.0f,    1.0f,    1.0f,    1.0f,   1.0f,  1.0f}, // F Mix
    {-1.0f,   1.0f,    1.0f,   -1.0f,    1.0f,   -1.0f,  -1.0f,  1.0f}, // X Mix
    { 1.0f,   1.0f,   -1.0f,   -1.0f,    1.0f,    1.0f,  -1.0f, -1.0f}, // Y Mix
    { 1.0f,  -1.0f,    1.0f,   -1.0f,    1.0f,   -1.0f,   1.0f, -1.0f}  // Z Mix
  };

  const mixer_t Y6_mixing =
  {
    {M, M, M, M, M, M, NONE, NONE},
    { 1.0f,   1.0f,    1.0f,    1.0f,    1.0f,    1.0f,   0.0f, 0.0f}, // F Mix
    {-1.0f,  -1.0f,    0.0f,    0.0f,    1.0f,    1.0f,   0.0f, 0.0f}, // X Mix
    { 0.667f, 0.667f, -1.333f, -1.333f,  0.667f,  0.667f, 0.0f, 0.0f}, // Y Mix
    {-1.0f,   1.0f,   -1.0f,    1.0f,   -1.0f,    1.0f,   0.0f, 0.0f}  // Z Mix
  };

  const mixer_t *mixer_to_use;

  const mixer_t *array_of_mixers[NUM_MIXERS] =
  {
    &quadcopter_plus_mixing,
    &quadcopter_x_mixing,
    &Y6_mixing,
    &X8_mixing,
    &fixedwing_mixing
  };

public:
  Mixer(ROSflight& _rf);
  void init();
  void init_PWM();
  void init_mixing();
  void mix_output();
  void param_change_callback(uint16_t param_id);
  inline float* get_outputs() {return _outputs;}
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MIXER_H
