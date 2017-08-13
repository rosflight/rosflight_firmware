/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson  BYU MAGICC Lab, Provo UT
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

#pragma once

#include <stdint.h>

namespace turbomath
{

class vector
{
public:
  float x;
  float y;
  float z;

  vector();
  vector(float x_, float y_, float z_);

  float norm() const;
  float sqrd_norm() const;
  vector &normalize();
  vector normalized() const;

  float dot(const vector v) const;
  vector cross(const vector v) const;

  vector operator* (const float s) const;
  vector operator/ (const float s) const;
  vector& operator*= (const float s);
  vector& operator/= (const float s);
  vector operator+ (const vector v) const;
  vector operator- (const vector v) const;
  vector& operator+= (const vector v);
  vector& operator-= (const vector v);
};

class quaternion
{
public:
  float w;
  float x;
  float y;
  float z;
  quaternion();
  quaternion(float w_, float x_, float y_, float z_);
  quaternion(const vector u, const vector v);
  vector rotate(const vector v) const;
  vector operator* (const vector v) const;
  quaternion& normalize();
  quaternion operator* (const quaternion q) const;
  quaternion& operator*= (const quaternion q);
  quaternion inverse() const;
  quaternion& invert();
  quaternion& from_two_unit_vectors(const vector u, const vector v);
  void get_RPY(float& roll, float& pitch, float& yaw) const;
};

// float-based wrappers
float atan2(float y, float x);
float asin(float x);

// turbo-speed trig approximation
float atan(float x);
float fsign(float y);

// turbo-speed approximation of (1.0 - pow(pressure/101325.0, 0.1902631)) * 39097.63
// Used for calculating altitude in m from atmospheric pressure in Pa
float alt(float x);

float inv_sqrt(float x);
float fabs(float x);

}
