/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson BYU MAGICC Lab, Provo UT
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

#pragma GCC diagnostic ignored "-Wstrict-aliasing"


#include <turbotrig/turbovec.h>
#include <turbotrig/turbotrig.h>


void pfquat() __attribute__((unused));


/**************************************/
/*** FLOATING POINT IMPLEMENTATIONS ***/
/**************************************/
float dot(vector_t v1, vector_t v2)
{
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

vector_t cross(vector_t u, vector_t v)
{
  vector_t out = {u.y * v.z - u.z * v.y,
                  u.z * v.x - u.x * v.z,
                  u.x * v.y - u.y * v.x
                 };
  return out;
}

vector_t scalar_multiply(float s, vector_t v)
{
  vector_t out = {s * v.x,
                  s * v.y,
                  s * v.z
                 };
  return out;
}

vector_t vector_add(vector_t u, vector_t v)
{
  vector_t out = {u.x + v.x,
                  u.y + v.y,
                  u.z + v.z
                 };
  return out;

}

vector_t vector_sub(vector_t u, vector_t v)
{
  vector_t out = {u.x - v.x,
                  u.y - v.y,
                  u.z - v.z
                 };
  return out;
}

float sqrd_norm(vector_t v)
{
  return v.x*v.x + v.y*v.y + v.z*v.z;
}

float norm(vector_t v)
{
  float out = 1.0/turboInvSqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  return out;
}

vector_t vector_normalize(vector_t v)
{
  float recipNorm = turboInvSqrt(v.x*v.x + v.y*v.y + v.z*v.z);
  vector_t out = {recipNorm*v.x,
                  recipNorm*v.y,
                  recipNorm*v.z
                 };
  return out;
}

quaternion_t quaternion_normalize(quaternion_t q)
{
  float recipNorm = turboInvSqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  quaternion_t out = {recipNorm*q.w,
                      recipNorm*q.x,
                      recipNorm*q.y,
                      recipNorm*q.z};
  return out;
}

quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2)
{
  quaternion_t q = {q1.w *q2.w - q1.x *q2.x - q1.y *q2.y - q1.z*q2.z,
                    q1.w *q2.x + q1.x *q2.w - q1.y *q2.z + q1.z*q2.y,
                    q1.w *q2.y + q1.x *q2.z + q1.y *q2.w - q1.z*q2.x,
                    q1.w *q2.z - q1.x *q2.y + q1.y *q2.x + q1.z *q2.w
                   };
  return q;
}

quaternion_t quaternion_inverse(quaternion_t q)
{
  q.x *= -1.0f;
  q.y *= -1.0f;
  q.z *= -1.0f;
  return q;
}

vector_t rotate_vector(quaternion_t q, vector_t v)
{
  vector_t out = { (1.0f - 2.0f*q.y*q.y - 2.0f*q.z*q.z) * v.x + (2.0f*(q.x*q.y + q.w*q.z))*v.y + 2.0f*(q.x*q.z - q.w*q.y)*v.z,
                   (2.0f*(q.x*q.y - q.w*q.z)) * v.x + (1.0f - 2.0f*q.x*q.x - 2.0f*q.z*q.z) * v.y + 2.0f*(q.y*q.z + q.w*q.x)*v.z,
                   (2.0f*(q.x*q.z + q.w*q.y)) * v.x + 2.0f*(q.y*q.z - q.w*q.x)*v.y + (1.0f - 2.0f*q.x*q.x - 2.0f*q.y*q.y)*v.z};
  return out;
}

quaternion_t quat_from_two_unit_vectors(vector_t u, vector_t v)
{
  // Adapted From the Ogre3d source code
  // https://bitbucket.org/sinbad/ogre/src/9db75e3ba05c/OgreMain/include/OgreVector3.h?fileviewer=file-view-default#cl-651
  quaternion_t out;
  float d = dot(u, v);
  if ( d >= 1.0f)
  {
    out = {1, 0, 0, 0};
  }
  else
  {
    float invs = turboInvSqrt(2*(1+d));
    vector_t xyz = scalar_multiply(invs, cross(u, v));
    out = {0.5f/invs, xyz.x, xyz.y, xyz.z};
  }
  return quaternion_normalize(out);
}

void euler_from_quat(quaternion_t q, float *phi, float *theta, float *psi)
{
  *phi = atan2_approx(2.0f * (q.w*q.x + q.y*q.z),
                      1.0f - 2.0f * (q.x*q.x + q.y*q.y));
  *theta = asin_approx(2.0f*(q.w*q.y - q.z*q.x));
  *psi = atan2_approx(2.0f * (q.w*q.z + q.x*q.y),
                      1.0f - 2.0f * (q.y*q.y + q.z*q.z));
}

union float_converter_t
{
  float fvalue;
  int32_t ivalue;
};

float turboInvSqrt(float x)
{
  volatile float x2;
  volatile float_converter_t y, i;
  const float threehalfs = 1.5F;

  x2 = x * 0.5F;
  y.fvalue  = x;
  i.ivalue  = y.ivalue;                             // evil floating point bit level hacking
  i.ivalue  = 0x5f3759df - (i.ivalue >> 1);
  y.fvalue = i.fvalue;
  y.fvalue  = y.fvalue * (threehalfs - (x2 * y.fvalue * y.fvalue));       // 1st iteration
  y.fvalue  = y.fvalue * (threehalfs - (x2 * y.fvalue * y.fvalue));       // 2nd iteration, this can be removed

  return fabs(y.fvalue);
}

float fsat(float value, float max)
{
  if (fabs(value) > fabs(max))
  {
    value = max*fsign(value);
  }
  return value;
}
