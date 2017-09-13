/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson - BYU MAGICC Lab, Provo UT
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

#include <turbomath/turbomath.h>

namespace turbomath
{

Vector::Vector() : x(0.0f), y(0.0f), z(0.0f)
{}

Vector::Vector(float x_, float y_, float z_) : x(x_), y(y_), z(z_)
{}


float Vector::norm() const
{
  return 1.0f/inv_sqrt(x*x + y*y + z*z);
}


float Vector::sqrd_norm() const
{
  return x*x + y*y + z*z;
}


Vector& Vector::normalize()
{
  float recip_norm = inv_sqrt(x*x + y*y + z*z);
  x *= recip_norm;
  y *= recip_norm;
  z *= recip_norm;
  return *this;
}


Vector Vector::normalized() const
{
  float recip_norm = inv_sqrt(x*x + y*y + z*z);
  Vector out(x*recip_norm, y*recip_norm, z*recip_norm);
  return out;
}


Vector Vector::operator+(const Vector& v) const
{
  return Vector(x + v.x, y + v.y, z + v.z);
}


Vector Vector::operator-(const Vector& v) const
{
  return Vector(x - v.x, y - v.y, z - v.z);
}


Vector& Vector::operator +=(const Vector& v)
{
  x += v.x;
  y += v.y;
  z += v.z;
  return *this;
}


Vector& Vector::operator -=(const Vector& v)
{
  x -= v.x;
  y -= v.y;
  z -= v.z;
  return *this;
}


Vector Vector::operator *(float s) const
{
  return Vector(x*s, y*s, z*s);
}


Vector Vector::operator /(float s) const
{
  return Vector(x/s, y/s, z/s);
}


Vector& Vector::operator *=(float s)
{
  x *= s;
  y *= s;
  z *= s;
  return *this;
}


Vector& Vector::operator /=(float s)
{
  x /= s;
  y /= s;
  z /= s;
  return *this;
}


float Vector::dot(const Vector& v) const
{
  return x*v.x + y*v.y + z*v.z;
}


Vector Vector::cross(const Vector& v) const
{
  return Vector( y * v.z - z * v.y,
                 z * v.x - x * v.z,
                 x * v.y - y * v.x);
}

Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f)
{}

Quaternion::Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_)
{}

Quaternion::Quaternion(const Vector& u, const Vector& v)
{
  from_two_unit_vectors(u, v);
}

Quaternion::Quaternion(float roll, float pitch, float yaw)
{
  from_RPY(roll, pitch, yaw);
}

Quaternion& Quaternion::normalize()
{
  float recip_norm = inv_sqrt(w*w + x*x + y*y + z*z);
  w *= recip_norm;
  x *= recip_norm;
  y *= recip_norm;
  z *= recip_norm;

  // Make sure the quaternion is canonical (w is always positive)
  if (w < 0.0f)
  {
    w *= -1.0f;
    x *= -1.0f;
    y *= -1.0f;
    z *= -1.0f;
  }

  return *this;
}

Quaternion Quaternion::operator *(const Quaternion& q) const
{
  return Quaternion(w*q.w - x*q.x - y*q.y - z*q.z,
                    w*q.x + x*q.w - y*q.z + z*q.y,
                    w*q.y + x*q.z + y*q.w - z*q.x,
                    w*q.z - x*q.y + y*q.x + z*q.w);
}

Quaternion& Quaternion::operator *=(const Quaternion& q)
{
  w = w*q.w - x*q.x - y*q.y - z*q.z;
  x = w*q.x + x*q.w - y*q.z + z*q.y;
  y = w*q.y + x*q.z + y*q.w - z*q.x;
  z = w*q.z - x*q.y + y*q.x + z*q.w;
  return *this;
}

Vector Quaternion::rotate(const Vector& v) const
{
  return Vector((1.0f - 2.0f*y*y - 2.0f*z*z) * v.x + (2.0f*(x*y + w*z))*v.y + 2.0f*(x*z - w*y)*v.z,
                (2.0f*(x*y - w*z)) * v.x + (1.0f - 2.0f*x*x - 2.0f*z*z) * v.y + 2.0f*(y*z + w*x)*v.z,
                (2.0f*(x*z + w*y)) * v.x + 2.0f*(y*z - w*x)*v.y + (1.0f - 2.0f*x*x - 2.0f*y*y)*v.z);
}

Vector Quaternion::operator *(const Vector& v) const
{
  return rotate(v);
}

Quaternion Quaternion::inverse() const
{
  return Quaternion(w, -x, -y, -z);
}

Quaternion& Quaternion::invert()
{
  x *= -1.0f;
  y *= -1.0f;
  z *= -1.0f;
  return *this;
}

Quaternion& Quaternion::from_two_unit_vectors(const Vector& u, const Vector& v)
{
  // Adapted From the Ogre3d source code
  // https://bitbucket.org/sinbad/ogre/src/9db75e3ba05c/OgreMain/include/OgreVector3.h?fileviewer=file-view-default#cl-651
  float d = u.dot(v);
  if (d >= 1.0f)
  {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    return *this;
  }
  else
  {
    float invs = inv_sqrt(2.0f*(1.0f+d));
    Vector xyz = u.cross(v)*invs;
    w = 0.5f/invs;
    x = xyz.x;
    y = xyz.y;
    z = xyz.z;
  }
  normalize();
  return *this;
}

Quaternion& Quaternion::from_RPY(float roll, float pitch, float yaw)
{
  // p 259 of "Small unmanned aircraft: Theory and Practice" by Randy Beard and Tim McLain
  float cp = turbomath::cos(roll/2.0);
  float sp = turbomath::sin(roll/2.0);
  float ct = turbomath::cos(pitch/2.0);
  float st = turbomath::sin(pitch/2.0);
  float cs = turbomath::cos(yaw/2.0);
  float ss = turbomath::sin(yaw/2.0);

  w = cs*ct*cp + ss*st*sp;
  x = cs*ct*sp - ss*st*cp;
  y = cs*st*cp + ss*ct*sp;
  z = ss*ct*cp - cs*st*sp;

  normalize();
  return *this;
}

void Quaternion::get_RPY(float *roll, float *pitch, float *yaw) const
{
  *roll = turbomath::atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
  *pitch = turbomath::asin(2.0f*(w*y - z*x));
  *yaw = turbomath::atan2(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
}



#ifndef M_PI
#define M_PI 3.14159265359
#endif

static const float atan_max_x = 1.000000;
static const float atan_min_x = 0.000000;
static const float atan_scale_factor = 41720.240162;
static const int16_t atan_num_entries = 125;
static const int16_t atan_lookup_table[125] = {
0,	334,	667,	1001,	1335,	1668,	2001,	2334,	2666,	2999,
3331,	3662,	3993,	4323,	4653,	4983,	5311,	5639,	5967,	6293,
6619,	6944,	7268,	7592,	7914,	8235,	8556,	8875,	9194,	9511,
9827,	10142,	10456,	10768,	11080,	11390,	11699,	12006,	12313,	12617,
12921,	13223,	13524,	13823,	14120,	14417,	14711,	15005,	15296,	15586,
15875,	16162,	16447,	16731,	17013,	17293,	17572,	17849,	18125,	18399,
18671,	18941,	19210,	19477,	19742,	20006,	20268,	20528,	20786,	21043,
21298,	21551,	21802,	22052,	22300,	22546,	22791,	23034,	23275,	23514,
23752,	23988,	24222,	24454,	24685,	24914,	25142,	25367,	25591,	25814,
26034,	26253,	26471,	26686,	26900,	27113,	27324,	27533,	27740,	27946,
28150,	28353,	28554,	28754,	28952,	29148,	29343,	29537,	29728,	29919,
30108,	30295,	30481,	30665,	30848,	31030,	31210,	31388,	31566,	31741,
31916,	32089,	32260,	32431,	32599,	};

static const float asin_max_x = 1.000000;
static const float asin_min_x = 0.000000;
static const float asin_scale_factor = 20860.120081;
static const int16_t asin_num_entries = 200;
static const int16_t asin_lookup_table[200] = {
0,	104,	209,	313,	417,	522,	626,	730,	835,	939,
1043,	1148,	1252,	1357,	1461,	1566,	1671,	1775,	1880,	1985,
2090,	2194,	2299,	2404,	2509,	2614,	2720,	2825,	2930,	3035,
3141,	3246,	3352,	3458,	3564,	3669,	3775,	3881,	3988,	4094,
4200,	4307,	4413,	4520,	4627,	4734,	4841,	4948,	5056,	5163,
5271,	5379,	5487,	5595,	5703,	5811,	5920,	6029,	6138,	6247,
6356,	6465,	6575,	6685,	6795,	6905,	7015,	7126,	7237,	7348,
7459,	7570,	7682,	7794,	7906,	8019,	8131,	8244,	8357,	8471,
8584,	8698,	8812,	8927,	9042,	9157,	9272,	9388,	9504,	9620,
9737,	9854,	9971,	10089,	10207,	10325,	10444,	10563,	10682,	10802,
10922,	11043,	11164,	11285,	11407,	11530,	11652,	11776,	11899,	12024,
12148,	12273,	12399,	12525,	12652,	12779,	12907,	13035,	13164,	13293,
13424,	13554,	13686,	13817,	13950,	14083,	14217,	14352,	14487,	14623,
14760,	14898,	15036,	15176,	15316,	15457,	15598,	15741,	15885,	16029,
16175,	16321,	16469,	16618,	16767,	16918,	17070,	17224,	17378,	17534,
17691,	17849,	18009,	18170,	18333,	18497,	18663,	18830,	19000,	19171,
19343,	19518,	19695,	19874,	20055,	20239,	20424,	20613,	20803,	20997,
21194,	21393,	21596,	21802,	22012,	22225,	22443,	22664,	22891,	23122,
23359,	23601,	23849,	24104,	24366,	24637,	24916,	25204,	25504,	25816,
26143,	26485,	26847,	27232,	27644,	28093,	28588,	29149,	29814,	30680,
};


static const float max_pressure = 106598.405011;
static const float min_pressure = 69681.635473;
static const float pressure_scale_factor = 10.754785;
static const int16_t pressure_num_entries = 200;
static const int16_t pressure_lookup_table[200] = {
32767,	32544,	32321,	32098,	31876,	31655,	31434,	31213,	30993,	30773,
30554,	30335,	30117,	29899,	29682,	29465,	29248,	29032,	28816,	28601,
28386,	28172,	27958,	27745,	27532,	27319,	27107,	26895,	26684,	26473,
26263,	26053,	25843,	25634,	25425,	25217,	25009,	24801,	24594,	24387,
24181,	23975,	23769,	23564,	23359,	23155,	22951,	22748,	22544,	22341,
22139,	21937,	21735,	21534,	21333,	21133,	20932,	20733,	20533,	20334,
20135,	19937,	19739,	19542,	19344,	19148,	18951,	18755,	18559,	18364,
18169,	17974,	17780,	17586,	17392,	17199,	17006,	16813,	16621,	16429,
16237,	16046,	15855,	15664,	15474,	15284,	15095,	14905,	14716,	14528,
14339,	14151,	13964,	13777,	13590,	13403,	13217,	13031,	12845,	12659,
12474,	12290,	12105,	11921,	11737,	11554,	11370,	11188,	11005,	10823,
10641,	10459,	10278,	10096,	9916,	9735,	9555,	9375,	9195,	9016,
8837,	8658,	8480,	8302,	8124,	7946,	7769,	7592,	7415,	7239,
7063,	6887,	6711,	6536,	6361,	6186,	6012,	5837,	5664,	5490,
5316,	5143,	4970,	4798,	4626,	4454,	4282,	4110,	3939,	3768,
3597,	3427,	3257,	3087,	2917,	2748,	2578,	2409,	2241,	2072,
1904,	1736,	1569,	1401,	1234,	1067,	901,	734,	568,	402,
237,	71,	-94,	-259,	-424,	-588,	-752,	-916,	-1080,	-1243,
-1407,	-1570,	-1732,	-1895,	-2057,	-2219,	-2381,	-2543,	-2704,	-2865,
-3026,	-3187,	-3347,	-3507,	-3667,	-3827,	-3987,	-4146,	-4305,	-4464,
};

static const float sin_max_x = 3.141593;
static const float sin_min_x = 0.000000;
static const float sin_scale_factor = 32767.000000;
static const int16_t sin_num_entries = 125;
static const int16_t sin_lookup_table[125] = {
0,	823,	1646,	2468,	3289,	4107,	4922,	5735,	6544,	7349,
8149,	8944,	9733,	10516,	11293,	12062,	12824,	13578,	14323,	15059,
15786,	16502,	17208,	17904,	18588,	19260,	19920,	20568,	21202,	21823,
22431,	23024,	23602,	24166,	24715,	25247,	25764,	26265,	26749,	27216,
27666,	28099,	28513,	28910,	29289,	29648,	29990,	30312,	30615,	30899,
31163,	31408,	31633,	31837,	32022,	32187,	32331,	32454,	32558,	32640,
32702,	32744,	32764,	32764,	32744,	32702,	32640,	32558,	32454,	32331,
32187,	32022,	31837,	31633,	31408,	31163,	30899,	30615,	30312,	29990,
29648,	29289,	28910,	28513,	28099,	27666,	27216,	26749,	26265,	25764,
25247,	24715,	24166,	23602,	23024,	22431,	21823,	21202,	20568,	19920,
19260,	18588,	17904,	17208,	16502,	15786,	15059,	14323,	13578,	12824,
12062,	11293,	10516,	9733,	8944,	8149,	7349,	6544,	5735,	4922,
4107,	3289,	2468,	1646,	823
};

float fsign(float y)
{
  return (0.0f < y) - (y < 0.0f);
}

float cos(float x)
{
  return sin(M_PI/2.0 - x);
}

float sin(float x)
{
  // wrap down to +/x PI
  while (x > M_PI)
    x -= 2.0*M_PI;

  while (x <= -M_PI)
    x += 2.0*M_PI;

  // sin is symmetric
  if (x < 0)
    return -1.0*sin(-x);

  // wrap onto (0, PI)
  if (x > M_PI)
    return -1.0*sin(x - M_PI);

  // Now, all we have left is the range 0 to PI, use the lookup table
  float t = (x - sin_min_x)/(sin_max_x - sin_min_x) * static_cast<float>(sin_num_entries);
  int16_t index = static_cast<int16_t>(t);
  float delta_x = t - index;

  if (index >= sin_num_entries)
      return sin_lookup_table[sin_num_entries - 1]/sin_scale_factor;
  else if (index < sin_num_entries - 1)
      return sin_lookup_table[index]/sin_scale_factor + delta_x * (sin_lookup_table[index + 1] - sin_lookup_table[index])/sin_scale_factor;
  else
      return sin_lookup_table[index]/sin_scale_factor + delta_x * (sin_lookup_table[index] - sin_lookup_table[index - 1])/sin_scale_factor;
}


float atan(float x)
{
  // atan is symmetric
  if (x < 0)
  {
    return -1.0*atan(-1.0*x);
  }
  // This uses a sweet identity to wrap the domain of atan onto (0,1)
  if (x > 1.0)
  {
    return M_PI/2.0 - atan(1.0/x);
  }

  float t = (x - atan_min_x)/(atan_max_x - atan_min_x) * static_cast<float>(atan_num_entries);
  int16_t index = static_cast<int16_t>(t);
  float delta_x = t - index;

  if (index >= atan_num_entries)
      return atan_lookup_table[atan_num_entries-1]/atan_scale_factor;
  else if (index < atan_num_entries - 1)
      return atan_lookup_table[index]/atan_scale_factor + delta_x * (atan_lookup_table[index + 1] - atan_lookup_table[index])/atan_scale_factor;
  else
      return atan_lookup_table[index]/atan_scale_factor + delta_x * (atan_lookup_table[index] - atan_lookup_table[index - 1])/atan_scale_factor;
}


float atan2(float y, float x)
{
  // algorithm from wikipedia: https://en.wikipedia.org/wiki/Atan2
  if (x == 0.0)
  {
    if (y < 0.0)
    {
      return - M_PI/2.0;
    }
    else if ( y > 0.0)
    {
      return M_PI/2.0;
    }
    else
    {
      return 0.0;
    }
  }

  float arctan = atan(y/x);

  if (x < 0.0)
  {
    if ( y < 0)
    {
      return arctan - M_PI;
    }
    else
    {
      return arctan + M_PI;
    }
  }

  else
  {
      return arctan;
  }
}


float asin(float x)
{
  if (x < 0.0)
  {
    return -1.0*asin(-1.0*x);
  }

  float t = (x - asin_min_x)/(asin_max_x - asin_min_x) * static_cast<float>(asin_num_entries);
  int16_t index = static_cast<int16_t>(t);
  float delta_x = t - index;

  if (index >= asin_num_entries)
      return asin_lookup_table[asin_num_entries - 1]/asin_scale_factor;
  else if (index < asin_num_entries - 1)
      return asin_lookup_table[index]/asin_scale_factor + delta_x * (asin_lookup_table[index + 1] - asin_lookup_table[index])/asin_scale_factor;
  else
      return asin_lookup_table[index]/asin_scale_factor + delta_x * (asin_lookup_table[index] - asin_lookup_table[index - 1])/asin_scale_factor;
}

float alt(float press)
{

  if(press < max_pressure && press > min_pressure)
  {
    float t = (press - min_pressure)/(max_pressure - min_pressure) * static_cast<float>(pressure_num_entries);
    int16_t index = static_cast<int16_t>(t);
    float delta_x = t - index;

    if (index >= pressure_num_entries)
        return asin_lookup_table[pressure_num_entries - 1]/pressure_scale_factor;
    else if (index < pressure_num_entries - 1)
        return pressure_lookup_table[index]/pressure_scale_factor + delta_x * (pressure_lookup_table[index + 1] - pressure_lookup_table[index])/pressure_scale_factor;
    else
        return pressure_lookup_table[index]/pressure_scale_factor + delta_x * (pressure_lookup_table[index] - pressure_lookup_table[index - 1])/pressure_scale_factor;
  }
  else
    return 0.0;
}

float fabs(float x)
{
  if (x < 0)
    return -x;
  else
    return x;

}

float inv_sqrt(float x)
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

} // namespace turbomath
