#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
  float x;
  float y;
  float z;
} vector_t;

typedef struct
{
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} intvec_t;

typedef struct
{
  int32_t w;
  int32_t x;
  int32_t y;
  int32_t z;
} intquat_t;

int32_t int_dot(intvec_t v1, intvec_t v2);
intvec_t int_cross(intvec_t u, intvec_t v);
intvec_t int_scalar_multiply(int32_t s, intvec_t v);
intvec_t int_vector_add(intvec_t u, intvec_t v);
intvec_t int_vector_sub(intvec_t u, intvec_t v);
int32_t int_sqrd_norm(intvec_t v);
intvec_t int_vector_normalize(intvec_t v);
intquat_t int_quaternion_normalize(intquat_t q);
intquat_t int_quaternion_multiply(intquat_t q1, intquat_t q2);
intquat_t int_quaternion_inverse(intquat_t q);
intquat_t int_quaternion_from_two_vectors(intvec_t u, intvec_t v);

float dot(vector_t v1, vector_t v2);
vector_t cross(vector_t u, vector_t v);
vector_t scalar_multiply(float s, vector_t v);
vector_t vector_add(vector_t u, vector_t v);
vector_t vector_sub(vector_t u, vector_t v);
float sqrd_norm(vector_t v);
vector_t vector_normalize(vector_t v);
quaternion_t quaternion_normalize(quaternion_t q);
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_inverse(quaternion_t q);
quaternion_t quat_from_two_vectors(vector_t u, vector_t v);

void euler_from_quat(quaternion_t q, float *phi, float *theta, float *psi);
void euler_from_int_quat(intquat_t q, int32_t phi, int32_t theta, int32_t psi);

float turboInvSqrt(float x);
#ifdef __cplusplus
}
#endif
