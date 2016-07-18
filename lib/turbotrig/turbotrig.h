#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// float-based wrappers
float atan2_approx(float y, float x);
float asin_approx(float x);

// turbo-speed trig approximation
int32_t turboatan2(int32_t y, int32_t x);
int32_t turboatan(int32_t x);
int32_t turboasin(int32_t x);
int32_t turbocos(int32_t x);
int32_t turbosin(int32_t x);
int32_t sign(int32_t y);

#ifdef __cplusplus
}
#endif
