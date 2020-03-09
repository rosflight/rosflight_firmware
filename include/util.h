/*
 * Copyright (c) 2020, James Jackson, Daniel Koch, and Trey Henrichsen, BYU MAGICC Lab
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

#ifndef ROSFLIGHT_FIRMWARE_UTIL_H
#define ROSFLIGHT_FIRMWARE_UTIL_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rosflight_firmware
{

/**
 * @brief Fletcher 16-bit checksum
 *
 * @param src Pointer to data on which to compute the checksum
 * @param len Number of bytes in the data
 * @param finalize Whether to finalize the checksum; set to false for intermediate chunks of non-contiguous data
 * @param start Value at which to start the checksum, set for subsequent calls on chunks of non-contiguous data
 * @return uint16_t Fletcher 16-bit checksum
 */
inline uint16_t checksum_fletcher16(const uint8_t *src, size_t len, bool finalize = true, uint16_t start = 0)
{
  static constexpr size_t max_block_length = 5800; // guarantee that no overflow will occur (reduce from standard value to account for values in 'start')

  uint32_t c1 = (start & 0xFF00) >> 8;
  uint32_t c2 = start & 0x00FF;

  size_t block_length;
  for (; len > 0; len -= block_length)
  {
    block_length = len < max_block_length ? len : max_block_length;
    for (size_t i = 0; i < block_length; i++)
    {
      c1 += *(src++);
      c2 += c1;
    }

    c1 %= 255;
    c2 %= 255;
  }

  uint16_t checksum = c1 << 8 | c2;

  if (finalize && checksum == 0)
    checksum = 0xFFFF;

  return checksum;
}

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_UTIL_H
