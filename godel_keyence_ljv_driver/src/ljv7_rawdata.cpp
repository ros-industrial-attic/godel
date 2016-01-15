/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ljv7_rawdata.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

int ljv7_unpack_profile_data(unsigned char* src, uint32_t src_sz, uint32_t num_pp,
                             profile_point_t* dst, uint32_t dst_sz)
{
  // constants from protocol documentation
  const uint32_t bpp = 20; // bits per profile point
  const uint32_t ppr = 8;  // profiles per 'row'

  const uint32_t bpb = sizeof(char) * CHAR_BIT; // bits per byte
  const uint32_t bpr = (bpp * ppr) / bpb;       // bytes per 'row'
  const uint32_t ipr = bpr / sizeof(int32_t);   // ints per 'row'
  unsigned int i = 0, j = 0;

  // make sure source buffer is multiple of 'bytes per row' bytes
  if (src_sz % bpr != 0)
  {
    return -1;
  }

  // make sure destination buffer is large enough to hold unpacked data
  if (dst_sz != (num_pp * sizeof(profile_point_t)))
  {
    return -2;
  }

  const unsigned char* src_end = src + ((num_pp * bpp) / 8);
  if (src_end > (src + src_sz))
  {
    // TODO: define our own internal error codes
    return -3;
  }

  uint32_t* ints_ptr = (uint32_t*)src;
  // from: https://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
  const int32_t m = 1U << (bpp - 1);
  for (i = 0, j = 0; j < num_pp; i += ipr, j += ppr)
  {
    // unpack 'ppr' points at a time (which needs 'ipr' int32's)
    dst[j] = ((ints_ptr[i] & 0x000FFFFF));
    dst[j + 1] = ((ints_ptr[i] & 0xFFF00000) >> 20) | ((ints_ptr[i + 1] & 0x000000FF) << 12);
    dst[j + 2] = ((ints_ptr[i + 1] & 0x0FFFFF00) >> 8);
    dst[j + 3] = ((ints_ptr[i + 1] & 0xF0000000) >> 28) | ((ints_ptr[i + 2] & 0x0000FFFF) << 4);

    dst[j + 4] = ((ints_ptr[i + 2] & 0xFFFF0000) >> 16) | ((ints_ptr[i + 3] & 0x0000000F) << 16);
    dst[j + 5] = ((ints_ptr[i + 3] & 0x00FFFFF0) >> 4);
    dst[j + 6] = ((ints_ptr[i + 3] & 0xFF000000) >> 24) | ((ints_ptr[i + 4] & 0x00000FFF) << 8);
    dst[j + 7] = ((ints_ptr[i + 4] & 0xFFFFF000) >> 12);

    // todo: rework this
    dst[j] = (dst[j] ^ m) - m;
    dst[j + 1] = (dst[j + 1] ^ m) - m;
    dst[j + 2] = (dst[j + 2] ^ m) - m;
    dst[j + 3] = (dst[j + 3] ^ m) - m;
    dst[j + 4] = (dst[j + 4] ^ m) - m;
    dst[j + 5] = (dst[j + 5] ^ m) - m;
    dst[j + 6] = (dst[j + 6] ^ m) - m;
    dst[j + 7] = (dst[j + 7] ^ m) - m;
  }

  return 0;
}
