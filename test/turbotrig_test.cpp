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

#include "math.h"
#include "turbotrig.h"
#include "printf.h"
#include <gtest/gtest.h>

TEST(turbotrig_test, atan_test) {
    for (float i = -200.0; i <= 200.0; i += 0.001)
    {
//        printf("i: %f, approx: %f, actual: %f, diff: %f\n",
//               i, turboatan(i), atan(i), fabs(turboatan(i) - atan(i)));
        ASSERT_LE(fabs(turboatan(i) - atan(i)), 0.0001);
    }
}

TEST(turbotrig_test, atan2_test) {
    for (float i = -1.0; i <= 1.0; i += 0.1)
    {
        for (float j = -100.0; j <= 100.0; j += 0.0001)
        {
            if (fabs(j) > 0.0001)
            {
//                printf("atan2(i: %f, j: %f), approx: %f, actual: %f, diff: %f\n",
//                       i, j, atan2_approx(i, j), atan2(i, j), fabs(atan2_approx(i, j) - atan2(i, j)));
                ASSERT_LE(fabs(atan2_approx(i, j) - atan2(i, j)), 0.001);
            }
        }
    }
}

//TEST(turbotrig_test, asin_test) {
//    for (float i = -1.0; i <= 1.0; i += 0.1)
//    {
//        printf("i: %f, approx: %f, actual: %f, diff: %f\n",
//               i, asin_approx(i), asin(i), fabs(asin_approx(i) - asin(i)));
//        ASSERT_LE(fabs(asin_approx(i) - asin(i)), 0.01);
//    }
//}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
