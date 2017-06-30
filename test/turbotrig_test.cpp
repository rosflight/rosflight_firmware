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
#include "turbovec.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "printf.h"
#include <gtest/gtest.h>

#define EXPECT_VEC3_SUPERCLOSE(vec, eig) EXPECT_LE(fabs(vec.x - eig.x()), 0.0001);\
                                         EXPECT_LE(fabs(vec.x - eig.x()), 0.0001);\
                                         EXPECT_LE(fabs(vec.x - eig.x()), 0.0001)

#define EXPECT_SUPERCLOSE(x, y) EXPECT_LE(fabs(x -y), 0.0001)

TEST(turbotrig_test, atan_test) {
    for (float i = -200.0; i <= 200.0; i += 0.001)
    {
        EXPECT_LE(fabs(turboatan(i) - atan(i)), 0.0001);
    }
}

TEST(turbotrig_test, atan2_test) {
    for (float i = -100.0; i <= 100.0; i += 0.1)
    {
        for (float j = -1.0; j <= 1.0; j += 0.001)
        {
            if (fabs(j) > 0.0001)
            {
                EXPECT_LE(fabs(atan2_approx(i, j) - atan2(i, j)), 0.001);
            }
        }
    }
}

TEST(turbotrig_test, asin_test) {
    for (float i = -1.0; i <= 1.0; i += 0.001)
    {
        if ( fabs(i) < 0.95 )
            EXPECT_LE(fabs(asin_approx(i) - asin(i)), 0.0001);
        else
            EXPECT_LE(fabs(asin_approx(i) - asin(i)), 0.05);
    }
}

TEST(turbotvec_test, vector_test) {
  Eigen::Vector3d eig1;
  eig1 << 1, 2, 3;
  vector_t vec1 = {1, 2, 3};

  EXPECT_VEC3_SUPERCLOSE(vec1, eig1);

  // Test Vector addition

  Eigen::Vector3d eig2;
  eig2 << 2, 3, 4;
  vector_t vec2 = {2, 3, 4};

  EXPECT_VEC3_SUPERCLOSE(vector_add(vec1, vec2), (eig1 + eig2));
  EXPECT_VEC3_SUPERCLOSE(vector_sub(vec1, vec2), (eig1 - eig2));
  EXPECT_VEC3_SUPERCLOSE(vector_normalize(vec1), eig1.normalized());
  EXPECT_VEC3_SUPERCLOSE(scalar_multiply(5.0, vec1), 5.0*eig1);

  EXPECT_SUPERCLOSE(norm(vec1), eig1.norm());
  EXPECT_SUPERCLOSE(sqrd_norm(vec1), eig1.squaredNorm());

  // Test Vector Dot Product
  EXPECT_SUPERCLOSE(dot(vec1, vec2), (eig1.transpose() * eig2));

  // Test Vector Cross Product
  EXPECT_VEC3_SUPERCLOSE(cross(vec1, vec2), eig1.cross(eig2));

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
