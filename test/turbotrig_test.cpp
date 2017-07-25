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
#include <turbotrig/turbotrig.h>
#include <turbotrig/turbovec.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <gtest/gtest.h>

#define EXPECT_VEC3_SUPERCLOSE(vec, eig) EXPECT_LE(fabs(vec.x - eig.x()), 0.0001);\
  EXPECT_LE(fabs(vec.y - eig.y()), 0.0001);\
  EXPECT_LE(fabs(vec.z - eig.z()), 0.0001)
#define EXPECT_QUAT_SUPERCLOSE(vec, eig) EXPECT_LE(fabs(vec.w - eig.w()), 0.0001);\
  EXPECT_LE(fabs(vec.x - eig.x()), 0.0001);\
  EXPECT_LE(fabs(vec.y - eig.y()), 0.0001);\
  EXPECT_LE(fabs(vec.z - eig.z()), 0.0001)

#define EXPECT_SUPERCLOSE(x, y) EXPECT_LE(fabs(x -y), 0.0001)
#define EXPECT_CLOSE(x, y) EXPECT_LE(fabs(x -y), 0.01)

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

TEST(turbovec_test, vector_test) {
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


TEST(turbovec_test, quaternion_test) {
  Eigen::Quaternionf eig1(1.0/sqrt(2.0), 0, 0, 1.0/sqrt(2.0));
  eig1.normalize();
  quaternion_t quat1 = {eig1.w(), eig1.x(), eig1.y(), eig1.z()};

  EXPECT_QUAT_SUPERCLOSE(quat1, eig1);

  quaternion_t quat2 = {-5, 10, 2, -15};
  Eigen::Quaternionf eig2(-5, 10, 2, -15);
  eig2.normalize();
  quat2 = quaternion_normalize(quat2);

  EXPECT_QUAT_SUPERCLOSE(quat2, eig2);

  // Test Quaternion Operators
  EXPECT_QUAT_SUPERCLOSE(quaternion_multiply(quat1, quat2), (eig2*eig1));
  EXPECT_QUAT_SUPERCLOSE(quaternion_inverse(quat2), eig2.inverse());


  // Test Quaternion Rotate
  vector_t vec1 = {1, 2, 3};
  Eigen::Vector3f veig1;
  veig1 << 1, 2, 3;

  // Test rotate_vector
  vector_t vec2 = rotate_vector(quat1, vec1);
  Eigen::Vector3f veig2 = veig1.transpose()*eig1.toRotationMatrix();
  EXPECT_VEC3_SUPERCLOSE(vec2, veig2);

  vector_t vec3 = rotate_vector(quat1, vec1);
  Eigen::Vector3f veig3 = veig1.transpose()*eig1.toRotationMatrix();
  EXPECT_VEC3_SUPERCLOSE(vec3, veig3);
}

TEST(turbovec_test, quat_from_two_vectors_test){
  // Test the "quat_from_two_vectors"
  vector_t vec1 = {1, 0, 0};
  quaternion_t quat1 = {1.0/sqrt(2.0), 0, 0, 1.0/sqrt(2.0)};
  vector_t vec2 = rotate_vector(quat1, vec1);
  quaternion_t quat_test = quat_from_two_unit_vectors(vec2, vec1);

  EXPECT_CLOSE(quat1.w, quat_test.w);
  EXPECT_CLOSE(quat1.x, quat_test.x);
  EXPECT_CLOSE(quat1.y, quat_test.y);
  EXPECT_CLOSE(quat1.z, quat_test.z);


  // A more complicated (off-axis) test
  vector_t vec3 = {-10, 4, -8};
  vec3 = vector_normalize(vec3);
  quaternion_t quat2 = {5, 1, 3, 0.2};
  quat2 = quaternion_normalize(quat2);
  vector_t vec4 = rotate_vector(quat2, vec3);

  quat_test = quat_from_two_unit_vectors(vec4, vec3);

  EXPECT_CLOSE(quat2.w, quat_test.w);
  EXPECT_CLOSE(quat2.x, quat_test.x);
  EXPECT_CLOSE(quat2.y, quat_test.y);
  EXPECT_CLOSE(quat2.z, quat_test.z);
}

TEST(turbotrig_test, fast_alt_test) {

  //out of bounds
  EXPECT_EQ(fast_alt(69681), 0.0);
  EXPECT_EQ(fast_alt(106598), 0.0);

  //all valid int values
  float trueResult = 0.0;
  for (int i = 69682; i < 106597; i++) {
    trueResult = (float)((1.0 - pow((float)i/101325, 0.190284)) * 145366.45) * (float)0.3048;
    EXPECT_LE(fabs(fast_alt(i) - trueResult), .15);
    //arbitrarily chose <= .15m since fast_alt isn't accurate enough for EXPECT_CLOSE,
    //but being within .15 meters of the correct result seems pretty good to me
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
