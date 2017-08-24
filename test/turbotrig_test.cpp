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
#include <stdio.h>

vector_t random_vectors[25] = {
  { -0.0376278050814 , 0.471775699711 , -0.336572370974 },
  { 0.842139998851 , -0.113277302409 , -0.435361598132 },
  { 0.402876930871 , -0.998517068538 , 0.956603957591 },
  { 0.366004030077 , -0.966554559399 , 0.236455814495 },
  { 0.170963581611 , -0.892193316086 , -0.360102936987 },
  { -0.675191763273 , -0.794118513048 , 0.561367212903 },
  { -0.0299477253533 , 0.0938177650483 , 0.525814272724 },
  { -0.676191678521 , -0.0780862208203 , -0.272955681219 },
  { -0.435749833209 , -0.673810649938 , -0.896559097382 },
  { 0.709083915552 , -0.135067363969 , -0.385492450532 },
  { -0.38728558039 , -0.502219301225 , 0.323557018529 },
  { -0.186870345154 , 0.554827454101 , 0.921567682061 },
  { -0.142106787605 , -0.764876359963 , 0.00303689980819 },
  { -0.677798963582 , -0.664595954482 , 0.339274533414 },
  { -0.700464041114 , 0.325731535871 , -0.621492014391 },
  { -0.604865828708 , 0.270639620454 , 0.188624833185 },
  { 0.464205180183 , -0.461504601245 , -0.578708441515 },
  { 0.498899172115 , -0.582342366402 , -0.694758083436 },
  { 0.0710544604541 , -0.63603887083 , -0.521799692437 },
  { -0.372025413205 , 0.83531212357 , 0.232484576742 },
  { 0.790872496361 , -0.89600683592 , 0.783984438621 },
  { 0.236462609786 , -0.636362560394 , 0.203951290805 },
  { 0.831924307534 , -0.482532468579 , 0.0600026189612 },
  { 0.0562194856302 , -0.605799189029 , -0.556494338297 },
  { -0.85014432598 , 0.0632157037573 , 0.0272188414114 }};


quaternion_t random_quaternions[25] = {
  { 0.10377420365 , -0.583993115868 , -0.731526280531 , -0.0530049846003 },
  { -0.00228103177408 , -0.506936771567 , 0.976002181169 , 0.90368722061 },
  { -0.280191704748 , 0.141235897077 , 0.770363502952 , 0.306427689307 },
  { 0.964538929753 , -0.849755381903 , 0.36374459234 , 0.694507794584 },
  { 0.0176390041681 , -0.960155080148 , 0.340078582124 , -0.119639355159 },
  { -0.213139865459 , -0.91618752978 , -0.192746623826 , -0.761937711418 },
  { -0.491440057128 , -0.468120646081 , -0.0682240789086 , -0.779728041272 },
  { 0.00414757516987 , -0.980357614738 , 0.243315557667 , 0.487816606638 },
  { -0.593742280674 , 0.245648066311 , 0.682367014935 , -0.0659175648814 },
  { -0.322464011587 , 0.706588950729 , -0.966024250287 , -0.50354344519 },
  { -0.537023302971 , -0.496355850419 , -0.326843736039 , 0.456606813517 },
  { -0.581585485434 , 0.225766708322 , -0.121402082687 , 0.160333514827 },
  { -0.422711480811 , 0.894994456476 , 0.392582496229 , 0.0035135659771 },
  { 0.326380783544 , 0.551413227108 , 0.89489801397 , 0.87883243747 },
  { 0.83500683695 , -0.263875030319 , -0.1783391105 , 0.453727091163 },
  { -0.30389938019 , -0.0744317276089 , -0.436917072268 , 0.907173926266 },
  { -0.320066655494 , -0.349065285706 , 0.0336903431161 , 0.573906603454 },
  { -0.103624452083 , -0.82874783662 , -0.635967208274 , 0.562138574765 },
  { 0.90735669209 , -0.611711092446 , 0.732474120503 , 0.866697480004 },
  { 0.626137839218 , 0.41320663394 , -0.821473642241 , -0.344696411875 },
  { 0.266650461152 , -0.784707647527 , 0.324347257562 , -0.724904312141 },
  { 0.964177603528 , -0.378173605577 , 0.767349174766 , 0.560290218637 },
  { 0.0812716046369 , 0.745067180353 , -0.476875959113 , -0.245887902551 },
  { -0.177027678376 , 0.214558558928 , -0.992910369554 , 0.592964390132 },
  { 0.0979109306209 , 0.121890109199 , 0.126418158551 , 0.242200145606 }};


#define EXPECT_VEC3_SUPERCLOSE(vec, eig) \
  EXPECT_NEAR(vec.x, eig.x(), 0.0001);\
  EXPECT_NEAR(vec.y, eig.y(), 0.0001);\
  EXPECT_NEAR(vec.z, eig.z(), 0.0001)
#define EXPECT_QUAT_SUPERCLOSE(q, q_eig) { \
  double e1 = quaternion_error(q_eig, q); \
  q_eig.coeffs() *= -1.0; \
  double e2 = quaternion_error(q_eig, q); \
  double error = (e1 < e2) ? e1 : e2; \
  EXPECT_LE(error, 0.0001); \
}
#define ASSERT_QUAT_SUPERCLOSE(q, q_eig) { \
  double e1 = quaternion_error(q_eig, q); \
  q_eig.coeffs() *= -1.0; \
  double e2 = quaternion_error(q_eig, q); \
  double error = (e1 < e2) ? e1 : e2; \
  ASSERT_LE(error, 0.0001); \
}

#define ASSERT_TURBOQUAT_SUPERCLOSE(q, q_eig) { \
  double e1 = quaternion_error(q_eig, q); \
  q_eig.w *= -1.0; \
  q_eig.x *= -1.0; \
  q_eig.y *= -1.0; \
  q_eig.z *= -1.0; \
  double e2 = quaternion_error(q_eig, q); \
  double error = (e1 < e2) ? e1 : e2; \
  ASSERT_LE(error, 0.0001); \
}

#define EXPECT_SUPERCLOSE(x, y) EXPECT_NEAR(x, y, 0.0001)
#define ASSERT_SUPERCLOSE(x, y) ASSERT_NEAR(x, y, 0.0001)
#define EXPECT_CLOSE(x, y) ASSERT_NEAR(x, y, 0.01)

double quaternion_error(Eigen::Quaternionf q_eig, quaternion_t q)
{
  Eigen::Quaternionf est_quat(q.w, q.x, q.y, q.z);
  Eigen::Quaternionf q_tilde = q_eig * est_quat.inverse();
  if(q_tilde.vec().norm() < 0.000001)
    return 0;
  else
  {
    Eigen::Vector3f v_tilde = atan2(q_tilde.vec().norm(), q_tilde.w())*q_tilde.vec()/q_tilde.vec().norm();
    return v_tilde.norm();
  }
}

double quaternion_error(quaternion_t q0, quaternion_t q)
{
  Eigen::Quaternionf est_quat(q.w, q.x, q.y, q.z);
  Eigen::Quaternionf q_eig(q0.w, q0.x, q0.y, q0.z);
  Eigen::Quaternionf q_tilde = q_eig * est_quat.inverse();
  if(q_tilde.vec().norm() < 0.000001)
    return 0;
  else
  {
    Eigen::Vector3f v_tilde = atan2(q_tilde.vec().norm(), q_tilde.w())*q_tilde.vec()/q_tilde.vec().norm();
    return v_tilde.norm();
  }
}

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
  for (int i = 0; i < 24; i++)
  {
    vector_t vec1 = random_vectors[i];
    vector_t vec2 = random_vectors[i + 1];
    Eigen::Vector3f eig2, eig1;
    eig1 << vec1.x, vec1.y, vec1.z;
    eig2 << vec2.x, vec2.y, vec2.z;

    // Test data type
    EXPECT_VEC3_SUPERCLOSE(vec1, eig1);
    EXPECT_VEC3_SUPERCLOSE(vec2, eig2);

    // Test norming operations
    EXPECT_VEC3_SUPERCLOSE(vector_normalize(vec1), eig1.normalized());
    EXPECT_SUPERCLOSE(norm(vec1), eig1.norm());
    EXPECT_SUPERCLOSE(sqrd_norm(vec1), eig1.squaredNorm());

    // Test add, subtract and multiply
    EXPECT_VEC3_SUPERCLOSE(vector_add(vec1, vec2), (eig1 + eig2));
    EXPECT_VEC3_SUPERCLOSE(vector_sub(vec1, vec2), (eig1 - eig2));
    EXPECT_VEC3_SUPERCLOSE(scalar_multiply(5.0, vec1), 5.0*eig1);

    // Test Vector Dot Product
    EXPECT_SUPERCLOSE(dot(vec1, vec2), (eig1.transpose() * eig2));

    // Test Vector Cross Product
    EXPECT_VEC3_SUPERCLOSE(cross(vec1, vec2), eig1.cross(eig2));
  }
}


TEST(turbovec_test, quaternion_test) {

  for (int i = 0; i < 24; i++)
  {
    quaternion_t quat1 = quaternion_normalize(random_quaternions[i]);
    quaternion_t quat2 = quaternion_normalize(random_quaternions[i + 1]);
    Eigen::Quaternionf eig1(quat1.w, quat1.x, quat1.y, quat1.z);
    Eigen::Quaternionf eig2(quat2.w, quat2.x, quat2.y, quat2.z);

    EXPECT_QUAT_SUPERCLOSE(quat1, eig1);
    EXPECT_QUAT_SUPERCLOSE(quat2, eig2);

    // Check normalization
    EXPECT_SUPERCLOSE(quat1.x*quat1.x + quat1.y*quat1.y + quat1.z*quat1.z + quat1.w*quat1.w, 1.0f);
    EXPECT_SUPERCLOSE(quat2.x*quat2.x + quat2.y*quat2.y + quat2.z*quat2.z + quat2.w*quat2.w, 1.0f);

    // Test Quaternion Operators
    ASSERT_QUAT_SUPERCLOSE(quaternion_multiply(quat1, quat2), (eig2*eig1));
    ASSERT_QUAT_SUPERCLOSE(quaternion_inverse(quat2), eig2.inverse());


    // Test Quaternion Rotate
    vector_t vec1 = random_vectors[i];
    Eigen::Vector3f veig1;
    veig1 << vec1.x, vec1.y, vec1.z;


    // Test rotate_vector
    vector_t vec2 = rotate_vector(quat1, vec1);
    Eigen::Vector3f veig2 = veig1.transpose()*eig1.toRotationMatrix();
    EXPECT_VEC3_SUPERCLOSE(vec2, veig2);

    vector_t vec3 = rotate_vector(quat1, vec1);
    Eigen::Vector3f veig3 = veig1.transpose()*eig1.toRotationMatrix();
    EXPECT_VEC3_SUPERCLOSE(vec3, veig3);
  }
}

TEST(turbovec_test, quat_from_two_vectors_test){
  // Test the "quat_from_two_vectors"
  vector_t vec1 = {1, 0, 0};
  quaternion_t quat1 = {1.0/sqrt(2.0), 0, 0, 1.0/sqrt(2.0)};
  vector_t vec2 = rotate_vector(quat1, vec1);
  quaternion_t quat_test = quat_from_two_unit_vectors(vec2, vec1);

  ASSERT_TURBOQUAT_SUPERCLOSE(quat1, quat_test);


  // A bunch of random (off-axes tests)
  for (int i = 0; i < 25; i++)
  {
    vector_t vec3 = vector_normalize(random_vectors[i]);
    quaternion_t quat2 = quaternion_normalize(random_quaternions[i]);

    // Manually rotate this vector with the quaternion
    vector_t vec4 = rotate_vector(quat2, vec3);

    // Extract the "shortest-arc" rotation
    quat_test = quat_from_two_unit_vectors(vec4, vec3);

    // Try rotating that same vector with this new quaternion
    vector_t vec5 = rotate_vector(quat_test, vec3);

    // Make sure that the vectors are the same
    ASSERT_SUPERCLOSE(vec5.x, vec4.x);
    ASSERT_SUPERCLOSE(vec5.y, vec4.y);
    ASSERT_SUPERCLOSE(vec5.z, vec4.z);
  }
}

TEST(turbotrig_test, fast_alt_test) {

  //out of bounds
  EXPECT_EQ(fast_alt(69681), 0.0);
  EXPECT_EQ(fast_alt(106598), 0.0);

  //all valid int values
  float trueResult = 0.0;
  for (int i = 69682; i < 106597; i++) {
    trueResult = static_cast<float>((1.0 - pow(static_cast<float>(i)/101325, 0.190284)) * 145366.45) * static_cast<float>(0.3048);
    EXPECT_LE(fabs(fast_alt(i) - trueResult), .15);
    //arbitrarily chose <= .15m since fast_alt isn't accurate enough for EXPECT_CLOSE,
    //but being within .15 meters of the correct result seems pretty good to me
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
