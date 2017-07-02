#include <gtest/gtest.h>
#include "math.h"
#include "rosflight.h"
#include "test_board.h"
#include "turbotrig.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/unsupported/Eigen/MatrixFunctions"


#define EXPECT_CLOSE(x, y) EXPECT_LE(fabs(x - y), 0.01);
#define EXPECT_FABSCLOSE(x, y) EXPECT_LE(fabs(x) - fabs(y), 0.01)


using namespace rosflight_firmware;

double sign(double y)
{
  return (0.0 < y) - (y < 0.0);
}

void step_dynamics(Eigen::Matrix3d& rotation, Eigen::Vector3d omega, double dt)
{
  Eigen::Matrix3d omega_skew;
  omega_skew << 0.0, -omega(2), omega(1),
      omega(2), 0.0, -omega(0),
      -omega(1), omega(0), 0.0;
  rotation = rotation*(omega_skew*dt).exp();
}


TEST(estimator_test, truth_simulation) {
  testBoard board;
  ROSflight rf(board);

  // Initialize the firmware
  rf.rosflight_init();

  double x_freq = 1.0;
  double y_freq = 2.0;
  double z_freq = 3.0;
  double x_amp = 1.0;
  double y_amp = 0.5;
  double z_amp = 0.2;

  double dt = 0.001;

  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, -9.80665;

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  for (double t = 0.0; t < 30.0; t += dt)
  {
    Eigen::Vector3d omega;
    omega << x_amp*sin(x_freq/(2.0*M_PI)*t),
        y_amp*sin(y_freq/(2.0*M_PI)*t),
        z_amp*sin(z_freq/(2.0*M_PI)*t);

    step_dynamics(rotation, omega, dt);


    // Extract Accel Orientation
    Eigen::Vector3d y_acc = rotation.transpose() * gravity;

    float acc[3] = {(float)y_acc(0), (float)y_acc(1), (float)y_acc(2)};
    float gyro[3] = {(float)omega(0), (float)omega(1), (float)omega(2)};

    // Simulate measurements
    board.set_imu(acc, gyro, (uint64_t)(t*1e6));

    // Run firmware
    rf.rosflight_run();

    Eigen::Quaterniond eig_quat(rotation);

    quaternion_t estimate = rf.estimator_.state().attitude;
    Eigen::Quaterniond est_quat(estimate.w, estimate.x, estimate.y, estimate.z);
    Eigen::Quaterniond q_tilde = eig_quat * est_quat.inverse();
    if (q_tilde.w() < 0)
      q_tilde.coeffs() *= -1.0;

    Eigen::Vector3d v_tilde = atan2(q_tilde.vec().norm(), q_tilde.w())*q_tilde.vec()/q_tilde.vec().norm();
    if (std::isfinite(v_tilde.norm()))
      EXPECT_LE(v_tilde.norm(), 0.01);
  }
}


TEST(estimator_test, bias_simulation) {
  testBoard board;
  ROSflight rf(board);

  // Initialize the firmware
  rf.rosflight_init();

  double x_freq = 1.0;
  double y_freq = 1.0;
  double z_freq = 3.0;
  double x_amp = 1.0;
  double y_amp = 0.5;
  double z_amp = 0.2;

  double x_bias = 0.05;
  double y_bias = 0.05;

  Eigen::Vector3d bias;
  bias << x_bias, y_bias, 0.0;

  double dt = 0.001;

  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, -9.80665;

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  for (double t = 0.0; t < 30.0; t += dt)
  {
    Eigen::Vector3d omega;
    omega << x_amp*sin(x_freq/(2.0*M_PI)*t),
        y_amp*sin(y_freq/(2.0*M_PI)*t),
        z_amp*sin(z_freq/(2.0*M_PI)*t);

    step_dynamics(rotation, omega, dt);


    // Extract Accel Orientation
    Eigen::Vector3d y_acc = rotation.transpose() * gravity;
    Eigen::Vector3d y_gyro = omega + bias;

    float acc[3] = {(float)y_acc(0), (float)y_acc(1), (float)y_acc(2)};
    float gyro[3] = {(float)y_gyro(0), (float)y_gyro(1), (float)y_gyro(2)};

    // Simulate measurements
    board.set_imu(acc, gyro, (uint64_t)(t*1e6));

    // Run firmware
    rf.rosflight_run();

    // See if bias estimate has converged
    if (t > 20.0)
    {
      EXPECT_CLOSE(rf.estimator_.state().angular_velocity.x, omega.x());
      EXPECT_CLOSE(rf.estimator_.state().angular_velocity.y, omega.y());
    }
  }
}
