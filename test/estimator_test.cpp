#include "common.h"
#include "math.h"
#include "rosflight.h"
#include "mavlink.h"
#include "test_board.h"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include <cmath>
#include <fstream>

//#define DEBUG

using namespace rosflight_firmware;

double sign(double y)
{
  return (y > 0.0) - (y < 0.0);
}

double run_estimator_test(std::string filename, ROSflight& rf, testBoard& board, std::vector<double> params)
{
#ifndef DEBUG
  (void) filename;
#endif

  double x_freq = params[0];
  double y_freq = params[1];
  double z_freq = params[2];
  double x_amp = params[3];
  double y_amp = params[4];
  double z_amp = params[5];
  double tmax = params[6];

  double dt = 0.001;

  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, -9.80665;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

#ifdef DEBUG
  // File for saving simulation results
  std::ofstream file;
  file.open(filename.c_str());
#endif

  double max_error = 0.0;
  volatile double t = 0.0;
  while(t < tmax)
  {
    // euler integration of S03 (probably a better way that isn't so intensive)
    double ddt = 0.00005;
    Eigen::Matrix3d omega_skew;
    double step = t + dt;
    while (t < step)
    {
      double p = x_amp*sin(x_freq/(2.0*M_PI)*t);
      double q = y_amp*sin(y_freq/(2.0*M_PI)*t);
      double r = z_amp*sin(z_freq/(2.0*M_PI)*t);

      omega_skew << 0.0, -r, q,
                    r, 0.0, -p,
                    -q, p, 0.0;
      rotation = rotation*(omega_skew*ddt).exp();
      t += ddt;
    }

    // Extract Accel Orientation
    Eigen::Vector3d y_acc = rotation.transpose() * gravity;

    // Create gyro measurement
    double p = x_amp*sin(x_freq/(2.0*M_PI)*t);
    double q = y_amp*sin(y_freq/(2.0*M_PI)*t);
    double r = z_amp*sin(z_freq/(2.0*M_PI)*t);

    float acc[3] = {static_cast<float>(y_acc(0)), static_cast<float>(y_acc(1)), static_cast<float>(y_acc(2))};
    float gyro[3] = {static_cast<float>(p), static_cast<float>(q), static_cast<float>(r)};

    // Simulate measurements
    board.set_imu(acc, gyro, static_cast<uint64_t>(t*1e6));

    // Run firmware
    rf.run();

    Eigen::Quaternionf eig_quat(rotation.cast<float>());
    turbomath::Quaternion estimate = rf.estimator_.state().attitude;
    if (eig_quat.w() < 0.0)
    {
      eig_quat.coeffs() *= -1.0;
    }

    double pos_error = quaternion_error(eig_quat, estimate);
    eig_quat.coeffs() *= -1.0;
    double neg_error = quaternion_error(eig_quat, estimate);

    double error = 0.0;
    if (pos_error < neg_error)
    {
      error = pos_error;
      eig_quat.coeffs() *= -1.0;
    }
    else
    {
      error = neg_error;
    }

    // output to file for plotting
#ifdef DEBUG
    file << t << ", " << (error > error_limit) << ", ";
    file << estimate.w << ", " << estimate.x << ", " << estimate.y << ", " << estimate.z << ", ";
    file << eig_quat.w() << ", " << eig_quat.x() << ", " << eig_quat.y() << ", " << eig_quat.z() << ", ";
    file << rf.estimator_.state().roll << ", " << rf.estimator_.state().pitch << ", " <<rf.estimator_.state().yaw << ", ";
    file << rf.estimator_.state().angular_velocity.x << ", " << rf.estimator_.state().angular_velocity.y << ", " << rf.estimator_.state().angular_velocity.z << ", ";
    file << p << ", " << q << ", " << r << ", ";
    file << error << "\n";
#endif

    if (std::isfinite(error))
    {
      if (error > max_error)
        max_error = error;
    }
  }
  return max_error;
}


TEST(estimator_test, linear_gyro_integration) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    5.0, // yfreq
    0.3, // zfreq
    1.0, // xamp
    0.75, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0003932 // error_limit
  };

  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, false);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_ALPHA, 0);

  double max_error = run_estimator_test("linear_gyro_sim.csv", rf, board, params);

  EXPECT_LE(max_error, params[7]);

#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}


TEST(estimator_test, quadratic_gyro_integration) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    0.1, // yfreq
    0.5, // zfreq
    1.5, // xamp
    0.4, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0000363 // error_limit
  };


  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, false);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_ALPHA, 0);

  double max_error = run_estimator_test("quad_int_sim.csv", rf, board, params);

  EXPECT_LE(max_error, params[7]);

#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}

TEST(estimator_test, mat_exp_integration) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    0.1, // yfreq
    0.5, // zfreq
    1.5, // xamp
    0.4, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0005029 // error_limit
  };

  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_ALPHA, 0);

  double max_error = run_estimator_test("mat_exp_sim.csv", rf, board, params);
  EXPECT_LE(max_error, params[7]);
#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}

TEST(estimator_test, mat_exp_quad_int) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    0.1, // yfreq
    0.5, // zfreq
    1.5, // xamp
    0.4, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0000403 // error_limit
  };

  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_ALPHA, 0);

  double max_error = run_estimator_test("mat_exp_quad_sim.csv", rf, board, params);
  EXPECT_LE(max_error, params[7]);

#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}


TEST(estimator_test, accel) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    0.1, // yfreq
    0.5, // zfreq
    1.5, // xamp
    0.4, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0280459 // error_limit
  };


  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, false);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, false);
  rf.params_.set_param_int(PARAM_ACC_ALPHA, 0);
  rf.params_.set_param_int(PARAM_GYRO_ALPHA, 0);
  rf.params_.set_param_int(PARAM_FILTER_KP, 3.0f);
  rf.params_.set_param_int(PARAM_INIT_TIME, 0.0f);

  double max_error = run_estimator_test("accel_sim.csv", rf, board, params);
  EXPECT_LE(max_error, params[7]);
#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}

TEST(estimator_test, all_features) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  std::vector<double> params = {
    10.0, // xfreq
    0.1, // yfreq
    0.5, // zfreq
    1.5, // xamp
    0.4, // yamp
    1.0, // zamp
    30.0, // tmax
    0.0632316 // error_limit
  };

  // Initialize the firmware
  rf.init();

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_float(PARAM_FILTER_KP, 2.0f);
  rf.params_.set_param_float(PARAM_ACC_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_X_BIAS, 0.0);
  rf.params_.set_param_float(PARAM_GYRO_Y_BIAS, 0.0);
  rf.params_.set_param_float(PARAM_GYRO_Z_BIAS, 0.0); // We don't converge on z bias

  double max_error = run_estimator_test("full_estimator_sim.csv", rf, board, params);
  EXPECT_LE(max_error, params[7]);

#ifdef DEBUG
  printf("max_error = %.7f\n", max_error);
#endif
}

TEST(estimator_test, level_bias_sim) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);


  std::vector<double> params = {
    0.0, // xfreq
    0.0, // yfreq
    0.0, // zfreq
    0.0, // xamp
    0.0, // yamp
    0.0, // zamp
    60.0, // tmax
    0.0280459 // error_limit
  };

  // Initialize the firmware
  rf.init();

  turbomath::Vector true_bias = {0.25, -0.15, 0.0};

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_float(PARAM_FILTER_KP, 2.0f);
  rf.params_.set_param_float(PARAM_FILTER_KI, 0.2f);
  rf.params_.set_param_float(PARAM_ACC_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_X_BIAS, true_bias.x);
  rf.params_.set_param_float(PARAM_GYRO_Y_BIAS, true_bias.y);
  rf.params_.set_param_float(PARAM_GYRO_Z_BIAS, 0.0); // We don't converge on z bias

  run_estimator_test("level_bias_sim.csv", rf, board, params);

  // Check bias at the end
  turbomath::Vector bias = rf.estimator_.state().angular_velocity - rf.sensors_.data().gyro;
  turbomath::Vector error_vec = bias - true_bias;
  float error_mag = error_vec.norm();
  EXPECT_LE(error_mag, 0.001);

#ifdef DEBUG
  printf("estimated_bias = %.7f, %.7f\n", bias.x, bias.y);
#endif
}

TEST(estimator_test, moving_bias_sim) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);


  std::vector<double> params = {
    5.0, // xfreq
    0.5, // yfreq
    0.0, // zfreq
    0.02, // xamp
    0.01, // yamp
    0.0, // zamp
    60.0, // tmax
    0.0280459 // error_limit
  };

  // Initialize the firmware
  rf.init();

  turbomath::Vector true_bias(0.01, -0.005, 0.0);

  rf.params_.set_param_int(PARAM_FILTER_USE_ACC, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_QUAD_INT, true);
  rf.params_.set_param_int(PARAM_FILTER_USE_MAT_EXP, true);
  rf.params_.set_param_float(PARAM_FILTER_KP, 0.5f);
  rf.params_.set_param_float(PARAM_FILTER_KI, 0.05f);
  rf.params_.set_param_float(PARAM_ACC_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_ALPHA, 0.0f);
  rf.params_.set_param_float(PARAM_GYRO_X_BIAS, true_bias.x);
  rf.params_.set_param_float(PARAM_GYRO_Y_BIAS, true_bias.y);
  rf.params_.set_param_float(PARAM_GYRO_Z_BIAS, 0.0); // We don't converge on z bias

  run_estimator_test("moving_bias_sim.csv", rf, board, params);

  // Check bias at the end
  turbomath::Vector bias = rf.estimator_.state().angular_velocity - rf.sensors_.data().gyro;
  turbomath::Vector error_vec = bias - true_bias;
  float error_mag = error_vec.norm();
  EXPECT_LE(error_mag, params[7]);

#ifdef DEBUG
  printf("estimated_bias = %.7f, %.7f\n", bias.x, bias.y);
#endif
}
