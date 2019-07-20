#include "common.h"

double quaternion_error(Eigen::Quaternionf q_eig, turbomath::Quaternion q)
{
  Eigen::Quaternionf est_quat(q.w, q.x, q.y, q.z);
  Eigen::Quaternionf q_tilde = q_eig * est_quat.inverse();
  if (q_tilde.vec().norm() < 0.000001)
    return 0;
  else
  {
    Eigen::Vector3f v_tilde = atan2(q_tilde.vec().norm(), q_tilde.w())*q_tilde.vec()/q_tilde.vec().norm();
    return v_tilde.norm();
  }
}

double quaternion_error(turbomath::Quaternion q0, turbomath::Quaternion q)
{
  Eigen::Quaternionf est_quat(q.w, q.x, q.y, q.z);
  Eigen::Quaternionf q_eig(q0.w, q0.x, q0.y, q0.z);
  Eigen::Quaternionf q_tilde = q_eig * est_quat.inverse();
  if (q_tilde.vec().norm() < 0.000001)
    return 0;
  else
  {
    Eigen::Vector3f v_tilde = atan2(q_tilde.vec().norm(), q_tilde.w())*q_tilde.vec()/q_tilde.vec().norm();
    return v_tilde.norm();
  }
}

void step_firmware(rosflight_firmware::ROSflight &rf, rosflight_firmware::testBoard &board, uint32_t us)
{
  uint64_t start_time_us = board.clock_micros();
  float dummy_acc[3] = {0, 0, -9.80665};
  float dummy_gyro[3] = {0, 0, 0};
  while (board.clock_micros() < start_time_us + us)
  {
    board.set_imu(dummy_acc, dummy_gyro, board.clock_micros() + 1000);
    rf.run();
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
