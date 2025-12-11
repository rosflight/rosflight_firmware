#include "mavlink.h"
#include "test_board.h"

#include "rosflight.h"

#include <gtest/gtest.h>

using namespace rosflight_firmware;

#define EXPECT_PARAM_EQ_INT(PARAM_id, value) EXPECT_EQ(value, rf.params_.get_param_int(PARAM_id))
#define EXPECT_PARAM_EQ_FLOAT(PARAM_id, value) EXPECT_EQ(value, rf.params_.get_param_float(PARAM_id))

TEST(Parameters, DefaultParameters)
{
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  rf.init();

  EXPECT_PARAM_EQ_INT(PARAM_BAUD_RATE, 921600); // Baud rate of MAVlink communication with companion computer | 9600 | 921600
  EXPECT_PARAM_EQ_INT(PARAM_SERIAL_DEVICE, 0); // Serial Port (for supported devices) | 0 | 3
  EXPECT_PARAM_EQ_INT(PARAM_NUM_MOTORS, 4); // Number of vertical-facing motors on the vehicle | 1 | 8
  EXPECT_PARAM_EQ_FLOAT(PARAM_MOTOR_RESISTANCE, 0.042f); // Electrical resistance of the motor windings (ohms) | 0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_MOTOR_KV, 0.01706f); // Back emf constant of the motor in SI units (V/rad/s) | 0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PROP_DIAMETER, 0.381f); // Diameter of the propeller in meters | 0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PROP_CT, 0.075f); // Thrust coefficient of the propeller | 0 | 100.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PROP_CQ, 0.0045f); // Torque coefficient of the propeller | 0 | 100.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_VOLT_MAX, 25.0f); // Maximum voltage of the battery (V) | 0 | 100.0
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_0, 0); // Output type of mixer output 0. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_1, 0); // Output type of mixer output 1. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_2, 0); // Output type of mixer output 2. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_3, 0); // Output type of mixer output 3. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_4, 0); // Output type of mixer output 4. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_5, 0); // Output type of mixer output 5. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_6, 0); // Output type of mixer output 6. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_7, 0); // Output type of mixer output 7. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_8, 0); // Output type of mixer output 8. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER_OUTPUT_9, 0); // Output type of mixer output 9. | 0 | 1 | 2 | 3
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_0, 0.0f); // Value of the custom mixer at element [0,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_0, 0.0f); // Value of the custom mixer at element [1,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_0, 0.0f); // Value of the custom mixer at element [2,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_0, 0.0f); // Value of the custom mixer at element [3,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_0, 0.0f); // Value of the custom mixer at element [4,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_0, 0.0f); // Value of the custom mixer at element [5,0] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_1, 0.0f); // Value of the custom mixer at element [0,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_1, 0.0f); // Value of the custom mixer at element [1,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_1, 0.0f); // Value of the custom mixer at element [2,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_1, 0.0f); // Value of the custom mixer at element [3,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_1, 0.0f); // Value of the custom mixer at element [4,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_1, 0.0f); // Value of the custom mixer at element [5,1] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_2, 0.0f); // Value of the custom mixer at element [0,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_2, 0.0f); // Value of the custom mixer at element [1,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_2, 0.0f); // Value of the custom mixer at element [2,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_2, 0.0f); // Value of the custom mixer at element [3,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_2, 0.0f); // Value of the custom mixer at element [4,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_2, 0.0f); // Value of the custom mixer at element [5,2] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_3, 0.0f); // Value of the custom mixer at element [0,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_3, 0.0f); // Value of the custom mixer at element [1,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_3, 0.0f); // Value of the custom mixer at element [2,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_3, 0.0f); // Value of the custom mixer at element [3,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_3, 0.0f); // Value of the custom mixer at element [4,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_3, 0.0f); // Value of the custom mixer at element [5,3] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_4, 0.0f); // Value of the custom mixer at element [0,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_4, 0.0f); // Value of the custom mixer at element [1,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_4, 0.0f); // Value of the custom mixer at element [2,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_4, 0.0f); // Value of the custom mixer at element [3,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_4, 0.0f); // Value of the custom mixer at element [4,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_4, 0.0f); // Value of the custom mixer at element [5,4] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_5, 0.0f); // Value of the custom mixer at element [0,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_5, 0.0f); // Value of the custom mixer at element [1,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_5, 0.0f); // Value of the custom mixer at element [2,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_5, 0.0f); // Value of the custom mixer at element [3,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_5, 0.0f); // Value of the custom mixer at element [4,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_5, 0.0f); // Value of the custom mixer at element [5,5] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_6, 0.0f); // Value of the custom mixer at element [0,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_6, 0.0f); // Value of the custom mixer at element [1,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_6, 0.0f); // Value of the custom mixer at element [2,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_6, 0.0f); // Value of the custom mixer at element [3,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_6, 0.0f); // Value of the custom mixer at element [4,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_6, 0.0f); // Value of the custom mixer at element [5,6] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_7, 0.0f); // Value of the custom mixer at element [0,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_7, 0.0f); // Value of the custom mixer at element [1,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_7, 0.0f); // Value of the custom mixer at element [2,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_7, 0.0f); // Value of the custom mixer at element [3,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_7, 0.0f); // Value of the custom mixer at element [4,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_7, 0.0f); // Value of the custom mixer at element [5,7] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_8, 0.0f); // Value of the custom mixer at element [0,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_8, 0.0f); // Value of the custom mixer at element [1,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_8, 0.0f); // Value of the custom mixer at element [2,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_8, 0.0f); // Value of the custom mixer at element [3,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_8, 0.0f); // Value of the custom mixer at element [4,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_8, 0.0f); // Value of the custom mixer at element [5,8] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_0_9, 0.0f); // Value of the custom mixer at element [0,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_1_9, 0.0f); // Value of the custom mixer at element [1,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_2_9, 0.0f); // Value of the custom mixer at element [2,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_3_9, 0.0f); // Value of the custom mixer at element [3,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_4_9, 0.0f); // Value of the custom mixer at element [4,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_PRIMARY_MIXER_5_9, 0.0f); // Value of the custom mixer at element [5,9] | -inf | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_0, 0.0f); // Value of the custom mixer at element [0,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_0, 0.0f); // Value of the custom mixer at element [1,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_0, 0.0f); // Value of the custom mixer at element [2,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_0, 0.0f); // Value of the custom mixer at element [3,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_0, 0.0f); // Value of the custom mixer at element [4,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_0, 0.0f); // Value of the custom mixer at element [5,0] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_1, 0.0f); // Value of the custom mixer at element [0,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_1, 0.0f); // Value of the custom mixer at element [1,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_1, 0.0f); // Value of the custom mixer at element [2,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_1, 0.0f); // Value of the custom mixer at element [3,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_1, 0.0f); // Value of the custom mixer at element [4,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_1, 0.0f); // Value of the custom mixer at element [5,1] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_2, 0.0f); // Value of the custom mixer at element [0,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_2, 0.0f); // Value of the custom mixer at element [1,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_2, 0.0f); // Value of the custom mixer at element [2,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_2, 0.0f); // Value of the custom mixer at element [3,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_2, 0.0f); // Value of the custom mixer at element [4,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_2, 0.0f); // Value of the custom mixer at element [5,2] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_3, 0.0f); // Value of the custom mixer at element [0,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_3, 0.0f); // Value of the custom mixer at element [1,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_3, 0.0f); // Value of the custom mixer at element [2,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_3, 0.0f); // Value of the custom mixer at element [3,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_3, 0.0f); // Value of the custom mixer at element [4,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_3, 0.0f); // Value of the custom mixer at element [5,3] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_4, 0.0f); // Value of the custom mixer at element [0,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_4, 0.0f); // Value of the custom mixer at element [1,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_4, 0.0f); // Value of the custom mixer at element [2,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_4, 0.0f); // Value of the custom mixer at element [3,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_4, 0.0f); // Value of the custom mixer at element [4,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_4, 0.0f); // Value of the custom mixer at element [5,4] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_5, 0.0f); // Value of the custom mixer at element [0,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_5, 0.0f); // Value of the custom mixer at element [1,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_5, 0.0f); // Value of the custom mixer at element [2,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_5, 0.0f); // Value of the custom mixer at element [3,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_5, 0.0f); // Value of the custom mixer at element [4,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_5, 0.0f); // Value of the custom mixer at element [5,5] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_6, 0.0f); // Value of the custom mixer at element [0,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_6, 0.0f); // Value of the custom mixer at element [1,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_6, 0.0f); // Value of the custom mixer at element [2,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_6, 0.0f); // Value of the custom mixer at element [3,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_6, 0.0f); // Value of the custom mixer at element [4,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_6, 0.0f); // Value of the custom mixer at element [5,6] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_7, 0.0f); // Value of the custom mixer at element [0,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_7, 0.0f); // Value of the custom mixer at element [1,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_7, 0.0f); // Value of the custom mixer at element [2,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_7, 0.0f); // Value of the custom mixer at element [3,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_7, 0.0f); // Value of the custom mixer at element [4,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_7, 0.0f); // Value of the custom mixer at element [5,7] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_8, 0.0f); // Value of the custom mixer at element [0,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_8, 0.0f); // Value of the custom mixer at element [1,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_8, 0.0f); // Value of the custom mixer at element [2,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_8, 0.0f); // Value of the custom mixer at element [3,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_8, 0.0f); // Value of the custom mixer at element [4,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_8, 0.0f); // Value of the custom mixer at element [5,8] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_0_9, 0.0f); // Value of the custom mixer at element [0,9] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_1_9, 0.0f); // Value of the custom mixer at element [1,9] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_2_9, 0.0f); // Value of the custom mixer at element [2,9] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_3_9, 0.0f); // Value of the custom mixer at element [3,9] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_4_9, 0.0f); // Value of the custom mixer at element [4,9] | -inf | inf 
  EXPECT_PARAM_EQ_FLOAT(PARAM_SECONDARY_MIXER_5_9, 0.0f); // Value of the custom mixer at element [5,9] | -inf | inf 
  EXPECT_PARAM_EQ_INT(PARAM_SYSTEM_ID, 1); // Mavlink System ID  | 1 | 255
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_ROLL_RATE_P, 0.070f); // Roll Rate Proportional Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_ROLL_RATE_I, 0.000f); // Roll Rate Integral Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_ROLL_RATE_D, 0.000f); // Roll Rate Derivative Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_PITCH_RATE_I, 0.0000f); // Pitch Rate Integral Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_PITCH_RATE_D, 0.0000f); // Pitch Rate Derivative Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_PITCH_ANGLE_D, 0.05f); // Pitch Angle Derivative Gain | 0.0 | 1000.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_X_EQ_TORQUE, 0.0f); // Equilibrium torque added to output of controller on x axis | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_Y_EQ_TORQUE, 0.0f); // Equilibrium torque added to output of controller on y axis | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_Z_EQ_TORQUE, 0.0f); // Equilibrium torque added to output of controller on z axis | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_PID_TAU, 0.05f); // Dirty Derivative time constant - See controller documentation | 0.0 | 1.0
  EXPECT_PARAM_EQ_INT(PARAM_MOTOR_PWM_SEND_RATE, 0); // Overrides default PWM rate specified by mixer if non-zero - Requires reboot to take effect | 0 | 490
  EXPECT_PARAM_EQ_INT(PARAM_SPIN_MOTORS_WHEN_ARMED, true); // Enforce MOTOR_IDLE_THR | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_INIT_TIME, 3000); // Time in ms to initialize estimator | 0 | 100000
  EXPECT_PARAM_EQ_FLOAT(PARAM_FILTER_KP_ACC, 0.5f); // estimator proportional gain on accel-based error - See estimator documentation | 0 | 10.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_FILTER_KI, 0.01f); // estimator integral gain - See estimator documentation | 0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_FILTER_KP_EXT, 1.5f); // estimator proportional gain on external attitude-based error - See estimator documentation | 0 | 10.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_FILTER_ACCEL_MARGIN, 0.1f); // allowable accel norm margin around 1g to determine if accel is usable | 0 | 1.0
  EXPECT_PARAM_EQ_INT(PARAM_FILTER_USE_QUAD_INT, 1); // Perform a quadratic averaging of LPF gyro data prior to integration (adds ~20 us to estimation loop on F1 processors) | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_FILTER_USE_MAT_EXP, 1); // 1 - Use matrix exponential to improve gyro integration (adds ~90 us to estimation loop in F1 processors) 0 - use euler integration | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_CALIBRATE_GYRO_ON_ARM, false); // True if desired to calibrate gyros on arm | 0 | 1
  EXPECT_PARAM_EQ_FLOAT(PARAM_GYRO_XY_ALPHA, 0.3f); // Low-pass filter constant on gyro X and Y axes - See estimator documentation | 0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_GYRO_Z_ALPHA, 0.3f); // Low-pass filter constant on gyro Z axis - See estimator documentation | 0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_ACC_ALPHA, 0.5f); // Low-pass filter constant on all accel axes - See estimator documentation | 0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_GYRO_X_BIAS, 0.0f); // Constant x-bias of gyroscope readings | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_GYRO_Y_BIAS, 0.0f); // Constant y-bias of gyroscope readings | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_GYRO_Z_BIAS, 0.0f); // Constant z-bias of gyroscope readings | -1.0 | 1.0
  EXPECT_PARAM_EQ_FLOAT(PARAM_BARO_BIAS, 0.0f); // Barometer measurement bias (Pa) | 0 | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_GROUND_LEVEL, 1387.0f); // Altitude of ground level (m) | -1000 | 10000
  EXPECT_PARAM_EQ_FLOAT(PARAM_DIFF_PRESS_BIAS, 0.0f); // Differential Pressure Bias (Pa) | -10 | 10
  EXPECT_PARAM_EQ_INT(PARAM_RC_TYPE, 0); // Type of RC input 0 - PPM, 1 - SBUS | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_RC_X_CHANNEL, 0); // RC input channel mapped to x-axis commands [0 - indexed] | 0 | 3
  EXPECT_PARAM_EQ_INT(PARAM_RC_Y_CHANNEL, 1); // RC input channel mapped to y-axis commands [0 - indexed] | 0 | 3
  EXPECT_PARAM_EQ_INT(PARAM_RC_Z_CHANNEL, 3); // RC input channel mapped to z-axis commands [0 - indexed] | 0 | 3
  EXPECT_PARAM_EQ_INT(PARAM_RC_F_CHANNEL, 2); // RC input channel mapped to F-axis commands [0 - indexed] | 0 | 3
  EXPECT_PARAM_EQ_INT(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4); // RC switch mapped to attitude override [0 indexed, -1 to disable] | 4 | 7
  EXPECT_PARAM_EQ_INT(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4); // RC switch channel mapped to throttle override [0 indexed, -1 to disable] | 4 | 7
  EXPECT_PARAM_EQ_INT(PARAM_RC_NUM_CHANNELS, 6); // number of RC input channels | 1 | 8
  EXPECT_PARAM_EQ_INT(PARAM_RC_SWITCH_5_DIRECTION, 1); // RC switch 5 toggle direction | -1 | 1
  EXPECT_PARAM_EQ_INT(PARAM_RC_SWITCH_6_DIRECTION, 1); // RC switch 6 toggle direction | -1 | 1
  EXPECT_PARAM_EQ_INT(PARAM_RC_SWITCH_7_DIRECTION, 1); // RC switch 7 toggle direction | -1 | 1
  EXPECT_PARAM_EQ_INT(PARAM_RC_SWITCH_8_DIRECTION, 1); // RC switch 8 toggle direction | -1 | 1
  EXPECT_PARAM_EQ_INT(PARAM_OVERRIDE_LAG_TIME, 1000); // RC stick deviation lag time before returning control (ms) | 0 | 100000
  EXPECT_PARAM_EQ_INT(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true); // Take minimum throttle between RC and computer at all times | 0 | 1
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_THROTTLE, 0.7f); // Maximum throttle command sent by full deflection of RC sticks, to maintain controllability during aggressive maneuvers | 0.0 | 1.0
  EXPECT_PARAM_EQ_INT(PARAM_RC_ATTITUDE_MODE, 1); // Attitude mode for RC sticks (0: rate, 1: angle). Overridden if RC_ATT_CTRL_CHN is set. | 0 | 1
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_ROLL, 0.786f); // Maximum roll angle command sent by full deflection of RC sticks | 0.0 | 3.14159
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_PITCH, 0.786f); // Maximum pitch angle command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_ROLLRATE, 3.14159f); // Maximum roll rate command sent by full stick deflection of RC sticks | 0.0 | 9.42477796077
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_PITCHRATE, 3.14159f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  EXPECT_PARAM_EQ_FLOAT(PARAM_RC_MAX_YAWRATE, 1.507f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  EXPECT_PARAM_EQ_INT(PARAM_PRIMARY_MIXER, Mixer::INVALID_MIXER); // Which mixer to choose for primary mixer - See Mixer documentation | 0 | 11
  EXPECT_PARAM_EQ_INT(PARAM_SECONDARY_MIXER, Mixer::INVALID_MIXER); // Which mixer to choose for secondary mixer - See Mixer documentation | 0 | 11
  EXPECT_PARAM_EQ_INT(PARAM_FIXED_WING, false); // switches on pass-through commands for fixed-wing operation | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_ELEVATOR_REVERSE, 0); // reverses elevator servo output | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_AILERON_REVERSE, 0); // reverses aileron servo output | 0 | 1
  EXPECT_PARAM_EQ_INT(PARAM_RUDDER_REVERSE, 0); // reverses rudder servo output | 0 | 1
  EXPECT_PARAM_EQ_FLOAT(PARAM_FC_ROLL, 0.0f); // roll angle (deg) of flight controller wrt aircraft body | 0 | 360
  EXPECT_PARAM_EQ_FLOAT(PARAM_FC_PITCH, 0.0f); // pitch angle (deg) of flight controller wrt aircraft body | 0 | 360
  EXPECT_PARAM_EQ_FLOAT(PARAM_FC_YAW, 0.0f); // yaw angle (deg) of flight controller wrt aircraft body | 0 | 360
  EXPECT_PARAM_EQ_FLOAT(PARAM_BATTERY_VOLTAGE_MULTIPLIER, 1.0f); // Multiplier for the voltage sensor | 0 | inf
  EXPECT_PARAM_EQ_FLOAT(PARAM_BATTERY_CURRENT_MULTIPLIER, 1.0f); // Multiplier for the current sensor | 0 | inf
  EXPECT_PARAM_EQ_INT(PARAM_OFFBOARD_TIMEOUT, 100); // Timeout in milliseconds for offboard commands, after which RC override is activated | 0 | 100000
}
