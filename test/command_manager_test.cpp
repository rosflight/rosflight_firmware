#include "common.h"
#include "rosflight.h"
#include "test_board.h"
#include "mavlink.h"
#include "cmath"

#define CHN_LOW 1100
#define CHN_HIGH 1900


using namespace rosflight_firmware;

// Initialize the full firmware, so that the state_manager can do its thing
void step_firmware(ROSflight& rf, testBoard& board, uint32_t us)
{
  uint64_t start_time_us = board.clock_micros();
  float dummy_acc[3] = {0, 0, -9.80665};
  float dummy_gyro[3] = {0, 0, 0};
  while(board.clock_micros() < start_time_us + us)
  {
    board.set_imu(dummy_acc, dummy_gyro, board.clock_micros() + 1000);
    rf.run();
  }
}

TEST(command_manager_test, rc) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Initialize the firmware
  rf.init();

  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = 1500;
  }
  rc_values[2] = 1000;

  float max_roll = rf.params_.get_param_float(PARAM_RC_MAX_ROLL);
  float max_pitch = rf.params_.get_param_float(PARAM_RC_MAX_PITCH);
  float max_yawrate = rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE);


  //=================================================
  // RC Commands Test
  //=================================================
  // First, lets just try sending rc_commands alone
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.0);

  rc_values[0] = 2000;
  rc_values[1] = 1000;
  rc_values[2] = 1500;
  rc_values[3] = 1250;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);

  output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 1.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, -1.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, -0.5*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.5);
}


TEST(command_manager_test, rc_arm_disarm) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Make sure that rc is hooked up
  board.set_pwm_lost(false);

  // Initialize the firmware
  rf.init();

  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = 1500;
  }
  rc_values[2] = 1000;

  float max_roll = rf.params_.get_param_float(PARAM_RC_MAX_ROLL);
  float max_pitch = rf.params_.get_param_float(PARAM_RC_MAX_PITCH);
  float max_yawrate = rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE);

  // Let's clear all errors in the state_manager
  rf.state_manager_.clear_error(rf.state_manager_.state().error_codes);

  //=================================================
  // RC Arming Test
  //=================================================

  // Let's send an arming signal
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 2000;
  board.set_rc(rc_values);
  // Step halfway long enough to arm (shouldn't be armed yet)
  step_firmware(rf, board, 500000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);


  // Wait the rest of the time
  step_firmware(rf, board, 600000);
  // Check the output
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 1.0*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.0);

  // See that we are armed
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  // Let's send a disarming signal
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 1000;
  board.set_rc(rc_values);
  // Step long enough for an arm to happen
  step_firmware(rf, board, 1200000);

  // See that we are disarmed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);


  //=================================================
  // Switch Arming Test
  //=================================================
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 1500;
  rc_values[4] = 1500;

  // Set all stick neutral position
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // make sure we are still disarmed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);

  // flip the arm switch on
  rc_values[4] = CHN_HIGH;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should be armed
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);

  // flip the arm switch off
  rc_values[4] = CHN_LOW;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should be disarmed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);

  // Try reversing the arm channel
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);
  rc_values[4] = CHN_LOW;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should be armed
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);

  // now it should be off
  rc_values[4] = CHN_HIGH;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should be disarmed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);


  //=================================================
  // Edge Cases
  //=================================================

  // Try to arm with the sticks
  // Let's send an arming signal (it shouldn't work)
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 2000;
  board.set_rc(rc_values);
  step_firmware(rf, board, 1100000);
  // Check the output
  output = rf.command_manager_.combined_control();
  EXPECT_EQ(rf.state_manager_.state().armed, false);

  // Go back to arming with sticks
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, -1);
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);

  // try to arm with all the switches (first put all switches CHN_HIGH)
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 1500;
  rc_values[4] = CHN_HIGH;
  rc_values[5] = CHN_HIGH;
  rc_values[6] = CHN_HIGH;
  rc_values[7] = CHN_HIGH;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should not be armed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  // now all switches CHN_LOW
  rc_values[4] = CHN_LOW;
  rc_values[5] = CHN_LOW;
  rc_values[6] = CHN_LOW;
  rc_values[7] = CHN_LOW;
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  // we should not be armed
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}


TEST(command_manager_test, rc_failsafe_test) {
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Make sure that rc is hooked up
  board.set_pwm_lost(false);

  // Initialize the firmware
  rf.init();

  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = 1500;
  }
  rc_values[2] = 1000;

  float max_roll = rf.params_.get_param_float(PARAM_RC_MAX_ROLL);
  float max_pitch = rf.params_.get_param_float(PARAM_RC_MAX_PITCH);
  float max_yawrate = rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE);
  float failsafe_throttle = rf.params_.get_param_float(PARAM_FAILSAFE_THROTTLE);

  // Let's clear all errors in the state_manager
  rf.state_manager_.clear_error(rf.state_manager_.state().error_codes);

  //=================================================
  // Disarmed Failsafe
  //=================================================

  // Let's lose rc while disarmed
  board.set_pwm_lost(true);
  step_firmware(rf, board, 20000);
  // Check the output - This should be the last rc value
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.0);

  // We should also be disarmed and in error
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);

  // Lets regain rc
  board.set_pwm_lost(false);
  board.set_rc(rc_values);
  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0*max_roll);
  EXPECT_CLOSE(output.y.value, 0.0*max_pitch);
  EXPECT_CLOSE(output.z.value, 0.0*max_yawrate);
  EXPECT_CLOSE(output.F.value, 0.0);

  // We should still be disarmed, but no failsafe or error
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x00);

  //=================================================
  // Armed Failsafe
  //=================================================

  // Let's send an arming signal
  rc_values[0] = 1500;
  rc_values[1] = 1500;
  rc_values[2] = 1000;
  rc_values[3] = 2000;
  board.set_rc(rc_values);
  // Step long enough for an arm to happen
  while (board.clock_millis() < 1200000)
  {
    step_firmware(rf, board, 20000);
  }

  // check that we are armed
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);

  // Set a command on the sticks
  rc_values[0] = 1750;
  rc_values[1] = 1250;
  rc_values[2] = 1600;
  rc_values[3] = CHN_LOW;

  // Lost RC
  board.set_rc(rc_values);
  board.set_pwm_lost(true);
  step_firmware(rf, board, 20000);
  // Check the output - This should be the failsafe control
  output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, failsafe_throttle);

  // We should still be armed, but now in failsafe
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, true);

  // Regain RC
  board.set_pwm_lost(false);
  step_firmware(rf, board, 20000);
  // Check the output - This should be our rc control
  output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.5*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, -0.5*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, -0.8*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.6);
}



#define OFFBOARD_X -1.0
#define OFFBOARD_Y 0.5
#define OFFBOARD_Z -0.7
#define OFFBOARD_F 0.9

#define RC_X_PWM 1800
#define RC_X ((RC_X_PWM - 1500)/500.0 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL))

TEST(command_manager_test, rc_offboard_muxing_test ) {

  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Make sure that rc is hooked up
  board.set_pwm_lost(false);

  // Initialize the firmware
  rf.init();
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, false);

  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = 1500;
  }
  rc_values[2] = 1000;

  // Let's clear all errors in the state_manager
  rf.state_manager_.clear_error(rf.state_manager_.state().error_codes);

  //=================================================
  // Offboard Command Integration
  //=================================================

  control_t offboard_command =
  {
    20000,
    {true, ANGLE, OFFBOARD_X},
    {true, ANGLE, OFFBOARD_Y},
    {true, RATE,  OFFBOARD_Z},
    {true, THROTTLE, OFFBOARD_F}
  };

  // First, just set an offboard command and rc, mux it and see what happens
  board.set_rc(rc_values);
  // step a bunch of times to clear the "lag time" on RC
  while (board.clock_micros() < 1000000)
  {
    step_firmware(rf, board, 20000);
  }

  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  // We don't have an override channel mapped, so this should be offboard
  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);

  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, 1);

  // ensure that the override switch is off
  rc_values[4] = CHN_LOW;

  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  // This should be offboard
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);

  // flip override switch on
  rc_values[4] = CHN_HIGH;
  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  // This should be RC
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_CLOSE(output.F.value, 0.0);


  //=================================================
  // Partial Offboard Command Integration
  //=================================================

  // Only override attitude
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, -1);

  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  // Throttle should be offboard
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);


  // Only override throttle
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, -1);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4);

  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  // Throttle should be rc
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, 0.0);


  //=================================================
  // RC Intervention
  //=================================================

  // Only override attitude
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4);

  // switch is off, but roll channel is deviated
  rc_values[4] = CHN_LOW;
  rc_values[0] = RC_X_PWM;
  rc_values[2] = 1000;  // move throttle to neutral position

  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);

  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, RC_X);  // This channel should be overwritten
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);

  //=================================================
  // RC Override Lag Test
  //=================================================

  // Switch is still off, and rc no longer deviated.  There should still be lag though
  rc_values[0] = 1500;
  rc_values[2] = 1000;
  board.set_rc(rc_values);
  uint64_t start_ms = board.clock_millis();
  step_firmware(rf, board, 20000);
  while (board.clock_millis() < rf.params_.get_param_int(PARAM_OVERRIDE_LAG_TIME) + start_ms)
  {
    output = rf.command_manager_.combined_control();
    EXPECT_CLOSE(output.x.value, 0.0);  // This channel should be overwritten
    EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
    EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
    EXPECT_CLOSE(output.F.value, OFFBOARD_F);
    offboard_command.stamp_ms = board.clock_millis();
    rf.command_manager_.set_new_offboard_command(offboard_command);
    step_firmware(rf, board, 20000);
  }
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, -1.0);  // This channel should no longer be overwritten


  //=================================================
  // Offboard Command Stale
  //=================================================

  start_ms = board.clock_millis();
  while (board.clock_millis() < 100 + start_ms)
  {
    output = rf.command_manager_.combined_control();
    EXPECT_CLOSE(output.x.value, OFFBOARD_X);  // Offboard Command is still valid
    EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
    EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
    EXPECT_CLOSE(output.F.value, OFFBOARD_F);
    step_firmware(rf, board, 20000);
  }

  // Offboard command has timed out -> revert to RC
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_CLOSE(output.F.value, 0.0);
}


TEST(command_manager_test, partial_muxing_test ) {

  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Make sure that rc is hooked up
  board.set_pwm_lost(false);

  // Initialize the firmware
  rf.init();

  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = 1500;
  }
  rc_values[2] = 1000;

  // Let's clear all errors in the state_manager
  rf.state_manager_.clear_error(rf.state_manager_.state().error_codes);

  //=================================================
  // Min Throttle Test
  //=================================================

  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);

  control_t offboard_command =
  {
    20000,
    {true, ANGLE, OFFBOARD_X},
    {true, ANGLE, OFFBOARD_Y},
    {true, RATE,  OFFBOARD_Z},
    {true, THROTTLE, OFFBOARD_F}
  };

  // step a bunch of times to clear the "lag time" on RC
  while (board.clock_micros() < 1000000)
  {
    step_firmware(rf, board, 20000);
  }

  rc_values[2] = 1200;

  // RC is the min throttle
  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);
  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.F.value, 0.2);

  // Now, offboard is the min throttle
  offboard_command.F.value = 0.2;
  rc_values[2] = 1500;

  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.F.value, 0.2);

  // Okay, remove the Min throttle setting
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, false);

  rc_values[2] = 1200;
  offboard_command.F.value = OFFBOARD_F;

  // RC is the min throttle
  board.set_rc(rc_values);
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  // We should get offboard command, even though RC is lower
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);


  // Now, let's disable the pitch channel on the onboard command
  offboard_command.y.active = false;
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);

  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);

  // Let's change the type on the x channel
  offboard_command.x.type = RATE;
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);

  step_firmware(rf, board, 20000);
  output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, RATE);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_EQ(output.F.type, THROTTLE);
}
