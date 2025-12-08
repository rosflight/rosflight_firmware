#include "common.h"
#include "mavlink.h"
#include "test_board.h"

#include "rosflight.h"

#include <cmath>

#define CHN_LOW 1100
#define CHN_HIGH 1900

#define OFFBOARD_QX -1.0
#define OFFBOARD_QY 0.5
#define OFFBOARD_QZ -0.7
#define OFFBOARD_FX 0.9
#define OFFBOARD_FY 0.9
#define OFFBOARD_FZ 0.9

#define RC_X_PWM 1800
#define RC_X ((RC_X_PWM - 1500) / 500.0 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL))

using namespace rosflight_firmware;

class CommandManagerTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;
  float last_set_rc = 0;
  bool rc_lost = false;
  uint16_t rc_values[8];
  float max_roll, max_pitch, max_yawrate;

  control_t offboard_command = {20000,
                                {true, ANGLE, OFFBOARD_QX},
                                {true, ANGLE, OFFBOARD_QY},
                                {true, RATE, OFFBOARD_QZ},
                                {true, THROTTLE, OFFBOARD_FX},
                                {true, THROTTLE, OFFBOARD_FY},
                                {true, THROTTLE, OFFBOARD_FZ}};

  uint16_t default_rc_override = CommandManager::OVERRIDE_X
    | CommandManager::OVERRIDE_Y
    | CommandManager::OVERRIDE_Z
    | CommandManager::OVERRIDE_OFFBOARD_X_INACTIVE
    | CommandManager::OVERRIDE_OFFBOARD_Y_INACTIVE
    | CommandManager::OVERRIDE_OFFBOARD_Z_INACTIVE
    | CommandManager::OVERRIDE_OFFBOARD_T_INACTIVE;

  CommandManagerTest()
      : mavlink(board)
      , rf(board, mavlink)
  {}

  void SetUp() override
  {
    rf.init();
    rf.state_manager_.clear_error(
      rf.state_manager_.state().error_codes); // Clear All Errors to Start
    rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false);

    for (int i = 0; i < 8; i++) { rc_values[i] = 1500; }
    rc_values[2] = 1000;

    rf.params_.set_param_int(PARAM_PRIMARY_MIXER, Mixer::ESC_CALIBRATION);
    rf.params_.set_param_float(PARAM_FAILSAFE_THROTTLE, 0.0);
    max_roll = rf.params_.get_param_float(PARAM_RC_MAX_ROLL);
    max_pitch = rf.params_.get_param_float(PARAM_RC_MAX_PITCH);
    max_yawrate = rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE);
  }

  void setOffboard(control_t & command)
  {
    command.stamp_ms = rf.board_.clock_millis();
    rf.command_manager_.set_new_offboard_command(command);
  }

  void stepFirmware(uint32_t us)
  {
    uint64_t start_time_us = board.clock_micros();
    float dummy_acc[3] = {0, 0, -9.80665};
    float dummy_gyro[3] = {0, 0, 0};
    while (board.clock_micros() < start_time_us + us) {
      if (board.clock_millis() > last_set_rc + 20) {
        last_set_rc = board.clock_millis();
        rf.rc_.fake_rx(rc_values, 8, rc_lost, false); // the false is for failsafe.
      }
      board.set_imu(dummy_acc, dummy_gyro, board.clock_micros() + 1000);
      rf.run();
    }
  }
};

TEST_F(CommandManagerTest, Default)
{
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, 0.0);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, 0.0);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, 0.0);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, 0.0);
}

TEST_F(CommandManagerTest, RCCommands)
{
  rc_values[0] = 2000;
  rc_values[1] = 1000;
  rc_values[2] = 1500;
  rc_values[3] = 1250;
  stepFirmware(50000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, 1.0 * max_roll);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, -1.0 * max_pitch);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, -0.5 * max_yawrate);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, 0.5);
}

TEST_F(CommandManagerTest, ArmWithSticksByDefault)
{
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  rc_values[2] = 1000;
  rc_values[3] = 2000;
  stepFirmware(500000);
  EXPECT_EQ(rf.state_manager_.state().armed,
            false); // need to wait 1 second, shouldn't be armed yet
  stepFirmware(600000);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F(CommandManagerTest, DontArmWithSticksWhenUsingSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000; // throttle low
  rc_values[3] = 2000; // yaw right
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F(CommandManagerTest, DisarmWithSticksByDefault)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  rc_values[2] = 1000; // throttle low
  rc_values[3] = 1000; // yaw left
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F(CommandManagerTest, ArmWithSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000;     // throttle low
  rc_values[4] = CHN_HIGH; // switch on
  stepFirmware(50000);     // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F(CommandManagerTest, DisarmWithStick)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rc_values[4] = CHN_LOW; // throttle low
  stepFirmware(50000);    // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F(CommandManagerTest, DontDisarmWithSticksWhenUsingSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[4] = CHN_HIGH; // switch on
  rc_values[2] = 1000;     // throttle low
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  stepFirmware(50000);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rc_values[2] = 1000; // throttle low
  rc_values[3] = 1000; // yaw left
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, true); // don't disarm
}

TEST_F(CommandManagerTest, ArmStickReversed)
{
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000;    // throttle low
  rc_values[4] = CHN_LOW; // switch on
  stepFirmware(50000);    // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F(CommandManagerTest, DisarmStickReversed)
{
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rc_values[2] = 1000;     // throttle low
  rc_values[4] = CHN_HIGH; // switch on
  stepFirmware(50000);     // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F(CommandManagerTest, DefaultRCOutput)
{
  stepFirmware(600000);

  // Check the output
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, 0.0);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, 0.0);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, 0.0);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, 0.0);
}

TEST_F(CommandManagerTest, RCOutput)
{
  rc_values[0] = 1250;
  rc_values[1] = 1750;
  rc_values[2] = 1500;
  rc_values[3] = 2000;
  stepFirmware(600000);

  // Check the output
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, max_roll * -0.5);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, max_pitch * 0.5);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, max_yawrate);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, 0.5);
}

TEST_F(CommandManagerTest, LoseRCDisarmed)
{

  rc_lost = true;
  stepFirmware(50000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, 0.0 * max_roll);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, 0.0 * max_pitch);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, 0.0 * max_yawrate);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, 0.0);

  // We should also be disarmed and in error
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
}

TEST_F(CommandManagerTest, RegainRCDisarmed)
{
  rc_lost = true;
  stepFirmware(40000);

  rc_lost = false;
  stepFirmware(40000);

  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F(CommandManagerTest, LoseRCArmed)
{
  stepFirmware(50000);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rc_lost = true;
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, ANGLE);
  EXPECT_CLOSE(output.Qx.value, 0.0 * max_roll);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_CLOSE(output.Qy.value, 0.0 * max_pitch);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_CLOSE(output.Qz.value, 0.0 * max_yawrate);
  EXPECT_EQ(output.Fz.type, THROTTLE);
  EXPECT_CLOSE(output.Fz.value, rf.params_.get_param_float(PARAM_FAILSAFE_THROTTLE));

  // We should also be disarmed and in error
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, true);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
}

TEST_F(CommandManagerTest, RegainRCArmed)
{
  stepFirmware(50000);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);

  rc_lost = true;
  stepFirmware(20000);

  rc_lost = false;
  stepFirmware(20000);

  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F(CommandManagerTest, OffboardCommandMuxNoMinThrottle)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, false);

  // set the new offboard command
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, OFFBOARD_QX);
  EXPECT_CLOSE(output.Qy.value, OFFBOARD_QY);
  EXPECT_CLOSE(output.Qz.value, OFFBOARD_QZ);
  EXPECT_CLOSE(output.Fx.value, OFFBOARD_FX);
  EXPECT_CLOSE(output.Fy.value, OFFBOARD_FY);
  EXPECT_CLOSE(output.Fz.value, OFFBOARD_FZ);
  EXPECT_EQ(rf.command_manager_.get_rc_override(), CommandManager::OVERRIDE_NO_OVERRIDE);
}

TEST_F(CommandManagerTest, OffboardCommandMuxMinThrottle)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);

  // set the new offboard command
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, OFFBOARD_QX);
  EXPECT_CLOSE(output.Qy.value, OFFBOARD_QY);
  EXPECT_CLOSE(output.Qz.value, OFFBOARD_QZ);
  EXPECT_CLOSE(output.Fx.value, 0.0);
  EXPECT_CLOSE(output.Fy.value, 0.0);
  EXPECT_CLOSE(output.Fz.value, 0.0);

  EXPECT_EQ(rf.command_manager_.get_rc_override(), CommandManager::OVERRIDE_T);
}

TEST_F(CommandManagerTest, OffboardCommandMuxRollDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[0] = 1250;
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL));
  EXPECT_CLOSE(output.Qy.value, OFFBOARD_QY);
  EXPECT_CLOSE(output.Qz.value, OFFBOARD_QZ);
  EXPECT_CLOSE(output.Fx.value, 0.0);
  EXPECT_CLOSE(output.Fy.value, 0.0);
  EXPECT_CLOSE(output.Fz.value, 0.0);

  uint16_t correct_rc_override =
    CommandManager::OVERRIDE_T
    | CommandManager::OVERRIDE_X;
  EXPECT_EQ(rf.command_manager_.get_rc_override(), correct_rc_override);
}

TEST_F(CommandManagerTest, OffboardCommandMuxPitchDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[1] = 1750;
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, OFFBOARD_QX);
  EXPECT_CLOSE(output.Qy.value, 0.5 * rf.params_.get_param_float(PARAM_RC_MAX_PITCH));
  EXPECT_CLOSE(output.Qz.value, OFFBOARD_QZ);
  EXPECT_CLOSE(output.Fx.value, 0.0);
  EXPECT_CLOSE(output.Fy.value, 0.0);
  EXPECT_CLOSE(output.Fz.value, 0.0);

  uint16_t correct_rc_override =
    CommandManager::OVERRIDE_T
    | CommandManager::OVERRIDE_Y;
  EXPECT_EQ(rf.command_manager_.get_rc_override(), correct_rc_override);
}

TEST_F(CommandManagerTest, OffboardCommandMuxYawrateDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[3] = 1250;
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, OFFBOARD_QX);
  EXPECT_CLOSE(output.Qy.value, OFFBOARD_QY);
  EXPECT_CLOSE(output.Qz.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE));
  EXPECT_CLOSE(output.Fx.value, 0.0);
  EXPECT_CLOSE(output.Fy.value, 0.0);
  EXPECT_CLOSE(output.Fz.value, 0.0);

  uint16_t correct_rc_override =
    CommandManager::OVERRIDE_T
    | CommandManager::OVERRIDE_Z;
  EXPECT_EQ(rf.command_manager_.get_rc_override(), correct_rc_override);
}

TEST_F(CommandManagerTest, OffboardCommandMuxLag)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[0] = 1250;
  setOffboard(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL));
  EXPECT_EQ(rf.command_manager_.get_rc_override(),
            rf.command_manager_.get_rc_override() | CommandManager::OVERRIDE_X);

  rc_values[0] = 1500; // return stick to center

  stepFirmware(500000);
  setOffboard(offboard_command);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, 0.0); // lag
  EXPECT_EQ(rf.command_manager_.get_rc_override(),
            rf.command_manager_.get_rc_override() | CommandManager::OVERRIDE_X);

  stepFirmware(600000);
  setOffboard(offboard_command);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, 0.0); // lag

  setOffboard(offboard_command);
  stepFirmware(20000);
  output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, OFFBOARD_QX);
  EXPECT_NE(rf.command_manager_.get_rc_override(),
            rf.command_manager_.get_rc_override() | CommandManager::OVERRIDE_X);
}

TEST_F(CommandManagerTest, StaleOffboardCommand)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  setOffboard(offboard_command);

  int timeout_us = rf.params_.get_param_int(PARAM_OFFBOARD_TIMEOUT) * 1000;
  stepFirmware(timeout_us + 40000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, 0.0);
  EXPECT_EQ(rf.command_manager_.get_rc_override(),
            rf.command_manager_.get_rc_override() | CommandManager::OVERRIDE_OFFBOARD_X_INACTIVE);
}

TEST_F(CommandManagerTest, PartialMux)
{
  offboard_command.Qx.active = false;
  stepFirmware(1000000);
  setOffboard(offboard_command);
  stepFirmware(30000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.Qx.value, 0.0);
  EXPECT_CLOSE(output.Qy.value, OFFBOARD_QY);
  EXPECT_CLOSE(output.Qz.value, OFFBOARD_QZ);
  EXPECT_CLOSE(output.Fx.value, 0.0);
  EXPECT_CLOSE(output.Fy.value, 0.0);
  EXPECT_CLOSE(output.Fz.value, 0.0);
  EXPECT_EQ(rf.command_manager_.get_rc_override(),
            rf.command_manager_.get_rc_override() | CommandManager::OVERRIDE_OFFBOARD_X_INACTIVE);
}

TEST_F(CommandManagerTest, MixedTypes)
{
  offboard_command.Qx.type = RATE;
  stepFirmware(1000000);
  setOffboard(offboard_command);
  stepFirmware(30000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.Qx.type, RATE);
  EXPECT_EQ(output.Qy.type, ANGLE);
  EXPECT_EQ(output.Qz.type, RATE);
  EXPECT_EQ(output.Fz.type, THROTTLE);
}

TEST_F(CommandManagerTest, DefaultRCOverride)
{
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, default_rc_override);
}

TEST_F(CommandManagerTest, RCOverrideGivenBothOvrdSwitchesOn)
{
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4);
  rc_values[4] = CHN_HIGH;
  stepFirmware(1000000);

  uint16_t override = rf.command_manager_.get_rc_override();

  uint16_t correct_override =
    CommandManager::OVERRIDE_ATT_SWITCH
    | CommandManager::OVERRIDE_THR_SWITCH
    | default_rc_override;
  EXPECT_EQ(override, correct_override);
}

TEST_F(CommandManagerTest, RCOverrideOnlyThrOvrdSwitchOn)
{
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 5);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 4);
  rc_values[4] = CHN_HIGH;
  rc_values[5] = CHN_LOW;
  stepFirmware(1000000);

  uint16_t override = rf.command_manager_.get_rc_override();

  uint16_t correct_override =
    CommandManager::OVERRIDE_THR_SWITCH
    | default_rc_override;
  EXPECT_EQ(override, correct_override);
}

TEST_F(CommandManagerTest, RCOverrideOnlyAttOvrdSwitchOn)
{
  rf.params_.set_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 4);
  rf.params_.set_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 5);
  rc_values[4] = CHN_HIGH;
  rc_values[5] = CHN_LOW;
  stepFirmware(1000000);

  uint16_t override = rf.command_manager_.get_rc_override();

  uint16_t correct_override =
    CommandManager::OVERRIDE_ATT_SWITCH
    | default_rc_override;
  EXPECT_EQ(override, correct_override);
}
