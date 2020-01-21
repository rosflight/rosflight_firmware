#include "common.h"
#include "rosflight.h"
#include "test_board.h"
#include "mavlink.h"
#include "cmath"

#define CHN_LOW 1100
#define CHN_HIGH 1900

#define OFFBOARD_X -1.0
#define OFFBOARD_Y 0.5
#define OFFBOARD_Z -0.7
#define OFFBOARD_F 0.9

#define RC_X_PWM 1800
#define RC_X ((RC_X_PWM - 1500)/500.0 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL))

using namespace rosflight_firmware;

class CommandManagerTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;

  uint16_t rc_values[8];
  float max_roll, max_pitch, max_yawrate;

  control_t offboard_command =
  {
    20000,
    {true, ANGLE, OFFBOARD_X},
    {true, ANGLE, OFFBOARD_Y},
    {true, RATE,  OFFBOARD_Z},
    {true, THROTTLE, OFFBOARD_F}
  };

  CommandManagerTest() :
    mavlink(board),
    rf(board,mavlink)
  {}

  void SetUp() override
  {
    rf.init();
    rf.state_manager_.clear_error(rf.state_manager_.state().error_codes); // Clear All Errors to Start
    rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false);

    for (int i = 0; i < 8; i++)
    {
      rc_values[i] = 1500;
    }
    rc_values[2] = 1000;

    rf.params_.set_param_int(PARAM_MIXER, Mixer::PASSTHROUGH);
    max_roll = rf.params_.get_param_float(PARAM_RC_MAX_ROLL);
    max_pitch = rf.params_.get_param_float(PARAM_RC_MAX_PITCH);
    max_yawrate = rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE);
  }

  void setOffboard(control_t& command)
  {
    command.stamp_ms = rf.board_.clock_millis();
    rf.command_manager_.set_new_offboard_command(command);
  }

  void stepFirmware(uint32_t us)
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
};

TEST_F (CommandManagerTest, Default)
{
  board.set_rc(rc_values);
  stepFirmware(20000); // 20 ms

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.0);
}

TEST_F (CommandManagerTest, RCCommands)
{
  rc_values[0] = 2000;
  rc_values[1] = 1000;
  rc_values[2] = 1500;
  rc_values[3] = 1250;
  board.set_rc(rc_values);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 1.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, -1.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, -0.5*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.5);
}

TEST_F (CommandManagerTest, ArmWithSticksByDefault)
{
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  rc_values[2] = 1000;
  rc_values[3] = 2000;
  board.set_rc(rc_values);
  stepFirmware(500000);
  EXPECT_EQ(rf.state_manager_.state().armed, false); // need to wait 1 second, shouldn't be armed yet
  stepFirmware(600000);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F (CommandManagerTest, DontArmWithSticksWhenUsingSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000; // throttle low
  rc_values[3] = 2000; // yaw right
  board.set_rc(rc_values);
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (CommandManagerTest, DisarmWithSticksByDefault)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  rc_values[2] = 1000; // throttle low
  rc_values[3] = 1000; // yaw left
  board.set_rc(rc_values);
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (CommandManagerTest, ArmWithSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000; // throttle low
  rc_values[4] = CHN_HIGH; // switch on
  board.set_rc(rc_values);
  stepFirmware(50000); // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F (CommandManagerTest, DisarmWithStick)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rc_values[4] = CHN_LOW; // throttle low
  board.set_rc(rc_values);
  stepFirmware(50000); // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (CommandManagerTest, DontDisarmWithSticksWhenUsingSwitch)
{
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[4] = CHN_HIGH; // switch on
  rc_values[2] = 1000; // throttle low
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  board.set_rc(rc_values);
  stepFirmware(50000);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rc_values[2] = 1000; // throttle low
  rc_values[3] = 1000; // yaw left
  stepFirmware(1100000);
  EXPECT_EQ(rf.state_manager_.state().armed, true); // don't disarm
}

TEST_F (CommandManagerTest, ArmStickReversed)
{
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rc_values[2] = 1000; // throttle low
  rc_values[4] = CHN_LOW; // switch on
  board.set_rc(rc_values);
  stepFirmware(50000); // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F (CommandManagerTest, DisarmStickReversed)
{
  rf.params_.set_param_int(PARAM_RC_SWITCH_5_DIRECTION, -1);
  rf.params_.set_param_int(PARAM_RC_ARM_CHANNEL, 4);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rc_values[2] = 1000; // throttle low
  rc_values[4] = CHN_HIGH; // switch on
  board.set_rc(rc_values);
  stepFirmware(50000); // Immediate
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (CommandManagerTest, DefaultRCOutputd)
{
  board.set_rc(rc_values);
  stepFirmware(600000);

  // Check the output
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.0);
}


TEST_F (CommandManagerTest, RCOutput)
{
  rc_values[0] = 1250;
  rc_values[1] = 1750;
  rc_values[2] = 1500;
  rc_values[3] = 2000;
  board.set_rc(rc_values);
  stepFirmware(600000);

  // Check the output
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, max_roll * -0.5);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, max_pitch * 0.5);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, 0.5);
}

TEST_F (CommandManagerTest, LoseRCDisarmed)
{
  board.set_pwm_lost(true);
  stepFirmware(20000);

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
}

TEST_F (CommandManagerTest, RegainRCDisarmed)
{
  board.set_pwm_lost(true);
  stepFirmware(40000);
  board.set_pwm_lost(false);
  stepFirmware(40000);

  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F (CommandManagerTest, LoseRCArmed)
{
  board.set_rc(rc_values);
  stepFirmware(50000);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  board.set_pwm_lost(true);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, ANGLE);
  EXPECT_CLOSE(output.x.value, 0.0*max_roll);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_CLOSE(output.y.value, 0.0*max_pitch);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_CLOSE(output.z.value, 0.0*max_yawrate);
  EXPECT_EQ(output.F.type, THROTTLE);
  EXPECT_CLOSE(output.F.value, rf.params_.get_param_float(PARAM_FAILSAFE_THROTTLE));

  // We should also be disarmed and in error
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, true);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
}

TEST_F (CommandManagerTest, RegainRCArmed)
{
  board.set_rc(rc_values);
  stepFirmware(50000);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);

  board.set_pwm_lost(true);
  stepFirmware(20000);
  board.set_pwm_lost(false);
  stepFirmware(20000);

  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F (CommandManagerTest, OffboardCommandMuxNoMinThrottle)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, false);

  // set the new offboard command
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, 0x0);

  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, OFFBOARD_F);
}

TEST_F (CommandManagerTest, OffboardCommandMuxMinThrottle)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);

  // set the new offboard command
  offboard_command.stamp_ms = board.clock_millis();
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(20000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, 0x20);

  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, 0.0);
}

TEST_F (CommandManagerTest, OffboardCommandMuxRollDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[0] = 1250;
  board.set_rc(rc_values);
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, 0x24);

  EXPECT_CLOSE(output.x.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL));
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, 0.0);
}

TEST_F (CommandManagerTest, OffboardCommandMuxPitchDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[1] = 1750;
  board.set_rc(rc_values);
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, 0x28);

  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, 0.5 * rf.params_.get_param_float(PARAM_RC_MAX_PITCH));
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, 0.0);
}

TEST_F (CommandManagerTest, OffboardCommandMuxYawrateDeviation)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[3] = 1250;
  board.set_rc(rc_values);
  rf.command_manager_.set_new_offboard_command(offboard_command);
  stepFirmware(40000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();

  EXPECT_EQ(override, 0x30);

  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_YAWRATE));
  EXPECT_CLOSE(output.F.value, 0.0);
}

TEST_F (CommandManagerTest, OffboardCommandMuxLag)
{
  stepFirmware(1100000); // 1.1s Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  rc_values[0] = 1250; // About halfway left
  board.set_rc(rc_values);
  setOffboard(offboard_command);
  stepFirmware(40000); // 40 ms

  control_t output = rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, -0.5 * rf.params_.get_param_float(PARAM_RC_MAX_ROLL));
  uint16_t override = rf.command_manager_.get_rc_override();
  EXPECT_EQ(override, 0x24); // Throttle and X stick overrides

  rc_values[0] = 1500; // return stick to center
  board.set_rc(rc_values);

  stepFirmware(500000); // 500 ms
  setOffboard(offboard_command);
  output=rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0); // lag
  override = rf.command_manager_.get_rc_override();
  EXPECT_EQ(override, 0x3E4); // Throttle and X stick overrides, plus stale offboard

  stepFirmware(600000); // 600 ms
  setOffboard(offboard_command);
  output=rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, 0.0); // lag
  override = rf.command_manager_.get_rc_override();
  EXPECT_EQ(override, 0x3E0); // Throttle override and stale offboard

  setOffboard(offboard_command);
  stepFirmware(20000); // 20 ms
  output=rf.command_manager_.combined_control();
  EXPECT_CLOSE(output.x.value, OFFBOARD_X);
  override = rf.command_manager_.get_rc_override();
  EXPECT_EQ(override, 0x20); // Throttle override only
}

TEST_F (CommandManagerTest, StaleOffboardCommand)
{
  stepFirmware(1100000); // Get past LAG_TIME
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  setOffboard(offboard_command);

  int timeout_us = rf.params_.get_param_int(PARAM_OFFBOARD_TIMEOUT)*1000;
  stepFirmware(timeout_us + 40000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();
  EXPECT_EQ(override, 0x3E0); // Offboard not present, plus throttle limited
  EXPECT_CLOSE(output.x.value, 0.0);
}

TEST_F (CommandManagerTest, PartialMux)
{
  offboard_command.x.active = false;
  stepFirmware(1000000);
  setOffboard(offboard_command);
  stepFirmware(30000);

  control_t output = rf.command_manager_.combined_control();
  uint16_t override = rf.command_manager_.get_rc_override();
  EXPECT_CLOSE(output.x.value, 0.0);
  EXPECT_CLOSE(output.y.value, OFFBOARD_Y);
  EXPECT_CLOSE(output.z.value, OFFBOARD_Z);
  EXPECT_CLOSE(output.F.value, 0.0);
  EXPECT_EQ(override, 0x60); // X channel stale and Throttle override
}

TEST_F (CommandManagerTest, MixedTypes)
{
  offboard_command.x.type = RATE;
  stepFirmware(1000000);
  setOffboard(offboard_command);
  stepFirmware(30000);

  control_t output = rf.command_manager_.combined_control();
  EXPECT_EQ(output.x.type, RATE);
  EXPECT_EQ(output.y.type, ANGLE);
  EXPECT_EQ(output.z.type, RATE);
  EXPECT_EQ(output.F.type, THROTTLE);
}
