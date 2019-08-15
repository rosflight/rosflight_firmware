#include "common.h"

#include "rosflight.h"
#include "mavlink.h"
#include "test_board.h"

using namespace rosflight_firmware;

class StateMachineTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;

  StateMachineTest() :
    mavlink(board),
    rf(board,mavlink)
  {}

  void SetUp() override
  {
    rf.init();
    rf.state_manager_.clear_error(rf.state_manager_.state().error_codes); // Clear All Errors to Start
    rf.params_.set_param_int(PARAM_MIXER, 10);
    rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false); // default to turning this off
  }

};

TEST_F(StateMachineTest, Init)
{
  // Should be in PREFLIGHT MODE
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x00);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F(StateMachineTest, SetAndClearAllErrors)
{
  // Try setting and clearing all the errors
  for (int error = 0x0001; error <= StateManager::ERROR_UNCALIBRATED_IMU; error *= 2)
  {
    // set the error
    rf.state_manager_.set_error(error);
    EXPECT_EQ(rf.state_manager_.state().armed, false);
    EXPECT_EQ(rf.state_manager_.state().failsafe, false);
    EXPECT_EQ(rf.state_manager_.state().error_codes, error);
    EXPECT_EQ(rf.state_manager_.state().error, true);

    // clear the error
    rf.state_manager_.clear_error(error);
    EXPECT_EQ(rf.state_manager_.state().armed, false);
    EXPECT_EQ(rf.state_manager_.state().failsafe, false);
    EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
    EXPECT_EQ(rf.state_manager_.state().error, false);
  }
}

TEST_F(StateMachineTest, SetAndClearComboErrors)
{
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
      StateManager::ERROR_TIME_GOING_BACKWARDS |
      StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x32);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F(StateMachineTest, AddErrorAfterPreviousError)
{
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
      StateManager::ERROR_TIME_GOING_BACKWARDS |
      StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x33);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F(StateMachineTest, ClearOneErrorOutOfMany)
{
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
      StateManager::ERROR_TIME_GOING_BACKWARDS |
      StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  rf.state_manager_.clear_error(StateManager::ERROR_UNCALIBRATED_IMU);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_IMU_NOT_RESPONDING |
                                                   StateManager::ERROR_TIME_GOING_BACKWARDS);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F(StateMachineTest, ClearMultipleErrorsAtOnce)
{
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
      StateManager::ERROR_TIME_GOING_BACKWARDS |
      StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  rf.state_manager_.clear_error(StateManager::ERROR_IMU_NOT_RESPONDING |
                                StateManager::ERROR_TIME_GOING_BACKWARDS);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_UNCALIBRATED_IMU);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F(StateMachineTest, ClearAllErrors)
{
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
      StateManager::ERROR_TIME_GOING_BACKWARDS |
      StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  rf.state_manager_.clear_error(error);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x00);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F (StateMachineTest, DoNotArmIfError)
{
  // Now add, an error, and then try to arm
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_INVALID_MIXER);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, ArmIfNoError)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F (StateMachineTest, ArmAndDisarm)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F (StateMachineTest, WaitForCalibrationToArm)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  // try to arm
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  // We shouldn't have armed yet
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  EXPECT_EQ(rf.state_manager_.state().error, false);

  // Calibration complete, it should arm now
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, 0x00);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F (StateMachineTest, CalibrationFailedDontArm)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_FAILED);

  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST_F (StateMachineTest, ErrorDuringCalibrationDontArm)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);

  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_INVALID_MIXER);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, RCLostDuringCalibrationDontArm)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);

  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, ClearErrorStayDisarmed)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
  rf.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F (StateMachineTest, RecoverRCStayDisarmed)
{
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F (StateMachineTest, SetErrorsWhileArmed)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  rf.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);

  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, ErrorsPersistWhenDisarmed)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);

  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, UnableToArmWithPersistentErrors)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);

  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, ArmIfThrottleLow)
{
  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = (i > 3) ? 1000 : 1500;
  }
  rc_values[2] = 1000;
  board.set_rc(rc_values);
  step_firmware(rf, board, 100);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(true, rf.state_manager_.state().armed);
}

TEST_F (StateMachineTest, ArmIfThrottleHighWithMinThrottle)
{
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, true);
  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = (i > 3) ? 1000 : 1500;
  }
  rc_values[2] = 1500;
  board.set_rc(rc_values);
  step_firmware(rf, board, 100000);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  step_firmware(rf, board, 1200000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (StateMachineTest, DontArmIfThrottleHighWithoutMinThrottle)
{
  rf.params_.set_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, false);
  uint16_t rc_values[8];
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = (i > 3) ? 1000 : 1500;
  }
  rc_values[2] = 1500;
  board.set_rc(rc_values);
  step_firmware(rf, board, 100000);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  step_firmware(rf, board, 1200000);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (StateMachineTest, LostRCWhenDisarmNoFailsafe)
{
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().error, true);
}

TEST_F (StateMachineTest, UnableToArmWithoutRC)
{
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
}

TEST_F (StateMachineTest, AbleToArmAfterRCRecovery)
{
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
}

TEST_F (StateMachineTest, RCLostWhileArmedEnterFailsafe)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().failsafe, true);
}

TEST_F (StateMachineTest, DisarmWhileInFailsafeGoToError)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  EXPECT_EQ(rf.state_manager_.state().armed, false);
  EXPECT_EQ(rf.state_manager_.state().error, true);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  EXPECT_EQ(rf.state_manager_.state().failsafe, true);
}

TEST_F (StateMachineTest, RegainRCAfterFailsafe)
{
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  EXPECT_EQ(rf.state_manager_.state().armed, true);
  EXPECT_EQ(rf.state_manager_.state().error, false);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  EXPECT_EQ(rf.state_manager_.state().failsafe, false);
}
