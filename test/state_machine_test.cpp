#include "common.h"

#include "rosflight.h"
#include "mavlink.h"
#include "test_board.h"

using namespace rosflight_firmware;

TEST(state_machine_test, error_check) {

  // Initialize the full firmware, so that the state_manager can do its thing
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Initialize just a subset of the modules
  // (some modules set errors when they initialize)
  rf.board_.init_board();
  rf.state_manager_.init();
  rf.params_.init();

  // Should be in PREFLIGHT MODE
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  // Try setting and clearing all the errors
  for (int error = 0x0001; error <= StateManager::ERROR_UNCALIBRATED_IMU; error *= 2)
  {
    // set the error
    rf.state_manager_.set_error(error);
    ASSERT_EQ(rf.state_manager_.state().armed, false);
    ASSERT_EQ(rf.state_manager_.state().failsafe, false);
    ASSERT_EQ(rf.state_manager_.state().error_codes, error);
    ASSERT_EQ(rf.state_manager_.state().error, true);

    // clear the error
    rf.state_manager_.clear_error(error);
    ASSERT_EQ(rf.state_manager_.state().armed, false);
    ASSERT_EQ(rf.state_manager_.state().failsafe, false);
    ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
    ASSERT_EQ(rf.state_manager_.state().error, false);
  }

  // Try combinations of errors
  uint32_t error = StateManager::ERROR_IMU_NOT_RESPONDING |
                   StateManager::ERROR_TIME_GOING_BACKWARDS |
                   StateManager::ERROR_UNCALIBRATED_IMU;
  rf.state_manager_.set_error(error);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x32);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Add another error
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x33);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Clear an error
  rf.state_manager_.clear_error(StateManager::ERROR_UNCALIBRATED_IMU);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x13);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Clear two errors
  rf.state_manager_.clear_error(StateManager::ERROR_IMU_NOT_RESPONDING |
                                StateManager::ERROR_TIME_GOING_BACKWARDS);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x01);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Clear final error
  rf.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);
}

TEST(state_machine_test, arm_check) {
  // Build the full firmware, so that the state_manager can do its thing
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Initialize just a subset of the modules
  // (some modules set errors when they initialize)
  rf.board_.init_board();
  rf.state_manager_.init();
  rf.params_.init();

  // Should be in PREFLIGHT MODE
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  //======================================================
  // Basic arming test
  //======================================================

  // Make sure that the parameter to calibrate before arm is off
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false);

  // Now add, an error, and then try to arm
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_INVALID_MIXER);
  ASSERT_EQ(rf.state_manager_.state().error, true);


  // Clear the error, and then try again to arm
  rf.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  // Disarm
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  ASSERT_EQ(rf.state_manager_.state().error, false);


  //======================================================
  // Preflight calibration arming test
  //======================================================
  // turn preflight calibration on
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);

  // try to arm
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  // We shouldn't have armed yet
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  // Set an error
  rf.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  // We shouldn't have armed yet
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_INVALID_MIXER);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Clear the error, and try again
  rf.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  // Tell it that the calibration failed
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_FAILED);
  // We shouldn't have armed yet
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  // Try again, but this time the calibration succeeds
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
  // We should be armed
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  //======================================================
  // Errors while armed test
  //======================================================
  // turn preflight calibration off
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false);

  // While armed, let's set some errors
  rf.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
  // We should still be armed, but have an error
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // What happens if we disarm now?
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Try to arm, it should fail
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Clear the error and try again
  rf.state_manager_.clear_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
}

TEST(state_machine_test, failsafe_check) {
  // Build the full firmware, so that the state_manager can do its thing
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Initialize just a subset of the modules
  // (some modules set errors when they initialize)
  rf.board_.init_board();
  rf.state_manager_.init();
  rf.params_.init();

  // Should be in PREFLIGHT MODE
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  //======================================================
  // RC Lost when disarmed - Should not be in failsafe, but in error
  //======================================================
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Try to arm
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);

  // Oh look!  we have RC again
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  //======================================================
  // RC Lost when armed - should enter failsafe
  //======================================================

  // Let's fly!
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, true);

  // Oh crap, we lost RC in the air
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, true);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // If by some magic we are able to disarm (perhaps via the computer)
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, true);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Let's regain RC
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  // Let's try this again!
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, true);

  // This is the worst receiver
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, true);

  // It's going in and out!
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);
}

TEST(state_machine_test, corner_cases) {
  // Build the full firmware, so that the state_manager can do its thing
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  // Initialize just a subset of the modules
  // (some modules set errors when they initialize)
  rf.board_.init_board();
  rf.state_manager_.init();
  rf.params_.init();

  // Should be in PREFLIGHT MODE
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  //======================================================
  // RC Lost when calibrating
  //======================================================

  // turn preflight calibration on
  rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, true);

  // Try to arm
  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);

  // Lose RC during calibration - error, no failsafe
  rf.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_RC_LOST);
  ASSERT_EQ(rf.state_manager_.state().error, true);

  // Regain RC, should be in preflight mode, no error
  rf.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  ASSERT_EQ(rf.state_manager_.state().failsafe, false);
  ASSERT_EQ(rf.state_manager_.state().error_codes, 0x00);
  ASSERT_EQ(rf.state_manager_.state().error, false);

  rf.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
  ASSERT_EQ(rf.state_manager_.state().armed, false);
  rf.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
  ASSERT_EQ(rf.state_manager_.state().armed, true);
}
