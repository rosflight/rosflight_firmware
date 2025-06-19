#include "common.h"
#include "mavlink.h"
#include "test_board.h"
#include "state_manager.h"

#include "rosflight.h"


using namespace rosflight_firmware;

class ROSflightTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;

  ROSflightTest()
    : mavlink(board)
    , rf(board, mavlink)
  {}

  void SetUp() override
  {
    // Initialize firmware with no errors
    board.backup_memory_clear();
    rf.init();
    rf.state_manager_.clear_error(
      rf.state_manager_.state().error_codes); // Clear All Errors to Start
    rf.params_.set_param_int(PARAM_PRIMARY_MIXER, 10);
    rf.params_.set_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, false); // default to turning this off
    rf.params_.set_param_float(PARAM_FAILSAFE_THROTTLE, 0.0f);
  }
};

TEST_F(ROSflightTest, MainLoopSetAndClearTimeGoingBackwards)
{
  // Set initial time and step
  board.set_time(100);
  rf.sensors_.run();
  board.set_time(500);
  rf.run();

  // Step time backward
  board.set_time(100);
  rf.run();

  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);

  // Step time forward
  board.set_time(500);
  rf.run();
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_NONE);
}

TEST_F(ROSflightTest, CheckTimeGoingForward)
{
  board.set_time(10000);
  rf.sensors_.run();
  EXPECT_EQ(rf.sensors_.get_imu()->header.timestamp, 10000);
  rf.check_time_going_forwards();

  board.set_time(1000);
  rf.sensors_.run();
  rf.check_time_going_forwards();

  EXPECT_EQ(rf.sensors_.get_imu()->header.timestamp, 1000);
  EXPECT_EQ(rf.state_manager_.state().error_codes, StateManager::ERROR_TIME_GOING_BACKWARDS);
}
