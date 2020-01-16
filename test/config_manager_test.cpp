#include <gtest/gtest.h>
#include <string>

#include "rosflight.h"
#include "mavlink.h"
#include "test_board.h"
#include "configuration_enum.h"
#include "config_manager.h"

using namespace rosflight_firmware;

class ConfigManagerTest : public ::testing::Test
{
public:
  testBoard board;
  Mavlink mavlink;
  ROSflight rf;

  ConfigManagerTest() :
    mavlink(board),
    rf(board, mavlink) {}
  void SetUp() override
  {
    rf.init();
  }
};

TEST_F(ConfigManagerTest, DefaultValues)
{
  for(device_t device{Configuration::FIRST_DEVICE}; device < Configuration::DEVICE_COUNT; ++device)
    EXPECT_EQ(rf.config_manager_[device], 0);
}

TEST_F(ConfigManagerTest, SetValid)
{
  device_t changed_device = Configuration::SERIAL;
  hardware_config_t config = 27;
  ConfigManager::ConfigResponse response = rf.config_manager_.attempt_set_configuration(changed_device, config);
  EXPECT_TRUE(response.successful);
  EXPECT_TRUE(response.reboot_required);
  EXPECT_EQ(std::string(reinterpret_cast<char*>(response.message)), "Succeed for testing");
  for(device_t device{Configuration::FIRST_DEVICE}; device < Configuration::DEVICE_COUNT; ++device)
    if(device == changed_device)
      EXPECT_EQ(rf.config_manager_[device], config);
    else
      EXPECT_EQ(rf.config_manager_[device], 0);
}

TEST_F(ConfigManagerTest, SetInvalid)
{
  device_t changed_device = Configuration::SERIAL;
  hardware_config_t config = 1;
  ConfigManager::ConfigResponse response = rf.config_manager_.attempt_set_configuration(changed_device, config);
  EXPECT_FALSE(response.successful);
  EXPECT_FALSE(response.reboot_required);
  EXPECT_EQ(std::string(reinterpret_cast<char*>(response.message)), "Fail for testing");
  for(device_t device{Configuration::FIRST_DEVICE}; device < Configuration::DEVICE_COUNT; ++device)
    EXPECT_EQ(rf.config_manager_[device], 0);
}
