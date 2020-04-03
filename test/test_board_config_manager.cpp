#include "test_board_config_manager.h"

#include <cstring>
#include <string>

namespace rosflight_firmware
{
hardware_config_t TestBoardConfigManager::get_max_config(device_t device) const
{
  (void)device;
  return 0; // This is not needed to test other software
}
ConfigManager::ConfigResponse TestBoardConfigManager::check_config_change(device_t device,
                                                                          hardware_config_t config,
                                                                          const ConfigManager &cm) const
{
  (void)cm;
  // A couple variations are given for testing
  ConfigManager::ConfigResponse response;
  response.successful = true;
  response.reboot_required = true;
  if (device == Configuration::SERIAL && config == 1)
  {
    response.successful = false;
    response.reboot_required = false;
    strcpy(reinterpret_cast<char *>(response.message), "Fail for testing");
    return response;
  }
  strcpy(reinterpret_cast<char *>(response.message), "Succeed for testing");
  return response;
}
void TestBoardConfigManager::get_device_name(device_t device,
                                             char (&name)[BoardConfigManager::DEVICE_NAME_LENGTH]) const
{
  std::string device_name = "device #" + std::to_string(static_cast<int>(device));
  strcpy(reinterpret_cast<char *>(name), device_name.c_str());
}
void TestBoardConfigManager::get_config_name(device_t device,
                                             hardware_config_t config,
                                             char (&name)[BoardConfigManager::CONFIG_NAME_LENGTH]) const
{
  std::string config_name =
      "config " + std::to_string(static_cast<int>(device)) + "," + std::to_string(static_cast<int>(config));
  strcpy(reinterpret_cast<char *>(name), config_name.c_str());
}
} // namespace rosflight_firmware
