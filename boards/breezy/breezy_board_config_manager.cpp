#include "breezy_board_config_manager.h"
#include <cstring>

namespace rosflight_firmware
{
hardware_config_t BreezyBoardConfigManager::get_max_config(device_t device)
{
  (void)device;
  return 0;
}

ConfigManager::config_response BreezyBoardConfigManager::check_config_change(device_t device, hardware_config_t config)
{
  (void)device;
  (void)config;
  ConfigManager::config_response response;
  response.successful = false;
  response.reboot_required = false;
  strcpy(reinterpret_cast<char*>(response.message), "Feature unsupported on naze");
  return response;
}

void BreezyBoardConfigManager::get_device_name(device_t device, uint8_t (&name)[20])
{
  (void)device;
  strcpy(reinterpret_cast<char*>(name), "Unsupported");
}

void BreezyBoardConfigManager::get_config_name(device_t device, hardware_config_t config, uint8_t (&name)[20])
{
  (void)device;
  (void)config;
  strcpy(reinterpret_cast<char*>(name), "Unsupported");
}

} // namespace rosflight_firmware
