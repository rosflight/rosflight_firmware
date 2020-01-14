#include "breezy_board_config_manager.h"
#include <cstring>

namespace rosflight_firmware
{
hardware_config_t BreezyBoardConfigManager::get_max_config(device_t device) const
{
  (void)device;
  return 0;
}

ConfigManager::ConfigResponse BreezyBoardConfigManager::check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) const
{
  (void)device;
  (void)config;
  (void)cm;
  ConfigManager::ConfigResponse response;
  response.successful = false;
  response.reboot_required = false;
  strcpy(reinterpret_cast<char*>(response.message), "Feature unsupported on naze");
  return response;
}

void BreezyBoardConfigManager::get_device_name(device_t device, char (&name)[BoardConfigManager::DEVICE_NAME_LENGTH]) const
{
  (void)device;
  strcpy(name, "Unsupported");
}

void BreezyBoardConfigManager::get_config_name(device_t device, hardware_config_t config, char (&name)[BoardConfigManager::CONFIG_NAME_LENGTH]) const
{
  (void)device;
  (void)config;
  strcpy(name, "Unsupported");
}

} // namespace rosflight_firmware
