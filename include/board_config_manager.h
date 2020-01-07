#ifndef BOARD_CONFIG_MANAGER_H
#define BOARD_CONFIG_MANAGER_H

#include "configuration_enum.h"
#include "config_manager.h"

namespace rosflight_firmware
{
class BoardConfigManager
{
public:

  // Get the largest number that is valid for the configuration of a given device
  // This number is inclusive, i.e. a value of 2 means 0, 1, or 2 are valid
  // For devices that are not configurable, returns 0
  virtual hardware_config_t get_max_config(device_t device) = 0;
  // Check if a config change is allowed.
  // If the response indicates success, then the config manager accepts the change
  // If not, the config manager returns the error indicated
  virtual ConfigManager::config_response check_config_change(device_t device, hardware_config_t config) = 0;
  virtual void get_device_name(device_t device, uint8_t (&name)[20])=0;
  virtual void get_config_name(device_t device, hardware_config_t config, uint8_t (&name)[20]) = 0;
};
} // namespace rosflight_firmware

#endif // BOARD_CONFIG_MANAGER_H
