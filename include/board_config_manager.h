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
  virtual hardware_config_t get_max_config(device_t device) const = 0;
  // Check if a config change is allowed.
  // If the response indicates success, then the config manager accepts the change
  // If not, the config manager returns the error indicated
  // Implementations should not assume that either the device or the config are valid
  virtual ConfigManager::ConfigResponse check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) const = 0;
  static constexpr int DEVICE_NAME_LENGTH = 20;
  static constexpr int CONFIG_NAME_LENGTH = 20;
  // Do not assume that the device or config are valid numbers
  // When passed an invalid argument, it is better to return a string so indicating than an
  // empty string
  virtual void get_device_name(device_t device, char (&name)[DEVICE_NAME_LENGTH]) const=0;
  virtual void get_config_name(device_t device, hardware_config_t config, char (&name)[CONFIG_NAME_LENGTH]) const = 0;
};
} // namespace rosflight_firmware

#endif // BOARD_CONFIG_MANAGER_H
