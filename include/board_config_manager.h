#ifndef BOARD_CONFIG_MANAGER_H
#define BOARD_CONFIG_MANAGER_H

#include "configuration_enum.h"

namespace rosflight_firmware
{
class BoardConfigManager
{
public:
  typedef struct
  {
    bool successful;
    bool reboot_required;
    char error_message[50];
  }config_response;

  // Get the largest number that is valid for the configuration of a given device
  virtual hardware_config_t get_max_config(device_t device) = 0;
  // Check if a config change is allowed.
  // If the response indicates success, then the config manager accepts the change
  // If not, the config manager returns the error indicated
  virtual config_response check_config_change(device_t device, hardware_config_t config) = 0;
  virtual void get_device_name(device_t device, char (&name)[20])=0;
  virtual void get_config_name(device_t device, hardware_config_t config, char (&name)[20]) = 0;
};
} // namespace rosflight_firmware

#endif // BOARD_CONFIG_MANAGER_H
