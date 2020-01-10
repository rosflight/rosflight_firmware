#ifndef AIRBOURNEBOARDCONFIGMANAGER_H
#define AIRBOURNEBOARDCONFIGMANAGER_H

#include "board_config_manager.h"

namespace rosflight_firmware{
class ROSflight;
class AirbourneBoard;
class AirbourneBoardConfigManager : public BoardConfigManager
{
public:
  AirbourneBoardConfigManager();
  hardware_config_t get_max_config(device_t device) override;
  ConfigManager::config_response check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) override;
  void get_device_name(device_t device, char (&name)[DEVICE_NAME_LENGTH]) override;
  void get_config_name(device_t device, hardware_config_t config, char (&name)[CONFIG_NAME_LENGTH]) override;
private:
  enum revo_port
  {
    NO_PORT,
    MAIN_PORT,
    FLEX_IO_PORT,
    FLEXI_PORT,
    USB_PORT,
    POWER_PORT, // the port labeled "PWR / SONAR" is not to provide power, but rather for a battery monitor
    INTERNAL_I2C
  };
  revo_port get_port(uint8_t device, uint8_t config); // Get the port used by a given configuration
  ROSflight *RF_;
  const hardware_config_t max_configs[Configuration::DEVICE_COUNT]{3, 1, 1, 3, 1, 1, 1, 1};
};
} // namespace rosflight_firmware

#endif // AIRBOURNEBOARDCONFIGMANAGER_H
