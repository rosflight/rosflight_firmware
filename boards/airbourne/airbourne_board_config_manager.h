#ifndef AIRBOURNE_BOARD_CONFIG_MANAGER_H
#define AIRBOURNE_BOARD_CONFIG_MANAGER_H

#include "board_config_manager.h"
#include "airbourne_configuration_enum.h"

namespace rosflight_firmware{
class ROSflight;
class AirbourneBoard;
class AirbourneBoardConfigManager : public BoardConfigManager
{
public:
  AirbourneBoardConfigManager();
  hardware_config_t get_max_config(device_t device) const override;
  ConfigManager::ConfigResponse check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) const override;
  void get_device_name(device_t device, char (&name)[DEVICE_NAME_LENGTH]) const override;
  void get_config_name(device_t device, hardware_config_t config, char (&name)[CONFIG_NAME_LENGTH]) const override;
private:
  static constexpr hardware_config_t max_configs[Configuration::DEVICE_COUNT]{3, 1, 1, 3, 1, 1, 1, 1};

  enum Port
  {
    NO_PORT,
    MAIN_PORT,
    FLEX_IO_PORT,
    FLEXI_PORT,
    USB_PORT,
    POWER_PORT, // the port labeled "PWR / SONAR" is not to provide power, but rather for a battery monitor
    INTERNAL_I2C
  };
  Port get_port(uint8_t device, uint8_t config) const; // Get the port used by a given configuration
};
} // namespace rosflight_firmware

#endif // AIRBOURNE_BOARD_CONFIG_MANAGER_H
