#ifndef TESTBOARDCONFIGMANAGER_H
#define TESTBOARDCONFIGMANAGER_H

#include "board_config_manager.h"

namespace rosflight_firmware
{
class TestBoardConfigManager : public BoardConfigManager
{
public:
  hardware_config_t get_max_config(device_t device) const override;
  ConfigManager::ConfigResponse check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) const override;
  void get_device_name(device_t device, char (&name)[BoardConfigManager::DEVICE_NAME_LENGTH]) const override;
  void get_config_name(device_t device, hardware_config_t config, char (&name)[BoardConfigManager::CONFIG_NAME_LENGTH]) const override;
};

} // rosflight_firmware
#endif // TESTBOARDCONFIGMANAGER_H
