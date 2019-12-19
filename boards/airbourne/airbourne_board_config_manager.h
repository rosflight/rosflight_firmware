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
  void init(ROSflight *rf, AirbourneBoard *board);
  hardware_config_t get_max_config(device_t device) override;
  config_response check_config_change(device_t device, hardware_config_t config) override;
  void get_device_name(device_t device, uint8_t (&name)[20]) override;
  void get_config_name(device_t device, hardware_config_t config, uint8_t (&name)[20]) override;
  inline bool is_initialized(){return is_initialized_;}
private:
  enum revo_port
  {
    none,
    main,
    flex_io,
    flexi,
    usb,
    power
  };
  revo_port get_port(uint8_t device, uint8_t config);
  bool is_initialized_{false};
  AirbourneBoard *board_;
  ROSflight *RF_;
  const hardware_config_t max_configs[device_count]{3, 1, 1, 3, 1, 1, 0, 0};
};
} // namespace rosflight_firmware

#endif // AIRBOURNEBOARDCONFIGMANAGER_H
