#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "configuration_enum.h"

namespace rosflight_firmware
{
class ROSflight;
class ConfigManager
{
public:
  typedef struct __attribute__ ((packed))
  {
    uint32_t checksum;
    hardware_config_t config[device_t::device_count];
  } config_t;

  ConfigManager(ROSflight &RF, config_t &config);
  bool init();
  bool configure_devices();
  void set_configuration(device_t device, uint8_t config);
  uint8_t get_configuration(device_t device);
  void prepare_write();

private:
  ROSflight &RF_;
  config_t &config_;
  bool read();
  void fill_defaults();
  uint32_t generate_checksum();

};
}
#endif // HARDWARE_CONFIG_H
