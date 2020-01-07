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

  typedef struct
  {
    bool successful;
    bool reboot_required;
    uint8_t error_message[50];
  }config_response;

  ConfigManager(ROSflight &RF, config_t &config);
  bool init();
  bool configure_devices();
  // Attempts to set a configuration, failing if the board config manager rejects it
  config_response attempt_set_configuration(device_t device, uint8_t config);
  void set_configuration(device_t device, uint8_t config); // Directly sets a configuration
  uint8_t get_configuration(device_t device);
  uint8_t operator[](device_t device); // same as get_configuration, for convenience

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
