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
    hardware_config_t config[Configuration::DEVICE_COUNT];
  } config_t;

  static const int CONFIG_RESPONSE_MESSAGE_LENGTH{50};
  typedef struct
  {
    bool successful;
    bool reboot_required;
    char message[CONFIG_RESPONSE_MESSAGE_LENGTH]; // Primarily for error messages
  }config_response;

  ConfigManager(ROSflight &RF, config_t &config);
  // Reads the memory, and loads defaults if it is invalid. Call after the memory manager is ready
  bool init();
  bool configure_devices() const; // Sends configurations to the board via the enable_device method
  // Attempts to set a configuration, failing if the board config manager rejects it
  config_response attempt_set_configuration(device_t device, uint8_t config);
  // Sets a config without checks. This may cause an invalid configuration combo,
  // so attempt_set_configuration is recommended
  void set_configuration(device_t device, uint8_t config);
  uint8_t get_configuration(device_t device) const;
  uint8_t operator[](device_t device) const; // same as get_configuration, for convenience

  void prepare_write(); // prepares a checksum, so that the config struct can be saved

private:
  ROSflight &RF_;
  config_t &config_;
  bool read(); // currently just checks that the memory manager is ready and the checksum is correct
  void fill_defaults(); // Default values are 0, by convention
  uint32_t generate_checksum() const; // Based off of fletcher algorithm

};
}
#endif // HARDWARE_CONFIG_H
