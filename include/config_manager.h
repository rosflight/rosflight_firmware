#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "configuration_enum.h"

namespace rosflight_firmware
{
class ROSflight;
/**
 * @brief A class for managing the configuration of various devices.
 * @details Devices include the serial connection, %RC, and sensors. Devices are represented by
 * @ref device_t and configurations by @ref hardware_config_t
 * @sa Configuration::device_t
 */
class ConfigManager
{
public:
  struct __attribute__((packed)) Config
  {
    uint32_t checksum;
    hardware_config_t config[Configuration::DEVICE_COUNT];
  };

  static constexpr int CONFIG_RESPONSE_MESSAGE_LENGTH = 50;
  /**
   * @brief A struct to hold responses to attempts to change configurations
   */
  struct ConfigResponse
  {
    bool successful;                              /**< If the change was successfully made **/
    bool reboot_required;                         /**< If a reboot is required for the change to take effect */
    char message[CONFIG_RESPONSE_MESSAGE_LENGTH]; /**< An optional message, often an error message */
  };

  ConfigManager(ROSflight &RF, Config &config);
  /**
   * @brief Reads from memory, and loads defaults if invalid.
   * @details By convention, all default values are 0.
   * @pre Memory manager is initialized
   * @return if the initialization suceeded
   */
  bool init();
  /**
   * @brief Sends configurations to the board via the enable_device method
   * @pre The config manager is initialized
   * @return if all devices were configured successfully
   */
  bool configure_devices() const;
  /**
   * @brief Attempts to set a configuration, failing if the board config manager rejects it.
   * If the board config manager does not reject the change, the change is made in the config manager.
   * This does not save the change, and currently does not change any configuration until reboot.
   * @param device Any device
   * @param config The new configuration for the device
   * @return A response, indicating if the change was successful, if a reboot is required for the
   *         change to take effect, and optionally a message (often an error message)
   */
  ConfigResponse attempt_set_configuration(device_t device, uint8_t config);
  /**
   * @brief Get the current configuration for a device.
   * This does not necessarily reflect how hardware is currently loaded, as changes via
   * attempt_set_configuration may require a reboot to take effect, but not to be reflected here.
   * @param device Any device
   * @return The current configuration for the device
   */
  uint8_t get_configuration(device_t device) const;
  /**
   * @brief Get the current configuration for a device. Alias for ConfigManager::get_configuration
   * @see ConfigManager::get_configuration
   */
  uint8_t operator[](device_t device) const;

  /**
   * @brief Prepares the checksum, so that the config struct can be saved to non-volatile memory
   */
  void prepare_write(); // prepares a checksum, so that the config struct can be saved

private:
  ROSflight &RF_;
  Config &config_;
  // Sets a config without checks. This may cause an invalid configuration combo,
  // so attempt_set_configuration is recommended
  void set_configuration(device_t device, uint8_t config);
  bool read();          // currently just checks that the memory manager is ready and the checksum is correct
  void fill_defaults(); // Default values are 0, by convention
  uint32_t generate_checksum() const; // Based off of fletcher algorithm
};
} // namespace rosflight_firmware
#endif // HARDWARE_CONFIG_H
