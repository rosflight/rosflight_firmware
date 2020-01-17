#ifndef CONFIGURATION_ENUM_H
#define CONFIGURATION_ENUM_H

#include <cstdint>

/**
 * Test
 */
namespace rosflight_firmware
{
/**
 * The namespace for Configuration options
 */
namespace Configuration
{

/**
 * @brief An enumeration of configurable devices
 */
enum device_t: uint8_t
{
  SERIAL,
  RC,
  AIRSPEED,
  GNSS,
  SONAR,
  BATTERY_MONITOR,
  BAROMETER,
  MAGNETOMETER,
  DEVICE_COUNT /**<Make sure this is last */
};

constexpr device_t FIRST_DEVICE{static_cast<device_t>(0)};
/**
 * @brief Allows incrementing device_t's for use in for loops. Stops incrementing past DEVICE_COUNT
 */
inline device_t& operator++(device_t &dev)
{
  uint8_t return_value = dev;
  return_value++;
  if (return_value > DEVICE_COUNT)
    return_value--;
  dev = static_cast<device_t>(return_value);
  return dev;
}

}
typedef uint8_t hardware_config_t;
typedef Configuration::device_t device_t; /**< typedef'd for your convenience */
}

#endif // CONFIGURATION_ENUM_H
