#ifndef CONFIGURATION_ENUM_H
#define CONFIGURATION_ENUM_H

#include <cstdint>

namespace Configuration
{

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
  DEVICE_COUNT // make sure this is last
};

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
typedef Configuration::device_t device_t;

#endif // CONFIGURATION_ENUM_H
