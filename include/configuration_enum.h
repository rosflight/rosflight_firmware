#ifndef CONFIGURATION_ENUM_H
#define CONFIGURATION_ENUM_H

#include <cstdint>

enum device_t: uint8_t
{
  serial,
  rc,
  airspeed,
  gnss,
  sonar,
  battery_monitor,
  barometer,
  magnetometer,
  device_count // make sure this is last
};

inline device_t& operator++(device_t &dev)
{
  uint8_t return_value = dev;
  return_value++;
  if (return_value > device_count)
    return_value--;
  dev = static_cast<device_t>(return_value);
  return dev;
}
typedef uint8_t hardware_config_t;


#endif // CONFIGURATION_ENUM_H
