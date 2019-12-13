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

/*
enum resource_t
{
  vcp,
  uart,
  sbus,
  i2c,
  spi,
  ppm,
  adc
};
*/

typedef uint8_t hardware_config_t;


#endif // CONFIGURATION_ENUM_H
