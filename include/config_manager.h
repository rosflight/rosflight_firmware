#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "rosflight.h"
#include "configuration_enum.h"

namespace rosflight_firmware
{
class ConfigManager
{
public:
  ConfigManager(ROSflight &rf);
  bool init();
  enum: uint8_t
  {
    CONFIG_SERIAL,
    CONFIG_RC,
    CONFIG_AIRSPEED,
    CONFIG_GNSS,
    CONFIG_SONAR,
    CONFIG_BATTERY_MONITOR,
    CONFIG_COUNT
  };
  typedef struct
  {
    uint8_t config[CONFIG_COUNT];
  } config_t;

private:
  ROSflight *rf_;

};
}
#endif // HARDWARE_CONFIG_H
