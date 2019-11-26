#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "rosflight.h"
#include "configuration_enum.h"

namespace rosflight_firmware
{
class DeviceManager
{
public:
  DeviceManager(ROSflight &rf);
  bool init();
private:
  ROSflight *rf_;

};
}
#endif // HARDWARE_CONFIG_H
