#include "airbourne_board_config_manager.h"

#include <cstring>

#include "rosflight.h"

namespace rosflight_firmware
{
constexpr hardware_config_t AirbourneBoardConfigManager::max_configs[]; // I can't wait for c++ 17 so that this is optional
AirbourneBoardConfigManager::AirbourneBoardConfigManager()
{
}

hardware_config_t AirbourneBoardConfigManager::get_max_config(device_t device) const
{
  if(device >= Configuration::DEVICE_COUNT)
    return 0;
  else
    return AirbourneBoardConfigManager::max_configs[device];
}
ConfigManager::ConfigResponse AirbourneBoardConfigManager::check_config_change(device_t device, hardware_config_t config, const ConfigManager &cm) const
{
  ConfigManager::ConfigResponse resp;
  resp.reboot_required = false;
  resp.successful = false;

  if(device >= Configuration::DEVICE_COUNT)
  {
    strcpy(resp.message, "Device not found");
    return resp;
  }

  if(config > AirbourneBoardConfigManager::max_configs[device])
  {
    strcpy(resp.message, "Configuration not found");
    return resp;
  }
  if(config == cm[device])
  {
    strcpy(resp.message, "Configuration already set. No change required");
    resp.successful = true;
    resp.reboot_required = false;
    return resp;
  }

  Port port = get_port(device, config);
  device_t conflict_device = Configuration::DEVICE_COUNT;

  // While a config may conflict with multiple devices, this will only report one
  switch(device)
  {
  case Configuration::RC:
    if(config ==0) // PPM is not known to conflict with anything
      break;
  case Configuration::SERIAL:
  case Configuration::GNSS:
    if(port != NO_PORT)
      for(device_t other_device{static_cast<device_t>(0)}; other_device != Configuration::DEVICE_COUNT; ++other_device)
      {
        if(other_device == Configuration::RC && cm[Configuration::RC] == 0) // RC over PPM does not conflict with UART, even though both use the same port
          continue;
        if(port == get_port(other_device, cm[other_device]))
        {
          conflict_device = other_device;
          break;
        }
      }
    break;
  case Configuration::AIRSPEED:
  case Configuration::SONAR:
    if(cm[Configuration::GNSS] == 3)
      conflict_device = Configuration::GNSS;
    if(cm[Configuration::SERIAL] == 3)
      conflict_device = Configuration::SERIAL;
    break;
  default:
    break;
  }
  if(conflict_device != Configuration::DEVICE_COUNT)
  {
    switch(conflict_device)
    {
    case Configuration::SERIAL:
      strcpy(resp.message, "Port is used by serial.");
      break;
    case Configuration::RC:
      strcpy(resp.message, "Port is used by RC.");
      break;
    case Configuration::AIRSPEED:
      strcpy(resp.message, "Port is used by airspeed sensor.");
      break;
    case Configuration::GNSS:
      strcpy(resp.message, "Port is used by GNSS receiver.");
      break;
    case Configuration::SONAR:
      strcpy(resp.message, "Port is used by sonar sensor.");
      break;
    // At the time of this writing, the below are incapable of conflicts
    case Configuration::BATTERY_MONITOR:
      strcpy(resp.message, "Port is used by battery monitor.");
      break;
    case Configuration::BAROMETER:
      strcpy(resp.message, "Port is used by barometer.");
      break;
    case Configuration::MAGNETOMETER:
      strcpy(resp.message, "Port is used by magnetometer.");
      break;
    default:
      strcpy(resp.message, "Other error.");
      break;
    }
    return resp;
  }
  resp.successful = true;
  resp.reboot_required = true;
  resp.message[0]=0;
  return resp;
}
void AirbourneBoardConfigManager::get_device_name(device_t device, char (&name)[DEVICE_NAME_LENGTH]) const
{
  switch(device)
  {
  case Configuration::SERIAL:
    strcpy(name, "Serial");
    break;
  case Configuration::RC:
    strcpy(name, "RC");
    break;
  case Configuration::AIRSPEED:
    strcpy(name, "Airspeed");
    break;
  case Configuration::GNSS:
    strcpy(name, "GNSS");
    break;
  case Configuration::SONAR:
    strcpy(name, "Sonar");
    break;
  case Configuration::BATTERY_MONITOR:
    strcpy(name, "Battery Monitor");
    break;
  case Configuration::BAROMETER:
    strcpy(name, "Barometer");
    break;
  case Configuration::MAGNETOMETER:
    strcpy(name, "Magnetometer");
    break;
  default:
    strcpy(name, "Error/Unsupported");
    break;
  }
}
void AirbourneBoardConfigManager::get_config_name(device_t device, hardware_config_t config, char (&name)[CONFIG_NAME_LENGTH]) const
{
  if(config > AirbourneBoardConfigManager::max_configs[device])
    strcpy(name, "Invalid config");
  else
    switch(device)
    {
    case Configuration::SERIAL:
      switch(config)
      {
      case 0:
        strcpy(name, "VCP over USB");
        break;
      case 1:
        strcpy(name, "UART1 on Main");
        break;
      case 2:
        strcpy(name, "UART2 on Flex-IO");
        break;
      case 3:
        strcpy(name, "UART3 on Flexi");
        break;
      }
      break;
    case Configuration::RC:
      if(config==0)
        strcpy(name, "PPM on Flex-IO");
      else
        strcpy(name, "SBUS on Main");
      break;
    case Configuration::AIRSPEED:
      if(config==0)
        strcpy(name, "Disabled");
      else
        strcpy(name, "I2C2 on Flexi");
      break;
    case Configuration::GNSS:
      switch(config)
      {
      case 0:
        strcpy(name, "Disabled");
        break;
      case 1:
        strcpy(name, "UART1 on main");
        break;
      case 2:
        strcpy(name, "UART2 on Flex-Io");
        break;
      case 3:
        strcpy(name, "UART3 on Flexi");
        break;
      }
      break;
    case Configuration::SONAR:
      if(config ==0)
        strcpy(name, "Disabled");
      else
        strcpy(name, "I2C2 on Flexi");
      break;
    case Configuration::BATTERY_MONITOR:
      if(config==0)
        strcpy(name, "Disabled");
      else
        strcpy(name, "ADC3 on Power");
      break;
    case Configuration::BAROMETER:
      if(config==0)
        strcpy(name, "Disabled");
      else
        strcpy(name, "Onboard barometer");
      break;
    case Configuration::MAGNETOMETER:
      if(config ==0)
        strcpy(name, "Disabled");
      else
        strcpy(name, "Onboard magnetometer");
      break;
    default:
      strcpy(name, "Invalid device");
    }
}
AirbourneBoardConfigManager::Port AirbourneBoardConfigManager::get_port(uint8_t device, uint8_t config) const
{
  switch(device)
  {
  case Configuration::SERIAL:
    switch(config)
    {
    case 0:
      return USB_PORT;
    case 1:
      return MAIN_PORT;
    case 2:
      return FLEX_IO_PORT;
    case 3:
      return FLEXI_PORT;
    }
  case Configuration::GNSS:
    switch(config)
    {
    case 0:
      return USB_PORT;
    case 1:
      return MAIN_PORT;
    case 2:
      return FLEX_IO_PORT;
    case 3:
      return FLEXI_PORT;
    }
  case Configuration::RC:
    if(config == 0)
      return FLEX_IO_PORT;
    if(config == 1)
      return MAIN_PORT;
    break;
  case Configuration::AIRSPEED:
  case Configuration::SONAR:
    if(config==1)
      return FLEXI_PORT;
    break;
  case Configuration::BATTERY_MONITOR:
    if(config == 1)
      return POWER_PORT;
    break;
  case Configuration::MAGNETOMETER:
  case Configuration::BAROMETER:
    if(config == 1)
      return INTERNAL_I2C;
  }
  return NO_PORT;
}
} //rosflight_firmware
