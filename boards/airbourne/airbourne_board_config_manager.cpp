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
    if(config ==AirbourneConfiguration::RC_PPM) // PPM is not known to conflict with anything
      break;
    [[gnu::fallthrough]];
  case Configuration::SERIAL:
  case Configuration::GNSS:
    if(port != NO_PORT)
      for(device_t other_device{Configuration::FIRST_DEVICE}; other_device != Configuration::DEVICE_COUNT; ++other_device)
      {
        if(other_device == Configuration::RC && cm[Configuration::RC] == AirbourneConfiguration::RC_PPM) // RC over PPM does not conflict with UART, even though both use the same port
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
    if(cm[Configuration::GNSS] == AirbourneConfiguration::GNSS_UART3)
      conflict_device = Configuration::GNSS;
    if(cm[Configuration::SERIAL] == AirbourneConfiguration::GNSS_UART3)
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
  resp.message[0]=0; // Ensuring that the message is treated as a zero-length string
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
    strcpy(name, "Baro");
    break;
  case Configuration::MAGNETOMETER:
    strcpy(name, "Mag");
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
      case AirbourneConfiguration::SERIAL_VCP:
        strcpy(name, "VCP over USB");
        break;
      case AirbourneConfiguration::SERIAL_UART1:
        strcpy(name, "UART1 on Main");
        break;
      case AirbourneConfiguration::SERIAL_UART2:
        strcpy(name, "UART2 on Flex-IO");
        break;
      case AirbourneConfiguration::SERIAL_UART3:
        strcpy(name, "UART3 on Flexi");
        break;
      }
      break;
    case Configuration::RC:
      if(config==AirbourneConfiguration::RC_PPM)
        strcpy(name, "PPM on Flex-IO");
      else if(config==AirbourneConfiguration::RC_SBUS)
        strcpy(name, "SBUS on Main");
      break;
    case Configuration::AIRSPEED:
      if(config==AirbourneConfiguration::AIRSPEED_DISABLED)
        strcpy(name, "Disabled");
      else if(config == AirbourneConfiguration::AIRSPEED_I2C2)
        strcpy(name, "I2C2 on Flexi");
      break;
    case Configuration::GNSS:
      switch(config)
      {
      case AirbourneConfiguration::GNSS_DISABLED:
        strcpy(name, "Disabled");
        break;
      case AirbourneConfiguration::GNSS_UART1:
        strcpy(name, "UART1 on main");
        break;
      case AirbourneConfiguration::GNSS_UART2:
        strcpy(name, "UART2 on Flex-Io");
        break;
      case AirbourneConfiguration::GNSS_UART3:
        strcpy(name, "UART3 on Flexi");
        break;
      }
      break;
    case Configuration::SONAR:
      if(config ==AirbourneConfiguration::SONAR_DISABLED)
        strcpy(name, "Disabled");
      else if(config == AirbourneConfiguration::SONAR_I2C2)
        strcpy(name, "I2C2 on Flexi");
      break;
    case Configuration::BATTERY_MONITOR:
      if(config==AirbourneConfiguration::BATTERY_MONITOR_DISABLED)
        strcpy(name, "Disabled");
      else if(config==AirbourneConfiguration::BATTERY_MONITOR_ADC3)
        strcpy(name, "ADC3 on Power");
      break;
    case Configuration::BAROMETER:
      if(config==AirbourneConfiguration::BAROMETER_DISABLED)
        strcpy(name, "Disabled");
      else if(config == AirbourneConfiguration::BAROMETER_ONBOARD)
        strcpy(name, "Onboard baro");
      break;
    case Configuration::MAGNETOMETER:
      if(config ==AirbourneConfiguration::MAGNETOMETER_DISABLED)
        strcpy(name, "Disabled");
      else if(config == AirbourneConfiguration::MAGNETOMETER_ONBOARD)
        strcpy(name, "Onboard mag");
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
    case AirbourneConfiguration::SERIAL_VCP:
      return USB_PORT;
    case AirbourneConfiguration::SERIAL_UART1:
      return MAIN_PORT;
    case AirbourneConfiguration::SERIAL_UART2:
      return FLEX_IO_PORT;
    case AirbourneConfiguration::SERIAL_UART3:
      return FLEXI_PORT;
    }
  case Configuration::GNSS:
    switch(config)
    {
    case AirbourneConfiguration::GNSS_DISABLED:
      return NO_PORT;
    case AirbourneConfiguration::GNSS_UART1:
      return MAIN_PORT;
    case AirbourneConfiguration::GNSS_UART2:
      return FLEX_IO_PORT;
    case AirbourneConfiguration::GNSS_UART3:
      return FLEXI_PORT;
    }
  case Configuration::RC:
    if(config == AirbourneConfiguration::RC_PPM)
      return FLEX_IO_PORT;
    if(config == AirbourneConfiguration::RC_SBUS)
      return MAIN_PORT;
    break;
  case Configuration::AIRSPEED:
    if(config==AirbourneConfiguration::AIRSPEED_I2C2)
      return FLEXI_PORT;
    break;
  case Configuration::SONAR:
    if(config==AirbourneConfiguration::SONAR_I2C2)
      return FLEXI_PORT;
    break;
  case Configuration::BATTERY_MONITOR:
    if(config == AirbourneConfiguration::BATTERY_MONITOR_ADC3)
      return POWER_PORT;
    break;
  case Configuration::MAGNETOMETER:
    if(config == AirbourneConfiguration::MAGNETOMETER_ONBOARD)
      return INTERNAL_I2C;
    break;
  case Configuration::BAROMETER:
    if(config == AirbourneConfiguration::BAROMETER_ONBOARD)
      return INTERNAL_I2C;
  }
  return NO_PORT;
}
} //rosflight_firmware
