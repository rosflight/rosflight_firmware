#include "airbourne_board_config_manager.h"

#include <cstring>

#include "rosflight.h"

namespace rosflight_firmware
{
AirbourneBoardConfigManager::AirbourneBoardConfigManager()
{
}
void AirbourneBoardConfigManager::init(ROSflight *rf, AirbourneBoard *board)
{
  this->RF_ = rf;
  this->board_ = board;
  is_initialized_ = true;
}
hardware_config_t AirbourneBoardConfigManager::get_max_config(device_t device)
{
  if(device >= Configuration::DEVICE_COUNT)
    return 0;
  else
    return max_configs[device];
}
ConfigManager::config_response AirbourneBoardConfigManager::check_config_change(device_t device, hardware_config_t config)
{
  ConfigManager::config_response resp;
  ConfigManager &cm = RF_->config_manager_;
  resp.reboot_required = false;
  resp.successful = false;
  char *error_message = reinterpret_cast<char*>(resp.message); // Because it wasn't converting implicitly

  if(device >= Configuration::DEVICE_COUNT)
  {
    strcpy(error_message, "Device not found");
    return resp;
  }

  if(config > max_configs[device])
  {
    strcpy(error_message, "Configuration not found");
    return resp;
  }
  if(config == cm[device])
  {
    strcpy(error_message, "Configuration already set. No change required");
    resp.successful = true;
    resp.reboot_required = false;
    return resp;
  }

  revo_port port = get_port(device, config);
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
        if(port == get_port(other_device, cm[other_device]) && (other_device !=Configuration::RC || cm[Configuration::RC]!=0)) // RC over PPM does not conflict with UART, even though both use the same port
        {
          conflict_device = other_device;
          break;
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
      strcpy(error_message, "Port is used by serial.");
      break;
    case Configuration::RC:
      strcpy(error_message, "Port is used by RC.");
      break;
    case Configuration::AIRSPEED:
      strcpy(error_message, "Port is used by airspeed sensor.");
      break;
    case Configuration::GNSS:
      strcpy(error_message, "Port is used by GNSS receiver.");
      break;
    case Configuration::SONAR:
      strcpy(error_message, "Port is used by sonar sensor.");
      break;
    // At the time of this writing, the below are incapable of conflicts
    case Configuration::BATTERY_MONITOR:
      strcpy(error_message, "Port is used by battery monitor.");
      break;
    case Configuration::BAROMETER:
      strcpy(error_message, "Port is used by barometer.");
      break;
    case Configuration::MAGNETOMETER:
      strcpy(error_message, "Port is used by magnetometer.");
      break;
    default:
      strcpy(error_message, "Other error.");
      break;
    }
    return resp;
  }
  resp.successful = true;
  resp.reboot_required = true;
  resp.message[0]=0;
  return resp;
}
void AirbourneBoardConfigManager::get_device_name(device_t device, uint8_t (&name)[DEVICE_NAME_LENGTH])
{
  char *name_char = reinterpret_cast<char*>(name);
  switch(device)
  {
  case Configuration::SERIAL:
    strcpy(name_char, "Serial");
    break;
  case Configuration::RC:
    strcpy(name_char, "RC");
    break;
  case Configuration::AIRSPEED:
    strcpy(name_char, "Airspeed");
    break;
  case Configuration::GNSS:
    strcpy(name_char, "GNSS");
    break;
  case Configuration::SONAR:
    strcpy(name_char, "Sonar");
    break;
  case Configuration::BATTERY_MONITOR:
    strcpy(name_char, "Battery Monitor");
    break;
  case Configuration::BAROMETER:
    strcpy(name_char, "Barometer");
    break;
  case Configuration::MAGNETOMETER:
    strcpy(name_char, "Magnetometer");
    break;
  default:
    strcpy(name_char, "Error/Unsupported");
    break;
  }
}
void AirbourneBoardConfigManager::get_config_name(device_t device, hardware_config_t config, uint8_t (&name)[CONFIG_NAME_LENGTH])
{
  char *name_char = reinterpret_cast<char*>(name);
  const char *name_str = "Invalid Config";
  switch(device)
  {
  case Configuration::SERIAL:
    switch(config)
    {
    case 0:
      name_str="VCP over USB";
      break;
    case 1:
      name_str="UART1 on Main";
      break;
    case 2:
      name_str="UART2 on Flex-IO";
      break;
    case 3:
      name_str="UART3 on Flexi";
      break;
    }
    break;
  case Configuration::RC:
    if(config==0)
      name_str = "PPM on Flex-IO";
    else
      name_str = "SBUS on Main";
    break;
  case Configuration::AIRSPEED:
    if(config==0)
      name_str = "Disabled";
    else
      name_str = "I2C2 on Flexi";
    break;
  case Configuration::GNSS:
    switch(config)
    {
    case 0:
      name_str = "Disabled";
      break;
    case 1:
      name_str = "UART1 on main";
      break;
    case 2:
      name_str = "UART2 on Flex-Io";
      break;
    case 3:
      name_str = "UART3 on Flexi";
      break;
    }
    break;
  case Configuration::SONAR:
    if(config ==0)
      name_str = "Disabled";
    else
      name_str = "I2C2 on Flexi";
    break;
  case Configuration::BATTERY_MONITOR:
    if(config==0)
      name_str = "Disabled";
    else
      name_str = "ADC3 on Power";
    break;
  case Configuration::BAROMETER:
    if(config==0)
      name_str = "Onboard barometer";
    break;
  case Configuration::MAGNETOMETER:
    if(config ==0)
      name_str = "Onboard magnetometer";
    break;
  default:
    name_str = "Invalid device";
  }
  if(config > max_configs[device])
    name_str = "Invalid config";
  strcpy(name_char, name_str);
}
AirbourneBoardConfigManager::revo_port AirbourneBoardConfigManager::get_port(uint8_t device, uint8_t config)
{
  switch(device)
  {
  case Configuration::SERIAL:
    if(config == 0)
      return USB_PORT;
    // break intentionally ommitted
  case Configuration::GNSS:
    return static_cast<revo_port>(config);
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
  }
  return NO_PORT;
}
} //rosflight_firmware
