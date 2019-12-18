#include "airbourne_board_config_manager.h"
#include <cstring>

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
  if(device >= device_count)
    return 0;
  else
    return max_configs[device];
}
BoardConfigManager::config_response AirbourneBoardConfigManager::check_config_change(device_t device, hardware_config_t config)
{
  BoardConfigManager::config_response resp;
  // TODO actual checks
  (void)device;
  (void)config;
  resp.successful = true;
  resp.reboot_required = true;
  return resp;
}
void AirbourneBoardConfigManager::get_device_name(device_t device, uint8_t (&name)[20])
{
  char *name_char = reinterpret_cast<char*>(name);
  switch(device)
  {
  case serial:
    strcpy(name_char, "Serial");
    break;
  case rc:
    strcpy(name_char, "RC");
    break;
  case airspeed:
    strcpy(name_char, "Airspeed");
    break;
  case gnss:
    strcpy(name_char, "GNSS");
    break;
  case sonar:
    strcpy(name_char, "Sonar");
    break;
  case battery_monitor:
    strcpy(name_char, "Battery Monitor");
    break;
  case barometer:
    strcpy(name_char, "Barometer");
    break;
  case magnetometer:
    strcpy(name_char, "Magnetometer");
    break;
  default:
    strcpy(name_char, "Error/Unsupported");
    break;
  }
}
void AirbourneBoardConfigManager::get_config_name(device_t device, hardware_config_t config, uint8_t (&name)[20])
{
  char *name_char = reinterpret_cast<char*>(name);
  const char *name_str;
  switch(device)
  {
  case serial:
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
  case rc:
    if(config==0)
      name_str = "PPM on Flex-IO";
    else
      name_str = "SBUS on Main";
    break;
  case airspeed:
    if(config==0)
      name_str = "Disabled";
    else
      name_str = "I2C2 on Flexi";
    break;
  case gnss:
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
  case sonar:
    if(config ==0)
      name_str = "Disabled";
    else
      name_str = "I2C2 on Flexi";
    break;
  case battery_monitor:
    if(config==0)
      name_str = "Disabled";
    else
      name_str = "ADC3 on Power";
    break;
  case barometer:
    if(config==0)
      name_str = "Onboard barometer";
    break;
  case magnetometer:
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
} //rosflight_firmware
