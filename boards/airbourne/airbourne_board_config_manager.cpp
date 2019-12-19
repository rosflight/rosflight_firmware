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
  if(device >= device_count)
    return 0;
  else
    return max_configs[device];
}
BoardConfigManager::config_response AirbourneBoardConfigManager::check_config_change(device_t device, hardware_config_t config)
{
  BoardConfigManager::config_response resp;
  resp.reboot_required = false;
  resp.successful = false;

  if(device > device_count)
  {
    strcpy(resp.error_message, "Device not found");
    return resp;
  }

  if(config > max_configs[device])
  {
    strcpy(resp.error_message, "Configuration not found");
    return resp;
  }
  revo_port port = get_port(device, config);
  ConfigManager &cm = RF_->config_manager_;
  device_t conflict_device = device_count;

  switch(device)
  {
  case rc:
    if(config ==0)
      break;
  case serial:
  case gnss:
    for(device_t other_device{static_cast<device_t>(0)}; other_device != device_count; ++other_device)
      if(other_device != rc && port == get_port(other_device, cm[other_device]))
      {
        conflict_device = other_device;
        break;
      }
    break;
  case airspeed:
  case sonar:
    if(cm[gnss] == 3)
      conflict_device = gnss;
    if(cm[serial] == 3)
      conflict_device = serial;
    break;
  default:
    break;
  }
  if(conflict_device != device_count)
  {
    switch(conflict_device)
    {
    case serial:
      strcpy(resp.error_message, "Port is used by serial.");
      break;
    case rc:
      strcpy(resp.error_message, "Port is used by RC.");
      break;
    case airspeed:
      strcpy(resp.error_message, "Port is used by airspeed sensor.");
      break;
    case gnss:
      strcpy(resp.error_message, "Port is used by GNSS receiver.");
      break;
    case sonar:
      strcpy(resp.error_message, "Port is used by sonar sensor.");
      break;
    // At the time of this writing, the below should not cause issues
    case battery_monitor:
      strcpy(resp.error_message, "Port is used by battery monitor.");
      break;
    case barometer:
      strcpy(resp.error_message, "Port is used by barometer.");
      break;
    case magnetometer:
      strcpy(resp.error_message, "Port is used by magnetometer.");
      break;
    default:
      strcpy(resp.error_message, "Other error.");
      break;
    }
    return resp;
  }
  resp.successful = true;
  resp.reboot_required = true;
  resp.error_message[0]=0;
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
AirbourneBoardConfigManager::revo_port AirbourneBoardConfigManager::get_port(uint8_t device, uint8_t config)
{
  switch(device)
  {
  case serial:
    if(config == 0)
      return usb;
    // break intentionally ommitted
  case gnss:
    return static_cast<revo_port>(device);
  case rc:
    if(config == 0)
      return flex_io;
    if(config == 1)
      return main;
    break;
  case airspeed:
  case sonar:
    if(config==1)
      return flexi;
    break;
  case battery_monitor:
    if(config == 1)
      return power;
  }
  return none;
}
} //rosflight_firmware
