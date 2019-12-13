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
void AirbourneBoardConfigManager::get_device_name(device_t device, char (&name)[20])
{
  switch(device)
  {
  case serial:
    strcpy(name, "Serial");
    break;
  case rc:
    strcpy(name, "RC");
    break;
  case airspeed:
    strcpy(name, "Airspeed");
    break;
  case gnss:
    strcpy(name, "GNSS");
    break;
  case sonar:
    strcpy(name, "Sonar");
    break;
  case battery_monitor:
    strcpy(name, "Battery Monitor");
    break;
  case barometer:
    strcpy(name, "Barometer");
    break;
  case magnetometer:
    strcpy(name, "Magnetometer");
    break;
  default:
    strcpy(name, "Error/Unsupported");
    break;
  }
}
void AirbourneBoardConfigManager::get_config_name(device_t device, hardware_config_t config, char (&name)[20])
{
  //TODO
  (void)device;
  (void)config;
  strcpy(name, "TODO");
}
} //rosflight_firmware
