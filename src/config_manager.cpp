#include "config_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{
ConfigManager::ConfigManager(ROSflight &RF, config_t &config):
  RF_{RF},
  config_{config}
{}

bool ConfigManager::init()
{
  if(!read())
    fill_defaults();
  return true;
}

bool ConfigManager::configure_devices()
{
  bool success = true;
  for(uint8_t device{0}; device < device_t::device_count; device++)
    success = success &&RF_.board_.enable_device(static_cast<device_t>(device), config_.config[device], RF_.params_);
  return success;
}

void ConfigManager::set_configuration(device_t device, uint8_t config)
{
  config_.config[device] = config;
  // TODO consider deinitializing and changing the config
}
uint8_t ConfigManager::get_configuration(device_t device)
{
  return config_.config[device];
}
uint8_t ConfigManager::operator[](device_t device)
{
  return get_configuration(device);
}

void ConfigManager::prepare_write()
{
  config_.checksum = generate_checksum();
}

bool ConfigManager::read()
{
  if(!RF_.memory_manager_.is_ready())
    return false;
  if(generate_checksum() != config_.checksum)
    return false;
  return true;
}

void ConfigManager::fill_defaults()
{
  memset(config_.config, 0, sizeof(config_.config));
}
uint32_t ConfigManager::generate_checksum()
{
  //8 bit fletcher algorithm
  const uint8_t *config_data = reinterpret_cast<const uint8_t*>(config_.config);
  uint8_t check_a{0};
  uint8_t check_b{0};
  for(size_t index{0}; index< sizeof(config_.config); index++ )
  {
    check_a += config_data[index];
    check_b += check_a;
  }
  return check_a & (check_b<<8) & (~check_a<<16) & (~check_b<<24);
}

}
