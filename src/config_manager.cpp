#include "config_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{
ConfigManager::ConfigManager(ROSflight &RF, Config &config):
  RF_{RF},
  config_{config}
{}

bool ConfigManager::init()
{
  if(!read())
    fill_defaults();
  return true;
}

bool ConfigManager::configure_devices() const
{
  bool success = true;
  for(device_t device{Configuration::FIRST_DEVICE}; device < Configuration::DEVICE_COUNT; ++device)
    success = RF_.board_.enable_device(device, config_.config[device], RF_.params_) && success;
  return success;
}

ConfigManager::ConfigResponse ConfigManager::attempt_set_configuration(device_t device, uint8_t config)
{
  ConfigResponse resp;
  if(RF_.state_manager_.state().armed)
  {
    resp.successful = false;
    resp.reboot_required = false;
    strcpy(resp.message, "Config changes while armed are not allowed.");
    return resp;
  }
  resp = RF_.board_.get_board_config_manager().check_config_change(device, config, *this);
  if(resp.successful)
    set_configuration(device, config);
  return resp;
}
void ConfigManager::set_configuration(device_t device, uint8_t config)
{
  config_.config[device] = config;
  // TODO consider deinitializing and changing the config
}
uint8_t ConfigManager::get_configuration(device_t device) const
{
  return config_.config[device];
}
uint8_t ConfigManager::operator[](device_t device) const
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
uint32_t ConfigManager::generate_checksum() const
{
  //8 bit fletcher algorithm, because we can't assume that the struct is a multiple of 16 bits
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
