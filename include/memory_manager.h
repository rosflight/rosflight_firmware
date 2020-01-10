#ifndef MEMORY_MANAGER_H
#define MEMORY_MANAGER_H

#include "param.h"
#include "config_manager.h"

namespace rosflight_firmware
{
class ROSflight;
class MemoryManager
{
public:
  typedef struct
  {
    Params::params_t params;
    ConfigManager::Config config;
  } persistent_memory_t;

  MemoryManager(ROSflight &rf);
  bool read_memory();
  bool write_memory();
  inline bool is_ready() {return ready_;}

  inline Params::params_t &get_params(){return memory_.params;}
  inline ConfigManager::Config &get_config(){return memory_.config;}

private:
  ROSflight &RF_;
  persistent_memory_t memory_;
  bool ready_{false};

};
} // namespace rosflight_firmware

#endif // MEMORY_MANAGER_H
