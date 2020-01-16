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
  struct PersistentMemory
  {
    Params::params_t params;
    ConfigManager::Config config;
  };

  MemoryManager(ROSflight &rf);
  /**
   * @brief Reads all memory from the board's persistent memory
   * @return if the read was succesful
   */
  bool read_memory();
  /**
   * @brief Writes all memory to the board's persistent memory
   * Beforehand, calls prepare_write on both the parameter server and config manager
   * @return if the write was successful
   */
  bool write_memory();
  /**
   * @brief Checks if memory has been read successfully at some point in the past
   * @return if memory has been successfully loaded
   */
  inline bool is_ready() {return ready_;}

  /**
   * @brief Get a pointer to the parameter structure, which was stored in persistent memory
   * @return A pointer to the parameter struct
   */
  inline Params::params_t &get_params(){return memory_.params;}
  /**
   * @brief Get a pointer to the config structure, which was stored in persistent memory
   * @return A pointer to the config struct
   */
  inline ConfigManager::Config &get_config(){return memory_.config;}

private:
  ROSflight &RF_;
  PersistentMemory memory_;
  bool ready_{false};

};
} // namespace rosflight_firmware

#endif // MEMORY_MANAGER_H
