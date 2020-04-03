#include "memory_manager.h"

#include "rosflight.h"

namespace rosflight_firmware
{
MemoryManager::MemoryManager(ROSflight &rf) : RF_{rf} {}
bool MemoryManager::read_memory()
{
  RF_.board_.memory_init();
  ready_ = RF_.board_.memory_read(&memory_, sizeof(PersistentMemory));
  return ready_;
}
bool MemoryManager::write_memory()
{
  RF_.params_.prepare_write();
  RF_.config_manager_.prepare_write();
  return RF_.board_.memory_write(&memory_, sizeof(PersistentMemory));
}

} // namespace rosflight_firmware
