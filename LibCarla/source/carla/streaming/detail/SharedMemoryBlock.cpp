// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "SharedMemoryBlock.h"

#include "carla/Logging.h"

#include <boost/interprocess/sync/sharable_lock.hpp>

#include <sstream>
#include <iostream>

namespace carla {
namespace streaming {
namespace detail {

SharedMemoryBlock::~SharedMemoryBlock() {
  // delete objects
  _memory.reset();
  _region.reset();
  _mutex.reset();
  _condition.reset();

  remove_named_objects(_name.c_str());
}

void SharedMemoryBlock::remove_named_objects(const char *name) {
  log_debug("Removing ", name, "...");
  if (!bi::shared_memory_object::remove(name))
    log_debug("...failed");
  else
    log_debug("...done");

  _mutex_name.assign(name);
  _mutex_name.append("_mutex");
  log_debug("Removing ", _mutex_name.c_str(), "...");
  if (!bi::named_upgradable_mutex::remove(_mutex_name.c_str()))
    log_debug("...failed");
  else
    log_debug("...done");

  _condition_name.assign(name);
  _condition_name.append("_condition");
  log_debug("Removing ", _condition_name.c_str(), "...");
  if (!bi::named_condition_any::remove(_condition_name.c_str()))
    log_debug("...failed");
  else
    log_debug("...done");
}

bool SharedMemoryBlock::create(const char *name) {
  remove_named_objects(name);
  
  if (!create_mutex(name))
    return false;

  log_debug("Creating ", name);
  _memory = std::make_unique<bi::shared_memory_object>(bi::open_or_create, name, bi::read_write);
  
  // resize to allow for the first size_t field
  resize(0);

  _region = std::make_unique<bi::mapped_region>(*_memory, bi::read_write);

  _name = name;
  return true;
}

bool SharedMemoryBlock::create(std::string name) {
  return create(name.c_str());
}

bool SharedMemoryBlock::create(uint16_t port, stream_id_type stream_id) {
  std::ostringstream tmp;
  tmp << "carla_" << port << "_" << stream_id;
  return create(tmp.str());
}

bool SharedMemoryBlock::open(const char *name) {
  if (!create_mutex(name))
    return false;

  log_debug("Opening ", name);
  _memory = std::make_unique<bi::shared_memory_object>(bi::open_only, name, bi::read_only);

  _region = std::make_unique<bi::mapped_region>(*_memory, bi::read_only);

  _name = name;
  return true;
}

bool SharedMemoryBlock::open(std::string name) {
  return open(name.c_str());
}

bool SharedMemoryBlock::open(uint16_t port, stream_id_type stream_id) {
  std::ostringstream tmp;
  tmp << "carla_" << port << "_" << stream_id;
  return open(tmp.str());
}

bool SharedMemoryBlock::resize(std::size_t size) {
  if (!_mutex) return false;

  bi::scoped_lock<bi::named_upgradable_mutex> locker(*_mutex);
  std::size_t size_needed = size + sizeof(std::size_t);

  bi::offset_t currentSize;
  _memory->get_size(currentSize);
  if (size_needed != currentSize) {
    if (size_needed > currentSize) {
      log_debug("Remapping for writing,", size_needed);
      _memory->truncate(size_needed);
      _region = std::make_unique<bi::mapped_region>(*_memory, bi::read_write);
    }
    // set the new size
    std::size_t *ptr = static_cast<std::size_t *>(_region->get_address());
    ptr[0] = size;
  }

  return true;
}

std::string SharedMemoryBlock::get_name() {
  return _name;
}

bool SharedMemoryBlock::create_mutex(const char *name) {
  _mutex_name.assign(name);
  _mutex_name.append("_mutex");
  log_debug("Creating mutex ",  _mutex_name.c_str());
  _mutex = std::make_unique<bi::named_upgradable_mutex>(bi::open_or_create, _mutex_name.c_str());

  _condition_name.assign(name);
  _condition_name.append("_condition");
  log_debug("Creating mutex ",  _condition_name.c_str());
  _condition = std::make_unique<bi::named_condition_any>(bi::open_or_create, _condition_name.c_str());

  return true;
}

void SharedMemoryBlock::wait_for_writing(std::function<void(uint8_t *ptr, size_t size)> callback) {
  if (!_mutex) {
    log_debug("Mutex error for writing");
    return;
  }
  if (!_region) {
    log_debug("Region error for writing");
    return;
  }

  log_debug("Mutex lock for writing");
  bi::scoped_lock<bi::named_upgradable_mutex> locker(*_mutex);
  
  uint8_t *ptr = static_cast<uint8_t *>(_region->get_address());
  std::size_t *size_ptr = static_cast<std::size_t *>(_region->get_address());
  if (ptr != nullptr) {
    // log_debug("Executing lambda for writing");
    callback(ptr + sizeof(std::size_t), *size_ptr);
    log_debug("Notifying all clients");
    _condition->notify_all();
  }
  log_debug("End of writing");
}

void SharedMemoryBlock::wait_for_reading(std::function<void(uint8_t *ptr, size_t size)> callback) {
  if (!_mutex || !_region) return;

  log_debug("Mutex lock for reading");
  bi::sharable_lock<bi::named_upgradable_mutex> locker(*_mutex);
  _condition->wait(locker);
  log_debug("After signal for reading");

  // check if it is still valid, after the wait
  if (!_mutex || !_region) {
    log_debug("Reading no valide");
    return;
  }

  // check to remap memory
  std::size_t *size_ptr = static_cast<std::size_t *>(_region->get_address());
  if (size_ptr && size_ptr[0] > _region->get_size()) {
    log_debug("Remapping for reading,", *size_ptr);
    _region = std::make_unique<bi::mapped_region>(*_memory, bi::read_only);
    size_ptr = static_cast<std::size_t *>(_region->get_address());
  }

  uint8_t *ptr = static_cast<uint8_t *>(_region->get_address());
  if (ptr != nullptr) {
    log_debug("Executing lambda for reading");
    callback(ptr + sizeof(std::size_t), *size_ptr);
  }
  log_debug("End of reading");
}

} // namespace detail
} // namespace streaming
} // namespace carla
