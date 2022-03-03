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
  
  log_debug("Creating ", name);
_memory = std::make_unique<bi::shared_memory_object>(bi::open_or_create, name, bi::read_write);
  
  if (!create_mutex(name))
    return false;

  // forces a size of 100 by at start
  resize(100);

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
  log_debug("Opening ", name);
  _memory = std::make_unique<bi::shared_memory_object>(bi::open_only, name, bi::read_only);

  if (!create_mutex(name))
    return false;

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
  bi::offset_t currentSize;
  _memory->get_size(currentSize);
  if (currentSize != size)
    _memory->truncate(size);

  return true;
}

std::size_t SharedMemoryBlock::get_size() {
  bi::offset_t size;
  if (_memory->get_size(size))
    return size;
  else
    return 0;
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
  if (!_mutex) return;
  bi::scoped_lock<bi::named_upgradable_mutex> locker(*_mutex);
  // log_debug("Mutex lock for writing");
  bi::mapped_region region(*_memory, bi::read_write);
  // log_debug("Mapped memory for writing");
  uint8_t *ptr = static_cast<uint8_t *>(region.get_address());
  if (ptr != nullptr) {
    // log_debug("Executing lambda for writing");
    callback(ptr, region.get_size());
    // log_debug("Notifying all clients");
    _condition->notify_all();
  }
  // log_debug("End of writing");
}

void SharedMemoryBlock::wait_for_reading(std::function<void(uint8_t *ptr, size_t size)> callback) {
  if (!_mutex) return;
  bi::sharable_lock<bi::named_upgradable_mutex> locker(*_mutex/*, bi::defer_lock*/);
  // log_debug("Mutex lock for reading");
  _condition->wait(locker);
  bi::mapped_region region(*_memory, bi::read_only);
  // log_debug("Mapped memory for reading");
  uint8_t *ptr = static_cast<uint8_t *>(region.get_address());
  if (ptr != nullptr) {
    // log_debug("Executing lambda for reading");
    callback(ptr, region.get_size());
  }
  // log_debug("End of reading");
}

} // namespace detail
} // namespace streaming
} // namespace carla
