// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "SharedMemoryBlock.h"

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
  std::cout << "Removing " << name << "...";
  if (!bi::shared_memory_object::remove(name))
    std::cout << "failed\n";
  else
    std::cout << "ok\n";

  _mutex_name.assign(name);
  _mutex_name.append("_mutex");
  std::cout << "Removing " << _mutex_name.c_str() << "...";
  if (!bi::named_upgradable_mutex::remove(_mutex_name.c_str()))
    std::cout << "failed\n";
  else
    std::cout << "ok\n";

  _condition_name.assign(name);
  _condition_name.append("_condition");
  std::cout << "Removing " << _condition_name.c_str() << "...";
  if (!bi::named_condition_any::remove(_condition_name.c_str()))
    std::cout << "failed\n";
  else
    std::cout << "ok\n";
}

bool SharedMemoryBlock::create(const char *name) {
  remove_named_objects(name);
  
  std::cout << "Creating " << name << "\n";
  try {
    _memory = std::make_unique<bi::shared_memory_object>(bi::open_or_create, name, bi::read_write);
  }
  catch(...) {
    std::cout << "Error creating " << name << "\n";
    return false;
  }
  
  if (!create_mutex(name))
    return false;

  _name = name;
  return true;
}

bool SharedMemoryBlock::create(std::string name) {
  return create(name.c_str());
}

bool SharedMemoryBlock::create(stream_id_type stream_id, uint16_t port) {
  std::ostringstream tmp;
  tmp << "carla_" << port << "_" << stream_id;
  return create(tmp.str());
}

bool SharedMemoryBlock::open(const char *name) {
  std::cout << "Open " << name << "\n";
  try {
    _memory = std::make_unique<bi::shared_memory_object>(bi::open_only, name, bi::read_only);
    // bi::shared_memory_object mem(bi::open_only, name, bi::read_only);
    // _memory->swap(mem);
  }
  catch(...) {
    std::cout << "Error opening " << name << "\n";
    return false;
  }

  if (!create_mutex(name))
    return false;

  _name = name;
  return true;
}

bool SharedMemoryBlock::open(std::string name) {
  return open(name.c_str());
}

bool SharedMemoryBlock::open(stream_id_type stream_id, uint16_t port) {
  std::ostringstream tmp;
  tmp << "carla_" << port << "_" << stream_id;
  return open(tmp.str());
}

bool SharedMemoryBlock::resize(std::size_t size) {
  try {
    bi::offset_t currentSize;
    _memory->get_size(currentSize);
    if (currentSize != size)
      _memory->truncate(size);
  }
  catch(...) {
    return false;
  }
  
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
  std::cout << "Creating mutex: " << _mutex_name.c_str() << "\n";
  try {
    _mutex = std::make_unique<bi::named_upgradable_mutex>(bi::open_or_create, _mutex_name.c_str());
  }
  catch(...) {
    std::cout << "Error creating mutex\n";
    return false;
  }

  _condition_name.assign(name);
  _condition_name.append("_condition");
  std::cout << "Creating condition: " << _condition_name.c_str() << "\n";
  try {
    _condition = std::make_unique<bi::named_condition_any>(bi::open_or_create, _condition_name.c_str());
  }
  catch(...) {
    std::cout << "Error creating condition variable\n";
    return false;
  }
  return true;
}

void SharedMemoryBlock::wait_for_writing(std::function<void(uint8_t *ptr, size_t size)> callback) {
  bi::scoped_lock<bi::named_upgradable_mutex> locker(*_mutex);
  bi::mapped_region region(*_memory, bi::read_write);
  uint8_t *ptr = static_cast<uint8_t *>(region.get_address());
  callback(ptr, region.get_size());
  _condition->notify_all();
}

void SharedMemoryBlock::wait_for_reading(std::function<void(uint8_t *ptr, size_t size)> callback) {
  bi::sharable_lock<bi::named_upgradable_mutex> locker(*_mutex/*, bi::defer_lock*/);
  _condition->wait(locker);
  bi::mapped_region region(*_memory, bi::read_only);
  uint8_t *ptr = static_cast<uint8_t *>(region.get_address());
  callback(ptr, region.get_size());
}

} // namespace detail
} // namespace streaming
} // namespace carla
