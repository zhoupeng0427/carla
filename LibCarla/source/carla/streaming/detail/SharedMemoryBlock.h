// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <boost/cstdint.hpp>

#include "carla/streaming/detail/Types.h"

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_upgradable_mutex.hpp>
#include <boost/interprocess/sync/named_condition_any.hpp>

#include <memory>

namespace carla {
namespace streaming {
namespace detail {

namespace bi = boost::interprocess;

class SharedMemoryBlock
{
  public:
  ~SharedMemoryBlock();

  // create memory for writing
  bool create(const char *name);
  bool create(std::string name);
  bool create(stream_id_type stream_id, uint16_t port);
  
  // open memory for reading
  bool open(const char *name);
  bool open(std::string name);
  bool open(stream_id_type stream_id, uint16_t port);
  
  bool resize(std::size_t size);
  std::size_t get_size();
  std::string get_name();

  // synchronization
  void wait_for_writing(std::function<void(uint8_t *ptr, size_t size)> callback);
  void wait_for_reading(std::function<void(uint8_t *ptr, size_t size)> callback);

  private:
  bool create_mutex(const char *name);
  void remove_named_objects(const char *name);

  std::string                                  _name;
  std::string                                  _mutex_name;
  std::string                                  _condition_name;
  std::unique_ptr<bi::shared_memory_object>    _memory;
  std::unique_ptr<bi::named_upgradable_mutex>  _mutex;
  std::unique_ptr<bi::named_condition_any>     _condition;
};

} // namespace detail
} // namespace streaming
} // namespace carla
