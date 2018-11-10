// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MEMORY_TOOLS__IMPL__STATIC_ALLOCATOR_HPP_
#define MEMORY_TOOLS__IMPL__STATIC_ALLOCATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>

#include "../safe_fwrite.hpp"

namespace osrf_testing_tools_cpp
{
namespace memory_tools
{
namespace impl
{

template<size_t MemoryPoolSize>
class StaticAllocator
{
public:
  StaticAllocator()
  : memory_pool_{0},
    begin_(memory_pool_),
    end_(memory_pool_ + MemoryPoolSize),
    stack_pointer_(memory_pool_)
  {
    (void)memset(memory_pool_, 0x0, sizeof(memory_pool_));
  }

  void *
  allocate(size_t size)
  {
    if (size <= size_t(std::distance(stack_pointer_, end_))) {
      uint8_t * result = stack_pointer_;
      stack_pointer_ += size;
      return result;
    }
    SAFE_FWRITE(stderr, "StackAllocator.allocate() -> nullptr\n");
    return nullptr;
  }

  void *
  reallocate(void * memory_in, size_t size)
  {
    if (!pointer_belongs_to_allocator(memory_in)) {
      SAFE_FWRITE(stderr,
        "StaticAllocator::reallocate(): asked to reallocate extra-allocator memory\n");
      return nullptr;
    }
    void * memory = this->allocate(size);
    if (nullptr != memory) {
      // This would be an unsafe opertion, because memcpy would read beyond the
      // original size of memory_in in the case that size is bigger than the
      // original size requested for memory_in, but since we know memory_in
      // came from this static allocator and that we were able to allocate
      // more memory after memory_in, then we can know that while what comes
      // after memory_in might be unrelated or garbage, it will be safe to read
      // from that space.
      memcpy(memory, memory_in, size);
      this->deallocate(memory_in);
    }
    return memory;
  }

  void *
  zero_allocate(size_t count, size_t size)
  {
    size_t total_size = count * size;
    void * memory = this->allocate(total_size);
    if (nullptr != memory) {
      memset(memory, 0x0, total_size);
    }
    return memory;
  }

  bool
  pointer_belongs_to_allocator(const void * pointer) const
  {
    const uint8_t * typed_pointer = reinterpret_cast<const uint8_t *>(pointer);
    return (
      !(std::less<const uint8_t *>()(typed_pointer, begin_)) &&
      (std::less<const uint8_t *>()(typed_pointer, end_))
    );
  }

  bool
  deallocate(void * pointer)
  {
    return this->pointer_belongs_to_allocator(pointer);
  }

private:
  uint8_t memory_pool_[MemoryPoolSize];
  uint8_t * begin_;
  uint8_t * end_;
  uint8_t * stack_pointer_;
};

}  // namespace impl
}  // namespace memory_tools
}  // namespace osrf_testing_tools_cpp

int main(void)
{
  osrf_testing_tools_cpp::memory_tools::impl::StaticAllocator<64> sa;
  void * mem = sa.allocate(16);
  (void)mem;
  return 0;
}

#endif  // MEMORY_TOOLS__IMPL__STATIC_ALLOCATOR_HPP_
