/*
 * buffer.cpp
 *
 *  Created on: Jun 9, 2020
 *      Author: Marcel Flottmann
 */

#include <trail_detection/buffer.h>

extern "C" {
#include <libxlnk_cma.h>
};

#include <stdexcept>

using namespace trail_detection;

Buffer::Buffer(size_t size) :
    size(size)
{
  allocate();
}

Buffer::~Buffer()
{
  free();
}

void Buffer::allocate()
{
  virtual_addr = cma_alloc(size, true);
  if (virtual_addr == (void *) -1)
    throw std::runtime_error("buffer allocation failed");
  physical_addr = cma_get_phy_addr(virtual_addr);
}

void Buffer::free()
{
  cma_free(virtual_addr);
}

void Buffer::invalidate()
{
  cma_invalidate_cache(virtual_addr, physical_addr, size);
}

void Buffer::flush()
{
  cma_flush_cache(virtual_addr, physical_addr, size);
}

