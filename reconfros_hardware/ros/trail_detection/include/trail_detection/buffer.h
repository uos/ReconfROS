/*
 * buffer.h
 *
 *  Created on: Jun 9, 2020
 *      Author: Marcel Flottmann
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stddef.h>
#include <stdint.h>

namespace trail_detection
{

/**
 * Represents a buffer in memory that can be accessed by the hardware through the physical address
 */
class Buffer
{
private:
  /// size of buffer
  size_t size;
  /// physical address
  uint32_t physical_addr;

  /// virtual address
  void *virtual_addr;

  /**
   * Allocates buffer
   */
  void allocate();

  /**
   * Frees buffer
   */
  void free();

public:
  /**
   * Create new Buffer of specified size
   * @param size
   */
  Buffer(size_t size);

  /// destructor
  ~Buffer();

  /// no copy constructor
  Buffer(const Buffer &) = delete;

  /// no assignment
  Buffer &operator=(const Buffer &) = delete;

  /**
	 * Invalidate the cache so that the software can see the changes from the hardware.
	 */
  void invalidate();

  /**
   * Flush the cache so that the hardware can see the changes from the software.
   */
  void flush();

  /**
   * Get virtual address of buffer
   * @return
   */
  inline void *getVirtual()
  {
    return virtual_addr;
  }

  /**
   * Get physical address of buffer
   * @return address
   */
  inline uint32_t getPhysical() const
  {
    return physical_addr;
  }
};

} // end namespace trail_detection

#endif /* BUFFER_H_ */
