//
// Created by julian on 6/12/20.
//

#ifndef _FPGA_H
#define _FPGA_H

#include <trail_detection/fpga/ip/mm2vs.h>
#include <trail_detection/fpga/ip/vs2mm.h>
#include <trail_detection/fpga/ip/trail_detection.h>
#include <trail_detection/buffer.h>

#include <sys/mman.h>

/// Control signal start: bit 0 = 1
#define AP_START 1
/// Control signal done: bit 1 = 1
#define AP_DONE 2

namespace trail_detection
{
namespace fpga
{

/**
 * FGPA handles initialization and communication with the FPGA at runtime
 */
class FPGA
{
private:
  // file descriptor of virtual memory address
  int file_descriptor;
  // length of virtual memory address
  int length;

  /**
   * FPGA constructor
   * @param binfile c-cstyle string with name of binfile. Will be passed to Xilinx lib
   * @param length length of virtual memory
   */
  explicit FPGA(const char *binfile, size_t length);

  /**
   * Flashes FPGA: Load binfile into FPGA
   * @param binfile: c-cstyle string with name of binfile. Will be passed to Xilinx lib
   */
  void flash_fpga(const char *binfile);

  /**
   * Creates virtual memory mapping between hardware address of FPGA and new virtually memory address
   * with mmap()
   */
  void open_mem();

public:
  /**
   * Factory method to ensure only one FPGA instance at all times
 * @param binfile c-cstyle string with name of binfile. Will be passed to Xilinx lib
 * @param length length of virtual memory
   * @return
   */
  static FPGA &init(const char *binfile, size_t length);

  /// trail detection ip block
  trail_detection::fpga::ip::TrailDetection *trail_detection_ip;

  /// mm2vs ip block
  trail_detection::fpga::ip::MM2VS *mm2vs_ip;

  /// vs2mm ip block
  trail_detection::fpga::ip::VS2MM *vs2mm_ip;

  /// Destructor
  ~FPGA();

  /**
   * Sets registers available for parameterization
   * @param width - width of image
   * @param height - heigth of image
   * @param blue_offset - blue offset used in algorithm
   * @param threshold - threshold for white/black binarization
   */
  void set_param_registers(int width, int height, int blue_offset, int threshold);

  /**
   * Connects buffers with virtual memory address to FPGA with their respective physical memory address
   * @param inbuffer - input buffer contains image that will be processed in fpga
   * @param outbuffer - output buffer contains image after processing
   */
  void set_buffers(trail_detection::Buffer &inbuffer, trail_detection::Buffer &outbuffer);

  /**
   * Sets blocksizes and weights according to values decided by the software ms1 team
   * @param blocksizes - array containing size of each horizontal chunk in image
   * @param weights - array containing weights that modify relative importance of results in one chunk
   */
  void set_weights(const trail_detection::fpga::ip::td_block_sizes &blocksizes,
                   const trail_detection::fpga::ip::td_block_weights &weights);

  /**
   * Blocks thread until trail_detection_ip is done (AP_DONE)
   */
  void block_until_ready();

  /**
   * Reads weighted x value/position (output of algorithm) from result register
   * @return weighted x value/position
   */
  float get_result_x();

  /**
 * Reads weighted y value/position (output of algorithm) from result register
 * @return weighted y value/position
 */
  float get_result_y();

  /**
   * Initializes all ip blocks: sets them to ready/start (AP_START)
   */
  void start_pipeline();

  FPGA &operator=(const FPGA &) = delete;
};

/**
 * Maps hardware base address to virtual memory address in program
 * @tparam T - ip block, see fpga/ip/*.h
 * @param base_address - hardware base address
 * @param length - length of virtual memory
 * @param file_descriptor - file descriptor of opened device /dev/mem
 * @return virtual memory address, cast to ip block architecture
 */
template<typename T>
T *map_device(uint32_t base_address, size_t length, int file_descriptor)
{
  return static_cast<T *>(mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, file_descriptor, base_address));
}

} // end namespace fpga
} // end namespace trail_detection

#endif //_FPGA_H
