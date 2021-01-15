/*
 * fpga.cpp
 *
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/fpga/fpga.h>
#include <boost/filesystem.hpp>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <ros/console.h>

using namespace trail_detection::fpga;
namespace fs = boost::filesystem;
namespace td = trail_detection;

FPGA &FPGA::init(const char *binfile, size_t length)
{
  static FPGA fpga(binfile, length);
  return fpga;
}

trail_detection::fpga::FPGA::FPGA(const char *binfile, size_t length) :
    file_descriptor(),
    length(length)
{
  flash_fpga(binfile);
  open_mem();

  mm2vs_ip = map_device<ip::MM2VS>(ip::MM2VS_BASE_ADDRESS, length, file_descriptor);
  vs2mm_ip = map_device<ip::VS2MM>(ip::VS2MM_BASE_ADDRESS, length, file_descriptor);
  trail_detection_ip = map_device<ip::TrailDetection>(ip::TRAIL_DETECTION_BASE_ADDRESS, length, file_descriptor);
}

trail_detection::fpga::FPGA::~FPGA()
{
  munmap(mm2vs_ip, length);
  munmap(vs2mm_ip, length);
  munmap(trail_detection_ip, length);
}

void FPGA::set_param_registers(int width, int height, int blue_offset, int threshold)
{
  mm2vs_ip->WIDTH = width;
  mm2vs_ip->HEIGHT = height;
  mm2vs_ip->STRIDE = width;

  vs2mm_ip->WIDTH = width;
  vs2mm_ip->HEIGHT = height;
  vs2mm_ip->STRIDE = width;

  trail_detection_ip->WIDTH = width;
  trail_detection_ip->HEIGHT = height;
  trail_detection_ip->BLUE_OFFSET = blue_offset;
  trail_detection_ip->THRESHOLD = threshold;
}

void FPGA::set_buffers(td::Buffer &inbuffer, td::Buffer &outbuffer)
{
  mm2vs_ip->MEM = inbuffer.getPhysical();
  vs2mm_ip->MEM = outbuffer.getPhysical();
}

void FPGA::set_weights(const ip::td_block_sizes &blocksizes, const ip::td_block_weights &weights)
{
  trail_detection_ip->setIndexTable(blocksizes);
  trail_detection_ip->setIndexWeights(weights);
}

float FPGA::get_result_x()
{
  return trail_detection_ip->RESULT_X / 16.f;
}

float FPGA::get_result_y()
{
  return trail_detection_ip->RESULT_Y / 16.f;
}

void FPGA::block_until_ready()
{
  while ((trail_detection_ip->AP_CTRL & AP_DONE) == 0)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}

void FPGA::start_pipeline()
{
  // start pipeline back to front
  vs2mm_ip->AP_CTRL = AP_START;
  trail_detection_ip->AP_CTRL = AP_START;
  mm2vs_ip->AP_CTRL = AP_START;
}

void FPGA::flash_fpga(const char *binfile)
{
  fs::path source(binfile);
  fs::path destination("/lib/firmware" / fs::path(binfile));

  if (not fs::exists(destination))
  {
    throw std::runtime_error("Make sure FastSenseMS1.bin exists in /lib/firmware");
  }

  int flags_fd = open("/sys/class/fpga_manager/fpga0/flags", O_WRONLY, 0);
  if (flags_fd < 0)
  {
    throw std::system_error(errno, std::system_category(), "Open flags file failed");
  }

  int ret = write(flags_fd, "0", 1);
  if (ret < 0)
  {
    close(flags_fd);
    throw std::system_error(errno, std::system_category(), "Write flags failed");
  }
  close(flags_fd);

  int firmware_fd = open("/sys/class/fpga_manager/fpga0/firmware", O_WRONLY, 0);
  if (firmware_fd < 0)
  {
    throw std::system_error(errno, std::system_category(), "Open firmware file failed");
  }

  ret = write(firmware_fd, binfile, strlen(binfile));
  if (ret < 0)
  {
    close(firmware_fd);
    throw std::system_error(errno, std::system_category(), "Write firmware failed");
  }

  close(firmware_fd);

  ROS_INFO_STREAM("Loaded bitstream");
}

void FPGA::open_mem()
{
  file_descriptor = open("/dev/mem", O_RDWR);
  if (file_descriptor < 0)
  {
    throw std::system_error(errno, std::system_category(), "Couldn't open /dev/mem for read write");
  }
}
