/*
 * params.h
 *
 *  Created on: Jun 13, 2020
 *      Author: Julian Gaal
 */

#ifndef _PARAMS_H
#define _PARAMS_H

#include <trail_detection/fpga/ip/trail_detection.h>
#include <trail_detection/ros_wrapper/misc.h>
#include <string>

namespace trail_detection
{
namespace ros_wrapper
{

/**
 * FPGAParams makes setting all parameters easier
 *
 * Future TODO: at the moment there is no enforcement to set values
 */
class FPGAParams
{
private:
  /// image width
  int width_;
  /// image height
  int height_;
  /// blue offset used in algorithm
  int blue_offset_;
  /// threshold for binary threshold
  int gray_threshold_;
  /// array holding horizontal block sizes
  trail_detection::fpga::ip::td_block_sizes block_sizes_;
  /// array holding weights, giving importance to specific blocks, less to others
  trail_detection::fpga::ip::td_block_weights block_weights_;
  /// length of virtual memory address
  int lenght_;
  /// bin file name, c-style string from compatibility with xilinx lib
  const char *binfile_;
  /// wether or not to save output image to buffer
  Debug debug_;
public:
  /// default constructor
  FPGAParams() = default;

  /// default destructor
  ~FPGAParams() = default;

  /**
   * Set image width
   * @param width width of image
   * @return FPGAParams reference
   */
  FPGAParams &setWidth(int width);

  /**
   * Set image height
   * @param height height of image
   * @return FPGAParams reference
   */
  FPGAParams &setHeight(int height);

  /**
   * Set blue offset
   * @param blue_offset offset used in algorithm
   * @return FPGAParams reference
   */
  FPGAParams &setBlueOffset(int blue_offset);

  /**
   * Set gray threshold for image binarization
   * @param gray_threshold threshold for image binarization
   * @return FPGAParams reference
   */
  FPGAParams &setGrayThreshold(int gray_threshold);

  /**
   * Set block sizes used in horizontal image slicing
   * @param blocksizes block sizes used in horizontal image slicing
   * @return
   */
  FPGAParams &setBlockSizes(trail_detection::fpga::ip::td_block_sizes blocksizes);

  /**
   * Set block weights responsible for weighing image blocks by importance
   * @param weights block weights responsible for weighing image blocks by importance
   * @return
   */
  FPGAParams &setBlockWeights(trail_detection::fpga::ip::td_block_weights weights);

  /**
   * Set name of bin file used to flash fpga
   * @param binfile name of bin file used to flash fpga
   * @return
   */
  FPGAParams &setBinfile(const char *binfile);

  /**
   * Set length of mmapped virtual address
   * @param length length of mmapped virtual address
   * @return
   */
  FPGAParams &setLength(int length);

  /**
   * Enable/disable debug
   * @param debug Enable/disable debug
   * @return
   */
  FPGAParams &setDebug(Debug debug);


  /**
   * Width getter
   * @return width
   */
  int width() const;

  /**
   * Heigth getter
   * @return height
   */
  int height() const;

  /**
   * Blue offset getter
   * @return blue offset
   */
  int blueOffset() const;

  /**
   * Get gray threshold
   * @return gray threshold
   */
  int grayThreshold() const;

  /**
   * virtual address length getter
   * @return virtual address length
   */
  int length() const;

  /**
   * bin file name getter
   * @return name of bin file used to flash fpga
   */
  const char *binfile() const;

  /**
   * Block size getter
   * @return block sizes
   */
  const trail_detection::fpga::ip::td_block_sizes &blockSizes() const;

  /**
   * Block weight getter
   * @return block weights
   */
  const trail_detection::fpga::ip::td_block_weights &blockWeights() const;

  /**
   * debug getter
   * @return debug
   */
  trail_detection::Debug debug() const;
};

} // end namespace ros_wrapper
} // end namespace trail_detection

#endif //_PARAMS_H
