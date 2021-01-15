/*
 * params.cpp
 *
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/ros_wrapper/params.h>

using namespace trail_detection::ros_wrapper;

FPGAParams &FPGAParams::setWidth(int width)
{
  width_ = width;
  return *this;
}

FPGAParams &FPGAParams::setHeight(int height)
{
  height_ = height;
  return *this;
}

FPGAParams &FPGAParams::setBlueOffset(int blue_offset)
{
  blue_offset_ = blue_offset;
  return *this;
}

FPGAParams &FPGAParams::setGrayThreshold(int gray_threshold)
{
  gray_threshold_ = gray_threshold;
  return *this;
}

FPGAParams &FPGAParams::setLength(int length)
{
  lenght_ = length;
  return *this;
}

FPGAParams &FPGAParams::setBinfile(const char *binfile)
{
  binfile_ = binfile;
  return *this;
}

FPGAParams &FPGAParams::setBlockSizes(trail_detection::fpga::ip::td_block_sizes blocksizes)
{
  block_sizes_ = blocksizes;
  return *this;
}

FPGAParams &FPGAParams::setBlockWeights(trail_detection::fpga::ip::td_block_weights weights)
{
  block_weights_ = weights;
  return *this;
}

int FPGAParams::width() const
{
  return width_;
}

int FPGAParams::height() const
{
  return height_;
}

int FPGAParams::blueOffset() const
{
  return blue_offset_;
}

int FPGAParams::grayThreshold() const
{
  return gray_threshold_;
}

int FPGAParams::length() const
{
  return lenght_;
}

const char *FPGAParams::binfile() const
{
  return binfile_;
}

const trail_detection::fpga::ip::td_block_sizes &FPGAParams::blockSizes() const
{
  return block_sizes_;
}

const trail_detection::fpga::ip::td_block_weights &FPGAParams::blockWeights() const
{
  return block_weights_;
}

trail_detection::Debug FPGAParams::debug() const
{
  return debug_;
}

FPGAParams &FPGAParams::setDebug(Debug debug)
{
  debug_ = debug;
  return *this;
}


