/*
 * misc.h
 *
 * Miscellaneous helper functions, struct, enums
 *
 *  Created on: Jun 13, 2020
 *      Author: Julian Gaal
 */

#ifndef _MISC_H
#define _MISC_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace trail_detection
{

/**
 * Enum used to enable/disable debug output (vs2mm output)
 */
enum Debug
{
  YES,
  NO
};

namespace ros_wrapper
{

/**
 * Calculates angle from weighted x and weighted y output from fpga
 *
 * This is calculated in software, because of heavy use of floating point operations. Speedung by use of fpga is
 * unlikely
 *
 * Taken from software team @
 * https://gitlab.informatik.uni-osnabrueck.de/FastSense/software_ms1/-/blob/master/src/final_pipeline.cpp#L182
 *
 * @param width - image width
 * @param height - image height
 * @param sum_x_weighted - weighted x result from fpga
 * @param sum_y_weighted - weighted y result from fpga
 * @return
 */
float angle_from_result(int width, int height, float sum_x_weighted, float sum_y_weighted);

} // end ros_wrapper
} // end namespace traildetection

#endif //_MISC_H
