/*
 * trail_detection.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/fpga/ip/trail_detection.h>

namespace ip = trail_detection::fpga::ip;

void ip::TrailDetection::setIndexTable(const ip::td_block_sizes &blocksizes)
{
  uint32_t index = 0;

  for (int i = 0; i < blocksizes.size(); i++)
  {
    index += blocksizes[i];
    this->INDEX_TABLE[i].data = index;
  }
}

void ip::TrailDetection::setIndexWeights(const ip::td_block_weights &weights)
{
  for (int i = 0; i < weights.size(); i++)
  {
    this->INDEX_WEIGHTS[i].data = static_cast<uint16_t>(weights[i] * (1 << 12));
  }
}
