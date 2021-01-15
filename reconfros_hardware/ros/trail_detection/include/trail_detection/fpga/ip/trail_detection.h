/*
 * mm2vs.h
 * 
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#ifndef _TRAIL_DETECTION_H
#define _TRAIL_DETECTION_H

#include <cstdint>
#include <array>

namespace trail_detection
{
namespace fpga
{
namespace ip
{

/// base address of trail_detection ip block found in Vivado address editor
constexpr uint32_t TRAIL_DETECTION_BASE_ADDRESS = 0x43c10000;

/// number of blocks/slots cut out of image
constexpr int N_BLOCKS = 16;

/// array to save block sizes
using td_block_sizes = std::array<uint32_t, N_BLOCKS>;

/// array to save block weights
using td_block_weights = std::array<float, N_BLOCKS>;


/**
 * This struct represents the memory architecture of the trail_detection ip block found in
 * <root>/ip/hls/trail_detection. It is used for virtual memory mapping with mmap
 *
 * The trail detection ip block implements the FastSense 2020 trail detection algorithm
 */
struct TrailDetection
{
  void setIndexTable(const std::array<uint32_t, N_BLOCKS> &blocksizes);

  void setIndexWeights(const std::array<float, N_BLOCKS> &weights);
  // ==============================================================
  // Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2019.2 (64-bit)
  // Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
  // ==============================================================
  // AXILiteS
  // 0x000 : Control signals
  //         bit 0  - ap_start (Read/Write/COH)
  //         bit 1  - ap_done (Read/COR)
  //         bit 2  - ap_idle (Read)
  //         bit 3  - ap_ready (Read)
  //         bit 7  - auto_restart (Read/Write)
  //         others - reserved
  // 0x004 : Global Interrupt Enable Register
  //         bit 0  - Global Interrupt Enable (Read/Write)
  //         others - reserved
  // 0x008 : IP Interrupt Enable Register (Read/Write)
  //         bit 0  - Channel 0 (ap_done)
  //         bit 1  - Channel 1 (ap_ready)
  //         others - reserved
  // 0x00c : IP Interrupt Status Register (Read/TOW)
  //         bit 0  - Channel 0 (ap_done)
  //         bit 1  - Channel 1 (ap_ready)
  //         others - reserved
  // 0x010 : Data signal of width_V
  //         bit 11~0 - width_V[11:0] (Read/Write)
  //         others   - reserved
  // 0x014 : reserved
  // 0x018 : Data signal of height_V
  //         bit 11~0 - height_V[11:0] (Read/Write)
  //         others   - reserved
  // 0x01c : reserved
  // 0x020 : Data signal of index_table_0
  //         bit 15~0 - index_table_0[15:0] (Read/Write)
  //         others   - reserved
  // 0x024 : reserved
  // 0x028 : Data signal of index_table_1
  //         bit 15~0 - index_table_1[15:0] (Read/Write)
  //         others   - reserved
  // 0x02c : reserved
  // 0x030 : Data signal of index_table_2
  //         bit 15~0 - index_table_2[15:0] (Read/Write)
  //         others   - reserved
  // 0x034 : reserved
  // 0x038 : Data signal of index_table_3
  //         bit 15~0 - index_table_3[15:0] (Read/Write)
  //         others   - reserved
  // 0x03c : reserved
  // 0x040 : Data signal of index_table_4
  //         bit 15~0 - index_table_4[15:0] (Read/Write)
  //         others   - reserved
  // 0x044 : reserved
  // 0x048 : Data signal of index_table_5
  //         bit 15~0 - index_table_5[15:0] (Read/Write)
  //         others   - reserved
  // 0x04c : reserved
  // 0x050 : Data signal of index_table_6
  //         bit 15~0 - index_table_6[15:0] (Read/Write)
  //         others   - reserved
  // 0x054 : reserved
  // 0x058 : Data signal of index_table_7
  //         bit 15~0 - index_table_7[15:0] (Read/Write)
  //         others   - reserved
  // 0x05c : reserved
  // 0x060 : Data signal of index_table_8
  //         bit 15~0 - index_table_8[15:0] (Read/Write)
  //         others   - reserved
  // 0x064 : reserved
  // 0x068 : Data signal of index_table_9
  //         bit 15~0 - index_table_9[15:0] (Read/Write)
  //         others   - reserved
  // 0x06c : reserved
  // 0x070 : Data signal of index_table_10
  //         bit 15~0 - index_table_10[15:0] (Read/Write)
  //         others   - reserved
  // 0x074 : reserved
  // 0x078 : Data signal of index_table_11
  //         bit 15~0 - index_table_11[15:0] (Read/Write)
  //         others   - reserved
  // 0x07c : reserved
  // 0x080 : Data signal of index_table_12
  //         bit 15~0 - index_table_12[15:0] (Read/Write)
  //         others   - reserved
  // 0x084 : reserved
  // 0x088 : Data signal of index_table_13
  //         bit 15~0 - index_table_13[15:0] (Read/Write)
  //         others   - reserved
  // 0x08c : reserved
  // 0x090 : Data signal of index_table_14
  //         bit 15~0 - index_table_14[15:0] (Read/Write)
  //         others   - reserved
  // 0x094 : reserved
  // 0x098 : Data signal of index_table_15
  //         bit 15~0 - index_table_15[15:0] (Read/Write)
  //         others   - reserved
  // 0x09c : reserved
  // 0x0a0 : Data signal of index_weights_0_V
  //         bit 15~0 - index_weights_0_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0a4 : reserved
  // 0x0a8 : Data signal of index_weights_1_V
  //         bit 15~0 - index_weights_1_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0ac : reserved
  // 0x0b0 : Data signal of index_weights_2_V
  //         bit 15~0 - index_weights_2_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0b4 : reserved
  // 0x0b8 : Data signal of index_weights_3_V
  //         bit 15~0 - index_weights_3_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0bc : reserved
  // 0x0c0 : Data signal of index_weights_4_V
  //         bit 15~0 - index_weights_4_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0c4 : reserved
  // 0x0c8 : Data signal of index_weights_5_V
  //         bit 15~0 - index_weights_5_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0cc : reserved
  // 0x0d0 : Data signal of index_weights_6_V
  //         bit 15~0 - index_weights_6_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0d4 : reserved
  // 0x0d8 : Data signal of index_weights_7_V
  //         bit 15~0 - index_weights_7_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0dc : reserved
  // 0x0e0 : Data signal of index_weights_8_V
  //         bit 15~0 - index_weights_8_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0e4 : reserved
  // 0x0e8 : Data signal of index_weights_9_V
  //         bit 15~0 - index_weights_9_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0ec : reserved
  // 0x0f0 : Data signal of index_weights_10_V
  //         bit 15~0 - index_weights_10_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0f4 : reserved
  // 0x0f8 : Data signal of index_weights_11_V
  //         bit 15~0 - index_weights_11_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x0fc : reserved
  // 0x100 : Data signal of index_weights_12_V
  //         bit 15~0 - index_weights_12_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x104 : reserved
  // 0x108 : Data signal of index_weights_13_V
  //         bit 15~0 - index_weights_13_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x10c : reserved
  // 0x110 : Data signal of index_weights_14_V
  //         bit 15~0 - index_weights_14_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x114 : reserved
  // 0x118 : Data signal of index_weights_15_V
  //         bit 15~0 - index_weights_15_V[15:0] (Read/Write)
  //         others   - reserved
  // 0x11c : reserved
  // 0x120 : Data signal of blue_offset
  //         bit 7~0 - blue_offset[7:0] (Read/Write)
  //         others  - reserved
  // 0x124 : reserved
  // 0x128 : Data signal of threshold
  //         bit 7~0 - threshold[7:0] (Read/Write)
  //         others  - reserved
  // 0x12c : reserved
  // 0x130 : Data signal of result_V
  //         bit 15~0 - result_V[15:0] (Read)
  //         others   - reserved
  // 0x134 : Control signal of result_V
  //         bit 0  - result_V_ap_vld (Read/COR)
  //         others - reserved
  // 0x138 : Data signal of result_y_V
  //         bit 15~0 - result_y_V[15:0] (Read)
  //         others   - reserved
  // 0x13c : Control signal of result_y_V
  //         bit 0  - result_y_V_ap_vld (Read/COR)
  //         others - reserved
  // (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

  //#define XTRAIL_DETECTION_AXILITES_ADDR_AP_CTRL                 0x000
  //#define XTRAIL_DETECTION_AXILITES_ADDR_GIE                     0x004
  //#define XTRAIL_DETECTION_AXILITES_ADDR_IER                     0x008
  //#define XTRAIL_DETECTION_AXILITES_ADDR_ISR                     0x00c
  //#define XTRAIL_DETECTION_AXILITES_ADDR_WIDTH_V_DATA            0x010
  //#define XTRAIL_DETECTION_AXILITES_BITS_WIDTH_V_DATA            12
  //#define XTRAIL_DETECTION_AXILITES_ADDR_HEIGHT_V_DATA           0x018
  //#define XTRAIL_DETECTION_AXILITES_BITS_HEIGHT_V_DATA           12
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_0_DATA      0x020
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_0_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_1_DATA      0x028
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_1_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_2_DATA      0x030
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_2_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_3_DATA      0x038
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_3_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_4_DATA      0x040
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_4_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_5_DATA      0x048
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_5_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_6_DATA      0x050
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_6_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_7_DATA      0x058
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_7_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_8_DATA      0x060
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_8_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_9_DATA      0x068
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_9_DATA      16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_10_DATA     0x070
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_10_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_11_DATA     0x078
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_11_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_12_DATA     0x080
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_12_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_13_DATA     0x088
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_13_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_14_DATA     0x090
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_14_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_TABLE_15_DATA     0x098
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_TABLE_15_DATA     16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_0_V_DATA  0x0a0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_0_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_1_V_DATA  0x0a8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_1_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_2_V_DATA  0x0b0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_2_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_3_V_DATA  0x0b8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_3_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_4_V_DATA  0x0c0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_4_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_5_V_DATA  0x0c8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_5_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_6_V_DATA  0x0d0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_6_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_7_V_DATA  0x0d8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_7_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_8_V_DATA  0x0e0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_8_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_9_V_DATA  0x0e8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_9_V_DATA  16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_10_V_DATA 0x0f0
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_10_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_11_V_DATA 0x0f8
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_11_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_12_V_DATA 0x100
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_12_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_13_V_DATA 0x108
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_13_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_14_V_DATA 0x110
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_14_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_INDEX_WEIGHTS_15_V_DATA 0x118
  //#define XTRAIL_DETECTION_AXILITES_BITS_INDEX_WEIGHTS_15_V_DATA 16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_BLUE_OFFSET_DATA        0x120
  //#define XTRAIL_DETECTION_AXILITES_BITS_BLUE_OFFSET_DATA        8
  //#define XTRAIL_DETECTION_AXILITES_ADDR_THRESHOLD_DATA          0x128
  //#define XTRAIL_DETECTION_AXILITES_BITS_THRESHOLD_DATA          8
  //#define XTRAIL_DETECTION_AXILITES_ADDR_RESULT_V_DATA           0x130
  //#define XTRAIL_DETECTION_AXILITES_BITS_RESULT_V_DATA           16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_RESULT_V_CTRL           0x134
  //#define XTRAIL_DETECTION_AXILITES_ADDR_RESULT_Y_V_DATA         0x138
  //#define XTRAIL_DETECTION_AXILITES_BITS_RESULT_Y_V_DATA         16
  //#define XTRAIL_DETECTION_AXILITES_ADDR_RESULT_Y_V_CTRL         0x13c

  uint32_t AP_CTRL;
  uint32_t GIE;
  uint32_t IER;
  uint32_t ISR;
  uint32_t WIDTH;
  uint32_t WIDTH_RESERVED;
  uint32_t HEIGHT;
  uint32_t HEIGHT_RESERVED;
  struct
  {
    uint32_t data;
    uint32_t reserved;
  } INDEX_TABLE[16];
  struct
  {
    uint32_t data;
    uint32_t reserved;
  } INDEX_WEIGHTS[16];
  uint32_t BLUE_OFFSET;
  uint32_t BLUE_OFFSET_RESERVED;
  uint32_t THRESHOLD;
  uint32_t THRESHOLD_RESERVED;
  uint32_t RESULT_X;
  uint32_t RESULT_X_VALID;
  uint32_t RESULT_Y;
  uint32_t RESULT_Y_VALID;
};

} // end namespace ip
} // end namespace fpga
} // end namespace trail_detection

#endif //_TRAIL_DETECTION_H
