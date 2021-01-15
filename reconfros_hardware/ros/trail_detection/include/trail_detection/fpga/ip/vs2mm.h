/*
 * vs2mm.h
 * 
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#ifndef _VS2MM_H
#define _VS2MM_H

#include <cstdint>

namespace trail_detection
{
namespace fpga
{
namespace ip
{

/// base address of vs2mm ip block found in Vivado address editor
constexpr uint32_t VS2MM_BASE_ADDRESS = 0x43c20000;

/**
 * This struct represents the memory architecture of the vs2mm ip block found in
 * <root>/ip/hls/vs2mm. It is used for virtual memory mapping with mmap
 *
 * the vs2mm ip block turn Vivado image stream into generic buffer that can be used to debug
 * the algorithms effects
 */
struct VS2MM
{
  // ==============================================================
  // Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2019.2 (64-bit)
  // Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
  // ==============================================================
  // regs
  // 0x00 : Control signals
  //        bit 0  - ap_start (Read/Write/COH)
  //        bit 1  - ap_done (Read/COR)
  //        bit 2  - ap_idle (Read)
  //        bit 3  - ap_ready (Read)
  //        bit 7  - auto_restart (Read/Write)
  //        others - reserved
  // 0x04 : Global Interrupt Enable Register
  //        bit 0  - Global Interrupt Enable (Read/Write)
  //        others - reserved
  // 0x08 : IP Interrupt Enable Register (Read/Write)
  //        bit 0  - Channel 0 (ap_done)
  //        bit 1  - Channel 1 (ap_ready)
  //        others - reserved
  // 0x0c : IP Interrupt Status Register (Read/TOW)
  //        bit 0  - Channel 0 (ap_done)
  //        bit 1  - Channel 1 (ap_ready)
  //        others - reserved
  // 0x10 : Data signal of mem
  //        bit 31~0 - mem[31:0] (Read/Write)
  // 0x14 : reserved
  // 0x18 : Data signal of width_V
  //        bit 11~0 - width_V[11:0] (Read/Write)
  //        others   - reserved
  // 0x1c : reserved
  // 0x20 : Data signal of height_V
  //        bit 11~0 - height_V[11:0] (Read/Write)
  //        others   - reserved
  // 0x24 : reserved
  // 0x28 : Data signal of stride_V
  //        bit 11~0 - stride_V[11:0] (Read/Write)
  //        others   - reserved
  // 0x2c : reserved
  // (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

  //#define XVS2MM_REGS_ADDR_AP_CTRL       0x00
  //#define XVS2MM_REGS_ADDR_GIE           0x04
  //#define XVS2MM_REGS_ADDR_IER           0x08
  //#define XVS2MM_REGS_ADDR_ISR           0x0c
  //#define XVS2MM_REGS_ADDR_MEM_DATA      0x10
  //#define XVS2MM_REGS_BITS_MEM_DATA      32
  //#define XVS2MM_REGS_ADDR_WIDTH_V_DATA  0x18
  //#define XVS2MM_REGS_BITS_WIDTH_V_DATA  12
  //#define XVS2MM_REGS_ADDR_HEIGHT_V_DATA 0x20
  //#define XVS2MM_REGS_BITS_HEIGHT_V_DATA 12
  //#define XVS2MM_REGS_ADDR_STRIDE_V_DATA 0x28
  //#define XVS2MM_REGS_BITS_STRIDE_V_DATA 12

  uint32_t AP_CTRL;
  uint32_t GIE;
  uint32_t IER;
  uint32_t ISR;
  uint32_t MEM;
  uint32_t MEM_RESERVED;
  uint32_t WIDTH;
  uint32_t WIDTH_RESERVED;
  uint32_t HEIGHT;
  uint32_t HEIGHT_RESERVED;
  uint32_t STRIDE;
  uint32_t STRIDE_RESERVED;
};

} // end namespace ip
} // end namespace fpga
} // end namespace trail_detection


#endif //_VS2MM_H
