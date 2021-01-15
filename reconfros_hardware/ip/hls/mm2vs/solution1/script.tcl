############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2019 Xilinx, Inc. All Rights Reserved.
############################################################
open_project mm2vs
set_top mm2vs
add_files mm2vs/mm2vs.h
add_files mm2vs/mm2vs.cpp
add_files -tb test.jpg -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb mm2vs/mm2vs_tb.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1"
set_part {xc7z020-clg400-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl vhdl
#source "./mm2vs/solution1/directives.tcl"
#csim_design
csynth_design
#cosim_design
export_design -format ip_catalog
