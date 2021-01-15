############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2019 Xilinx, Inc. All Rights Reserved.
############################################################
open_project vs2mm
set_top vs2mm
add_files vs2mm/vs2mm.h
add_files vs2mm/vs2mm.cpp
add_files -tb vs2mm/vs2mm_tb.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb test.jpg -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1"
set_part {xc7z020-clg400-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl vhdl
#source "./vs2mm/solution1/directives.tcl"
#csim_design
csynth_design
#cosim_design -trace_level port -rtl vhdl
export_design -rtl vhdl -format ip_catalog
