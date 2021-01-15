############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2019 Xilinx, Inc. All Rights Reserved.
############################################################
open_project trail_detection
set_top trail_detection
add_files trail_detection/trail_detection.h
add_files trail_detection/trail_detection.cpp
add_files -tb trail_detection/reference.png -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb test.jpg -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb trail_detection/trail_detection_tb.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1"
set_part {xc7z020-clg400-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl vhdl
#source "./trail_detection/solution1/directives.tcl"
#csim_design
csynth_design
#cosim_design -trace_level port -rtl vhdl
export_design -rtl vhdl -format ip_catalog
