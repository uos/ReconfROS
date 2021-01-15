# BSD 3-Clause License
#
# Copyright (c) 2018, Xilinx
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Rebuild HLS IP from source
set current_dir [pwd]
cd ip/hls/
# get list of IP from folder names
set ip [glob -type d *]
# Check and build each IP
foreach item $ip {
   if {[catch { glob -directory ${item}/solution1/impl/ip/ *.zip} zip_file]} {
# Build IP only if a packaged IP does not exist
      puts "Building $item IP"
      if [file exist $item/script.tcl] {
         exec vivado_hls -f $item/script.tcl
      } else {
         exec vivado_hls $item/solution1/script.tcl
      }
   } else {
# Skip IP when a packaged IP exists in ip directory
      puts "Skipping building $item"
   }
   unset zip_file
# Testing the built IP
   puts "Checking $item"
   set fd [open ${item}/solution1/syn/report/${item}_csynth.rpt r]
   set timing_flag 0
   set latency_flag 0
   while { [gets $fd line] >= 0 } {
# Check whether the timing has been met
    if [string match {+ Timing (ns): } $line]  { 
      set timing_flag 1
      set latency_flag 0
      continue
    }
    if {$timing_flag == 1} {
      if [regexp {[0-9]+} $line]  {
        set period [regexp -all -inline {[0-9]*\.[0-9]*} $line]
        lassign $period target estimated uncertainty
        if {$target < $estimated} {
            puts "ERROR: Estimated clock period $estimated > target $target."
            puts "ERROR: Revise $item to be compatible with Vivado_HLS."
            exit 1
        }
      }
    }
# Check whether the II has been met
    if [string match {+ Latency (clock cycles): } $line]  { 
      set timing_flag 0
      set latency_flag 1
      continue
    }
    if {$latency_flag == 1} {
      if [regexp {[0-9]+} $line]  {
        set interval [regexp -all -inline {[0-9]+} $line]
        lassign $interval l iteration achieved target
        if {$achieved != $target} {
            puts "ERROR: Achieved II $achieved != target $target for loop $l."
            puts "ERROR: Revise $item to be compatible with Vivado_HLS."
            exit 1
        }
      }
    }
# Testing ends
    if [string match {== Utilization Estimates} $line]  { 
       unset -nocomplain timing_flag latency_flag period interval
       break
    }
   }
   unset fd
}
cd $current_dir
puts "HLS IP builds complete"
