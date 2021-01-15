all: clean hls_ip block_design

hls_ip:
	vivado -mode batch -source build_ip.tcl -notrace

block_design:
	vivado -mode batch -source FastSenseMS1.tcl -notrace

copy_output:
	cp ./FastSenseMS1/FastSenseMS1.runs/impl_1/FastSenseMS1_design_wrapper.bit FastSenseMS1.bit
	cp ./FastSenseMS1/FastSenseMS1.srcs/sources_1/bd/FastSenseMS1_design/hw_handoff/FastSenseMS1_design.hwh FastSenseMS1.hwh

bit2bin: FastSenseMS1.bit
	python3 bit2bin.py FastSenseMS1.bit

bin2firmware: clean_firmware FastSenseMS1.bin
	cp FastSenseMS1.bin /lib/firmware/

clean_firmware:
	@if [ -f "/lib/firmware/FastSenseMS1.bin" ]; then rm /lib/firmware/FastSenseMS1.bin; fi

clean:
	rm -rf FastSenseMS1 *.jou *.log NA

doc:
	cd ros/trail_detection/ && doxygen Doxyfile && xdg-open doc/html/index.html
