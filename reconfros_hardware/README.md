# Hardware_MS1

## Prerequisites
* Linux: `sudo apt install libjpeg62 && wget http://security.ubuntu.com/ubuntu/pool/main/libp/libpng/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && sudo dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb`

* Make Vivado binaries available to `$PATH`: `source /tools/Xilinx/Vivado/2019.2/settings64.sh`

## Create projects

At first the HLS-IPs must be synthesized with the `build_ip.tcl` script. Then the Vivado project is created with the `FastSenseMS1.tcl` script. In the root directory is a Makefile to help with the creation:

`make hls_ip`: Build all HLS-IPs.

`make block_design`: Create the Vivado project from with `FastSenseMS1.tcl`. It is necessery that the HLS-IPs are already built.

`make copy_output`: Copy the bit and hwh files from the inside the project directory structure to the current directory.

`make all`: Build all HLS-IPs und create the Vivado project.

`make clean`: **Use with caution!** Delete the Vivado project folder and generated files. (**Not** the results from the HLS-IP builds)

`make clean_firmware`: **Use with caution!** removes binfile from `/lib/firmware`

`make bit2bin`: generates binfile from Xilinx bitstream with `bin2bit.py`

`make bin2firmware`: loads binfile into /lib/firmware (only on board)

`make doc`: generate Doxygen docs for ROS node

## Directory structure

- constraints: XDC-Contraints files for the project
- FastSenseMS1 (generated): Vivado project root directory
- ip: Contains IPs. Each subfolder not mentioned here is a IP
    - hls: Contains the HLS-IPs. Each subfolder is a HLS-IP
- NA (generated): Processing system summary
- src: HDL source files for the project
- ros: ROS node which communicates with FPGA. **NOTE**: can only be compiled on xilinx board due to modified kernel by Xilinx

## Create new HLS-IP

Open Vivado HLS and create a new project. Set the Location to `<gitrepo>/ip/hls/`. Click Next. Set the name (preferably the project name) of the top-function but do not specify source and testbench files (next page) at this time because the project folder is not created yet. On the last page, the solution name must be "solution1", the period must be 10 and "xc7z020clg400-1" is selected as part number.

Place new source files in the project folder. Testbench source files should end with "_tb"

Only the source files and the directive.tcl and script.tcl files are version controlled. The HLS-IP build process recreates the project that can be opened in the GUI.

Comment out (#) the lines with "csim_design" and "cosim_design" in `solution1/script.tcl` before a commit to avoid a very long build time when recreating the projects.

## Export Vivado project

Make sure the block design is validated without complaints and is not opened. Otherwise the project may not be recreated.

Click `File->Project->Write Tcl...`. Select the `FastSenseMS1.tcl` as output and check only "Copy sources to new project" and "Recreate Block Designs using Tcl". Click OK to create the Tcl file.

## ROS Node
* `trail_detection_webcam_node` reads data directly from webcam and processes on FPGA
* `trail_detection_eval_node` subscribes to data from `/camera/image_raw` and processes on FPGA. Should be used for evaluation of hardware

## Third Party Licenses

Parts of this project is based on the PYNQ base deign for the Pynq-Z2 under the BSD 3 License. This is indicated by a file header with the license text.
