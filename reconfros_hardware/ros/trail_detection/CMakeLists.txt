cmake_minimum_required(VERSION 3.0.2)
project(trail_detection)

set(BINFILE "FastSenseMS1.bin")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

if (NOT EXISTS "/lib/firmware/${BINFILE}")
    message(FATAL_ERROR "Please run 'make bin2firmware' in repo root to ensure binfile can be found at /lib/firmware")
endif ()

if (NOT EXISTS "/usr/include/libxlnk_cma.h")
    message(FATAL_ERROR "This node MUST be compiled on xilinx board with modified Xilinx Kernel")
endif ()

message(STATUS "Compiling on Xilinx board")
message(STATUS "Binfile loaded to /lib/firmware/${BINFILE}")

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(fpga
        src/fpga/fpga.cpp
        src/fpga/trail_detection.cpp
        src/buffer.cpp
        )

add_library(ros_wrapper
        src/buffer.cpp
        src/ros_wrapper/misc.cpp
        src/ros_wrapper/webcam.cpp
        src/ros_wrapper/params.cpp
        src/ros_wrapper/pipeline_ros_wrapper.cpp
        src/buffer.cpp)

target_link_libraries(ros_wrapper fpga)

add_executable(${PROJECT_NAME}_webcam_node src/${PROJECT_NAME}_webcam_node.cpp)
add_executable(${PROJECT_NAME}_eval_node src/${PROJECT_NAME}_eval_node.cpp)

target_link_libraries(${PROJECT_NAME}_webcam_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        cma
        ros_wrapper
        )

target_link_libraries(${PROJECT_NAME}_eval_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        cma
        ros_wrapper
        )
