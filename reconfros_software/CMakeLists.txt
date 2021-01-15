cmake_minimum_required(VERSION 2.8.3)
project(reconfros_software)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/CV.cfg
  cfg/NAV.cfg
  cfg/PIPELINE.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(xmllib SHARED IMPORTED)
set_property(TARGET xmllib PROPERTY IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libxml2.so")

add_executable(cv_node src/cv_node.cpp)
add_executable(navigation src/navigation.cpp)
add_executable(final_pipeline src/final_pipeline.cpp)

add_dependencies(navigation ${PROJECT_NAME}_gencfg)

target_link_libraries(cv_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
)
target_link_libraries(final_pipeline
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
)

target_link_libraries(navigation
  ${catkin_LIBRARIES}
)
