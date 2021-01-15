/*
 * trail_detection_eval_node.cpp
 *
 * This ROS node subscribes to /camera/image_raw, copies received image to fpga and processes it to detect the trail.
 * The resulting target angle is published to /target_angle.
 * The resulting image after the complete pipeline can be published to /hw_debug_img with the parameter Debug enabled.
 * All calculations happen in the subscribers callback function.
 *
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>

#include <cmath>
#include <array>
#include <numeric>
#include <algorithm>
#include <chrono>

#include <trail_detection/ros_wrapper/pipeline_ros_wrapper.h>
#include <trail_detection/fpga/ip/trail_detection.h>
#include <trail_detection/ros_wrapper/params.h>
#include <trail_detection/buffer.h>

#define CV_WIDTH 1280
#define CV_HEIGHT 720

namespace td = trail_detection;
namespace td_ros = trail_detection::ros_wrapper;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trail_detection_eval_node");
  ros::NodeHandle n;
  ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("/target_angle", 1000);
  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("/hw_debug_img", 1000);

  // setup input and output buffer: input 3 channel color image, output grayscale single channel picture
  td::Buffer input_buffer(CV_HEIGHT * CV_WIDTH * 3);
  td::Buffer output_buffer(CV_HEIGHT * CV_WIDTH);
  ROS_INFO_STREAM("Setup buffers");

  // set ip block sizes and block weights
  constexpr uint32_t point_diff = std::ceil((double) CV_HEIGHT / (double) (10 + 1.0));
  td::fpga::ip::td_block_sizes blocksizes{};
  std::fill(blocksizes.begin(), blocksizes.end(), point_diff);

  // top to bottom in image
  td::fpga::ip::td_block_weights weights{
      0.0,
      0.173648,
      0.342020,
      0.500000,
      0.642788,
      0.766044,
      0.866025,
      0.939693,
      0.984808,
      1.000000,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
  };

  // set parameters
  td_ros::FPGAParams params;
  params
      .setWidth(CV_WIDTH)
      .setHeight(CV_HEIGHT)
      .setBlueOffset(18)
      .setGrayThreshold(50)
      .setBlockSizes(blocksizes)
      .setBlockWeights(weights)
      .setBinfile("FastSenseMS1.bin")
      .setLength(0xffff)
      .setDebug(td::Debug::NO);

  td_ros::PipelineROSWrapper &pipeline = td_ros::PipelineROSWrapper::init(n, params, input_buffer, output_buffer);
  pipeline.init_sub("/camera/image_raw");

  ros::spin();

  return 0;
}

