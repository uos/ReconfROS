/*
 * pipeline_ros_wrapper.cpp
 *
 *  Created on: Jun 13, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/ros_wrapper/pipeline_ros_wrapper.h>
#include <trail_detection/ros_wrapper/misc.h>

#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <cassert>

using namespace trail_detection::ros_wrapper;

PipelineROSWrapper &
PipelineROSWrapper::init(ros::NodeHandle &nodehandle, FPGAParams &params, Buffer &inbuffer,
                         Buffer &outbuffer)
{
  static PipelineROSWrapper pipeline(nodehandle, params, inbuffer, outbuffer);
  return pipeline;
}

PipelineROSWrapper::PipelineROSWrapper(ros::NodeHandle &nodehandle, FPGAParams &params, Buffer &inbuffer,
                                       Buffer &outbuffer)
    : width_(params.width()), height_(params.height()),
      nodehandle_(nodehandle),
      fpga_(fpga::FPGA::init(params.binfile(), params.length())),
      inbuffer_(inbuffer),
      outbuffer_(outbuffer),
      img_sub_(),
      angle_pub_(),
      img_pub_(),
      cv_output_(cv::Mat(height_, width_, CV_8UC1, outbuffer_.getVirtual())),
      cv_input_(cv::Mat(height_, width_, CV_8UC3, inbuffer_.getVirtual())),
      debug_(params.debug())
{
  init_pub();

  fpga_.set_buffers(inbuffer_, outbuffer_);
  fpga_.set_param_registers(params.width(), params.height(), params.blueOffset(), params.grayThreshold());
  fpga_.set_weights(params.blockSizes(), params.blockWeights());
}

void PipelineROSWrapper::callback(const sensor_msgs::ImageConstPtr &image)
{
  std::copy(image->data.begin(), image->data.end(), reinterpret_cast<uint8_t *>(inbuffer_.getVirtual()));

  assert(image->width == width_ and image->height == height_);

  float sum_x_weight, sum_y_weight;
  std::tie(sum_x_weight, sum_y_weight) = run_pipeline();

  publish_angle(angle_from_result(width_, height_, sum_x_weight, sum_y_weight));

  publish_debug_img(sum_x_weight, sum_y_weight);
}

std::tuple<float, float> PipelineROSWrapper::run_pipeline()
{
  // Flush the cache so that the hardware can see the changes from the software
  inbuffer_.flush();
  fpga_.start_pipeline();
  fpga_.block_until_ready();

  float sum_x_weight = fpga_.get_result_x();
  float sum_y_weight = fpga_.get_result_y();

  // empty cache, software can see changes from hardware
  outbuffer_.invalidate();

  return std::forward_as_tuple(sum_x_weight, sum_y_weight);
}

void PipelineROSWrapper::publish_angle(float angle)
{
  std_msgs::Float32 msg;
  msg.data = angle;
  angle_pub_.publish(msg);
}

void PipelineROSWrapper::publish_debug_img(float sum_x_weight, float sum_y_weight)
{
  if (debug_ == YES)
  {
    sensor_msgs::Image img;
    img.header.stamp = ros::Time::now();
    img.header.frame_id = "idk";
    img.height = height_;
    img.width = width_;
    img.step = width_;
    img.encoding = "mono8";
    img.data.resize(width_ * height_);
    cv::circle(cv_output_, cv::Point(sum_x_weight, sum_y_weight), 25, cv::Scalar(128), -1);
    uint8_t *output_buffer_to_img = reinterpret_cast<uint8_t *>(outbuffer_.getVirtual());
    std::copy(output_buffer_to_img, output_buffer_to_img + (width_ * height_), img.data.begin());
    img_pub_.publish(img);
  }
}

void PipelineROSWrapper::init_pub()
{
  angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/target_angle", 1000);

  if (debug_ == YES)
  {
    img_pub_ = nodehandle_.advertise<sensor_msgs::Image>("/hw_debug_img", 10);
  }
}

void PipelineROSWrapper::init_sub(std::string topic)
{
  img_sub_ = nodehandle_.subscribe<sensor_msgs::Image>(topic, 10, &PipelineROSWrapper::callback, this);
}

bool PipelineROSWrapper::read_frame(Webcam &webcam)
{
  webcam.cap().read(cv_input_);
  return not cv_input_.empty();
}
