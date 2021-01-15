/*
 * pipeline_ros_wrapper.h
 *
 *  Created on: Jun 16, 2020
 *      Author: Julian Gaal
 */

#ifndef _FPGA_SUBSCRIBER_H
#define _FPGA_SUBSCRIBER_H

#include <ros/node_handle.h>
#include <trail_detection/fpga/fpga.h>
#include <trail_detection/ros_wrapper/webcam.h>
#include <trail_detection/ros_wrapper/params.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

namespace trail_detection
{
namespace ros_wrapper
{

// TODO use boost::optional to prevent webcam AND subscriber

/**
 * PipeLineROSWrapper wraps all FPGA and buffer operations into an easy to use ROS node
 */
class PipelineROSWrapper
{
private:
  /// width of image
  int width_;
  /// heigth of image
  int height_;
  /// ROS Node handle
  ros::NodeHandle &nodehandle_;
  /// FPGA instance
  trail_detection::fpga::FPGA &fpga_;
  /// input buffer
  Buffer &inbuffer_;
  /// output buffer
  Buffer &outbuffer_;
  /// optional image subscriber
  boost::optional<ros::Subscriber> img_sub_;
  /// image publisher
  ros::Publisher img_pub_;
  /// angle publisher
  ros::Publisher angle_pub_;
  /**
   * output of pipeline as cv::Mat for debugging
   * It writes directly into output buffer
   */
  cv::Mat cv_output_;
  /**
   * input of pipeline cv::Mat: used for reading from webcam
   * cv_input_ writes directly into inbuffer_, from where it is sent to fpga
   */
  cv::Mat cv_input_;

  /// wether of not to enable debug image publishing
  Debug debug_;

  /**
   * Private PipelineROSWrapper constructor
   * @param nodehandle ROS node handle
   * @param params Params with custom parameters
   * @param inbuffer input buffer
   * @param outbuffer output buffer
   */
  PipelineROSWrapper(ros::NodeHandle &nodehandle, FPGAParams &params, Buffer &inbuffer,
                     Buffer &outbuffer);

  /**
   * Initializes publisher(s)
   */
  void init_pub();

public:
  /**
   * PipeLineROSWrapper factory method: ensured only one instance of the object
      * @param nodehandle ROS node handle
      * @param params Params with custom parameters
      * @param inbuffer input buffer
      * @param outbuffer output buffer
   * @return PipelineROSWrapper instance
   */
  static PipelineROSWrapper &
  init(ros::NodeHandle &nodehandle, FPGAParams &params, Buffer &inbuffer,
       Buffer &outbuffer);

  /// default destructor
  ~PipelineROSWrapper() = default;

  /**
   * Image callback for subscriber
   * @param image image subscribed at specific topic
   */
  void callback(const sensor_msgs::ImageConstPtr &image);

  /**
   * Target angle publisher
   * @param angle target angle
   */
  void publish_angle(float angle);

  /**
   * Publishes output of fpga operation as output
   * @param sum_x_weight weighted x position in image (output from fpga)
   * @param sum_y_weight weighted y position in image (output from fpga)
   */
  void publish_debug_img(float sum_x_weight, float sum_y_weight);

  /**
   * Initialize subscriber
   * @param topic topic to subscribe to
   */
  void init_sub(std::string topic);

  /**
   * runs hardware pipeline
   * @return tuple of results (weighted x/y position in camera)
   */
  std::tuple<float, float> run_pipeline();

  /**
   * Reads frame from webcam object
   * @param webcam OpenCV VideoCapture device
   * @return true if successful, false if couldn't read from webcam
   */
  bool read_frame(trail_detection::ros_wrapper::Webcam &webcam);
};

} // end namespace ros_wrapper
} // end namespace trail_detection

#endif //_FPGA_SUBSCRIBER_H
