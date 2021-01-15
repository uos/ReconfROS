/*
 * webcam.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/ros_wrapper/webcam.h>

using namespace trail_detection::ros_wrapper;

cv::VideoCapture &Webcam::cap()
{
  return cap_;
}

Webcam &Webcam::init(int device_id, int width, int height)
{
  static Webcam cam(device_id, width, height);
  return cam;
}

Webcam::Webcam(int device_id, int width, int height)
    : cap_(cv::VideoCapture())
{
  cap_.open(device_id);
  if (!cap_.isOpened())
  {
    throw std::runtime_error("ERROR! Unable to open camera\n");
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
}
