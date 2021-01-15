/*
 * webcam.h
 *
 *  Created on: Jun 15, 2020
 *      Author: Julian
 */

#ifndef _WEBCAM_H
#define _WEBCAM_H

#include <opencv2/videoio.hpp>

namespace trail_detection
{
namespace ros_wrapper
{

/**
 * Webcam is a light wrapper around OpenCVs VideoCapture and allows for only one webcam instance
 */
class Webcam
{
private:
  /// Opencv capture device
  cv::VideoCapture cap_;

  /**
   * Webcam private constructor: opens webcam for reading and sets width and height
   * @param device_id - device id of webcam (0 for webcam)
   * @param width - width of image
   * @param height - height of image
   */
  Webcam(int device_id, int width, int height);

public:
  /// default constructor deleted
  Webcam() = delete;

  /// default destructor
  ~Webcam() = default;


  /// no copy constructor
  Webcam(const Webcam &) = delete;

  /// no assignment
  Webcam &operator=(const Webcam &) = delete;

  /**
   * Webcam factory method: opens webcam for reading and sets width and height
   * @param device_id - device id of webcam (0 for webcam)
   * @param width - width of image
   * @param height - height of image
   * @return initialized webcam
   */
  static Webcam &init(int device_id, int width, int height);

  /**
   * Access to underlying capture object
   * @return VideoCapture object reference
   */
  cv::VideoCapture &cap();
};

} // end namespace ros_wrapper
} // end namespace trail_detection


#endif //_WEBCAM_H
