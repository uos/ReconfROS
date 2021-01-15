/*
 * misc.cpp
 *
 *  Created on: Jun 12, 2020
 *      Author: Julian Gaal
 */

#include <trail_detection/ros_wrapper/misc.h>

float trail_detection::ros_wrapper::angle_from_result(int width, int height, float sum_x_weighted, float sum_y_weighted)
{
  {
    // Hardware already calculates sum_x, no need to convert to sum_x_weighted with sum_x / sum_weights like in
    // software solution
    float dir_vec_x = sum_x_weighted - width / 2.f;
    float dir_vec_y = sum_y_weighted - height;

    float center_vec_x = 0;
    float center_vec_y = -1;
    int sign = dir_vec_x < 0 ? -1 : 1;

    float length_dir = std::sqrt(std::pow(dir_vec_x, 2) + std::pow(dir_vec_y, 2));
    float length_center = std::sqrt(std::pow(center_vec_x, 2) + std::pow(center_vec_y, 2));

    float vec_dot = center_vec_x * dir_vec_x + center_vec_y * dir_vec_y;
    float angle = sign * std::acos(vec_dot / (length_dir * length_center)) * 180 / M_PI;

    return angle;
  }
}
