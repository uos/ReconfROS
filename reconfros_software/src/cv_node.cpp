// ROS stuff
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <reconfros_software/CVConfig.h>

// OpenCV stuff
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>

ros::Publisher target_pub_;

image_transport::Publisher direction_pub_;
image_transport::Publisher contour_pub_;
image_transport::Publisher threshold_pub_;
image_transport::Publisher gauss_pub_;
image_transport::Publisher gray_pub_;

image_transport::Publisher morph_pub_;

reconfros_software::CVConfig config_;

std::mutex mutex_;

// direction control
double left_x = -1000, right_x = -1000; // -1000 as a "not initiallised" value
bool left_not_found = true, right_not_found = true;

void dyn_cfg_callback(reconfros_software::CVConfig& config, uint32_t level)
{
  mutex_.lock();
  config_ = config;

  config_.gauss_kernel_size += (config_.gauss_kernel_size + 1) % 2;
  config_.morph_kernel_size += (config_.morph_kernel_size + 1) % 2;
  mutex_.unlock();

  ROS_INFO_STREAM("Parameters have changed:\n"
                  << "\t       Use HSV Color Space:\t" << config.use_hsv                    << "\n"
                  << "\t                       Dry:\t" << config.dry                        << "\n"
                  << "\t  Difference Green to Blue:\t" << config.difference_gb              << "\n"
                  << "\t         Gauss Kernel Size:\t" << config.gauss_kernel_size          << "\n"
                  << "\t            Gray Threshold:\t" << config.gray_thresh                << "\n"
                  << "\t              Use Contours:\t" << config.use_contours               << "\n"
                  << "\t        Average Block Size:\t" << config.avg_block_size             << "\n"
                  << "\t     Contour target Height:\t" << config.contour_target_height      << "\n"
                  << "\t      Contour Target Range:\t" << config.contour_target_range       << "\n"
                  << "\tNumber of Horizontal Lines:\t" << config.number_of_horizontal_lines << "\n"
                  << "\t                 Use Morph:\t" << config.use_morph                  << "\n"
                  << "\t         Morph Kernel Size:\t" << config.morph_kernel_size);

}

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO_STREAM("Image received!");

  // Try to convert the received image to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  mutex_.lock();

  cv::Mat rgb_image = cv_ptr->image.clone();

  cv::GaussianBlur(cv_ptr->image, rgb_image, cv::Size(config_.gauss_kernel_size, config_.gauss_kernel_size), 0.0, 0.0);
  
  // Only considering gray values for the rest of the pipeline
  cv::Mat gray;

  if (config_.use_hsv)
  {
    cv::Mat hsv_image;
    cv::Mat hsv[3];

    cv::cvtColor(rgb_image, hsv_image, CV_BGR2HSV);
    cv::split(hsv_image, hsv);

    for (auto i = 0u; i < hsv_image.rows; ++i)
    {
      for (auto j = 0u; j < hsv_image.cols; ++j)
      {
        if (hsv[0].at<uchar>(i, j) >= config_.h_lower_bound && hsv[0].at<uchar>(i, j) <= config_.h_upper_bound)
        {
          hsv[2].at<uchar>(i, j) = 0;
        }
      }
    }

    gray = hsv[2];
  }
  else
  {

    /********** Trail Border Detection **********/

    cv::cvtColor(rgb_image, gray, CV_BGR2GRAY);

    // Set plants or gras to black
    for (auto i = 0u; i < rgb_image.rows; i++)
    {
      for (auto j = 0u; j < rgb_image.cols; j++)
      {
        auto& pixel = rgb_image.at<cv::Vec3b>(i, j);

        if ((pixel[1] >= pixel[0] || config_.dry) && pixel[1] >= pixel[2] + config_.difference_gb)
        {
          gray.at<uchar>(i, j) = 0;
        }
      }
    }
  }

  // Enhance contrast
  cv::Mat contrast;
  cv::convertScaleAbs(gray, contrast, 1.6);

  // Apply binary threshold
  cv::Mat thresh;
  cv::threshold(contrast, thresh, config_.gray_thresh, 255, CV_THRESH_BINARY);

  /********** Object Extraction **********/

  cv::Mat direction = cv_ptr->image.clone();
  int w = cv_ptr->image.cols, h = cv_ptr->image.rows;

  double target_x = 0.0;
  int target_y = (1.0 - config_.contour_target_height) * h;
  double target_delta = 0.0;

  cv::Mat morph;

  if (config_.use_morph)
  {
    //cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((40) + 1, (40)+1));  // 4 * 4  // 30
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(config_.morph_kernel_size, config_.morph_kernel_size));
    cv::erode(thresh, morph, morph_kernel);
    cv::dilate(morph, morph, morph_kernel);
    cv::dilate(morph, morph, morph_kernel);
    cv::erode(morph, morph, morph_kernel);
  }
  else
  {
    morph = thresh;
  }

  /****************** Contours //DEPRECATED\\ ******************/
  // Uses cv::findContours to detect the border of the path.
  // The target is the middle between the left and right edge of the Path

  if (config_.use_contours)
  {
    cv::Mat contour_vis = cv::Mat::zeros(cv_ptr->image.size(), CV_8UC1);

    // Calculating parent contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(morph, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Visualize largest contour
    if (contours.size() != 0)
    {
      auto largest = cv::contourArea(contours[0]);
      auto largest_id = 0;

      for (auto i = 1u; i < contours.size(); ++i)
      {
        auto area = cv::contourArea(contours[i]);

        if (area > largest)
        {
          largest = area;
          largest_id = i;
        }
      }

      cv::drawContours(contour_vis, contours, largest_id, 255, 1);

      /********* direction control *********/

      if (left_x == -1000) // first iteration
      {
        left_x = w / 4;
        right_x = w * 3 / 4;
        left_not_found = 0;
        right_not_found = 0;
      }

      auto& contour = contours[largest_id];
      int delta_x = config_.contour_target_range * w;

      double left_sum = 0, right_sum = 0;
      int left_count = 0, right_count = 0;
      for (auto& point : contour)
      {
        if (abs(point.y - target_y) < 10)
        {
          double l_dist = abs(point.x - left_x);
          double r_dist = abs(point.x - right_x);
          if ((l_dist < delta_x || left_not_found && point.x <= w / 2) && l_dist <= r_dist)
          {
            left_sum += point.x;
            left_count++;
          }
          else if ((r_dist < delta_x || right_not_found && point.x > w / 2) && r_dist <= l_dist)
          {
            right_sum += point.x;
            right_count++;
          }
        }
      }

      if (left_count > 0)
      {
        left_x = left_sum / left_count;
      }
      left_not_found = left_count == 0;

      if (right_count > 0)
      {
        right_x = right_sum / right_count;
      }
      right_not_found = right_count == 0;

      if (right_x < left_x)
      {
        std::swap(left_x, right_x);
      }

      target_x = (left_x + right_x) / 2.0;
      target_delta = static_cast<double>(target_x - w / 2) / (w / 2);

      // Contour visualization
      cv::cvtColor(contour_vis, cv_ptr->image, CV_GRAY2RGB);
      contour_pub_.publish(cv_ptr->toImageMsg());

      cv::drawContours(direction, contours, largest_id, cv::Scalar(0, 0, 0), 3);
    }

    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(target_x, target_y), cv::Scalar(255, 255, 255), 12, cv::LineTypes::LINE_8, 0, 0.05);
    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(target_x, target_y), cv::Scalar(255, 0, 0), 4, cv::LineTypes::LINE_8, 0, 0.05);
  }
  else
  {
    left_x = right_x = -1000;
    left_not_found = right_not_found = true;
  }

  /****************** Average x-Coordinate ******************/
  // Calculates the average of all Pixels that belong to the Path
  // The target is simply that average in a certain region

  if (config_.use_x_average)
  {
    uint64_t sumpos = 0;
    uint64_t cnt = 0;

    double sum_x_weighted = 0.0;
    double sum_y_weighted = 0.0;
    int num_lines = 10;
    int line_counter_weighted = 0;
    double weight_sum = 0.0;
    int point_diff = ceil((double) h / (double) (num_lines+ 1.0));

    for (int y = 0; y < morph.rows; y++)
    {
      //if(y % 64 == 0)
      if (y % config_.avg_block_size == 0)
      {
        if (cnt)
        {
          //circle(direction, cv::Point(sumpos / cnt, y), 12, cv::Scalar(255, 255, 255), -1);
          //circle(direction, cv::Point(sumpos / cnt, y), 8, cv::Scalar(0, 255, 0), -1);
        
          double sin_weight = sin((double) ((double) line_counter_weighted / ((double) num_lines - 1.0)) * M_PI / 2);
                sum_x_weighted += sin_weight * (sumpos / cnt); // weighted add
                sum_y_weighted += sin_weight * (y);
                weight_sum += sin_weight;
        }
        line_counter_weighted++;
        sumpos = 0;
        cnt = 0;
      }
      for (int x = 0; x < morph.cols; x++)
      {
        if (morph.at<uchar>(y, x) == 255)
        {
          sumpos += x;
          cnt++;
        }
      }
    }
    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(sum_x_weighted / weight_sum, target_y), cv::Scalar(255, 255, 255), (int) 12, cv::LineTypes::LINE_8, 0, 0.05);
    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(sum_x_weighted / weight_sum, target_y), cv::Scalar(0, 0, 255), (int) 4, cv::LineTypes::LINE_8, 0, 0.05);
  }

  /****************** Pixel Count ******************/
  // Counts, how many Pixels are on the left and right side of the middle.
  // The target is calculated from the ratio of those counts

  if (config_.use_pixel_cnt)
  {
    unsigned int row_step_size = morph.rows / config_.number_of_horizontal_lines;
    unsigned int col_middle_left = morph.cols / 2;
    unsigned int col_middle_right = col_middle_left + 1;

    double difference = 0.0;

    for (auto row = 0u; row < morph.rows; row += row_step_size)
    {
      int left_count = 0;
      int right_count = 0;

      for (auto col_step = 0u; col_step < col_middle_left; ++col_step)
      {
        auto col_left = col_middle_left - col_step;
        auto col_right = col_middle_right + col_step;

        if (morph.at<uchar>(row, col_left) != 0)
        {
          ++left_count;
        }

        if (morph.at<uchar>(row, col_right) != 0)
        {
          ++right_count;
        }
      }

      int sum = left_count + right_count;

      if (sum > 0)
      {
        difference += (right_count - left_count) / 2.0 / sum;
      }

    }

    target_delta = difference / config_.number_of_horizontal_lines;

    target_x = (target_delta * w / 2) + w / 2;

    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(target_x, target_y), cv::Scalar(255, 255, 255), 12, cv::LineTypes::LINE_8, 0, 0.05);
    cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(target_x, target_y), cv::Scalar(255, 0, 255), 4, cv::LineTypes::LINE_8, 0, 0.05);
  }

  std_msgs::Float32 td;
  td.data = target_delta;
  target_pub_.publish(td);

  /********** Pipeline Visualization **********/

  // center guide
  cv::line(direction, cv::Point(w / 2, 0), cv::Point(w / 2, h), cv::Scalar(128, 128, 128), 2);

  // Morph visualization
  cv::cvtColor(morph, cv_ptr->image, CV_GRAY2RGB);
  morph_pub_.publish(cv_ptr->toImageMsg());

  cv_ptr->image = direction;
  direction_pub_.publish(cv_ptr->toImageMsg()); // this has to be published first for some reason, otherwise there won't be color

  cv::cvtColor(gray, cv_ptr->image, CV_GRAY2RGB);
  gray_pub_.publish(cv_ptr->toImageMsg());

  cv_ptr->image = rgb_image;
  gauss_pub_.publish(cv_ptr->toImageMsg());

  cv::cvtColor(thresh, cv_ptr->image, CV_GRAY2RGB);
  threshold_pub_.publish(cv_ptr->toImageMsg());

  mutex_.unlock();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_node");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<reconfros_software::CVConfig> server;
  dynamic_reconfigure::Server<reconfros_software::CVConfig>::CallbackType f;
  f = boost::bind(&dyn_cfg_callback, _1, _2);
  server.setCallback(f);

  image_transport::ImageTransport it(n);

  // Subscribes on the camera topic
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
  gray_pub_ = it.advertise("/gray", 1);
  gauss_pub_ = it.advertise("/gauss", 1);
  threshold_pub_ = it.advertise("/threshold", 1);
  contour_pub_ = it.advertise("/contours", 1);
  direction_pub_ = it.advertise("/direction", 1);

  morph_pub_ = it.advertise("/morph", 1);

  // Publishes navigation commands to the robot system
  target_pub_ = n.advertise<std_msgs::Float32>("/target_delta", 1);

  ros::spin();
}
