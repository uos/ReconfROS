#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <dynamic_reconfigure/server.h>
#include <reconfros_software/NAVConfig.h>

#include <mutex>

bool mode_reorientate = false;

reconfros_software::NAVConfig config_;

ros::Publisher pub;
geometry_msgs::Twist twist;
geometry_msgs::Vector3 angular;
geometry_msgs::Vector3 linear;

std::mutex mutex_;

void dyn_cfg_callback(reconfros_software::NAVConfig& config, uint32_t level)
{
  mutex_.lock();
  config_ = config;
  mutex_.unlock();
  ROS_INFO_STREAM("Parameter changed");
}

void callback(const std_msgs::Float32ConstPtr& v)
{
  std_msgs::Float32 value;
  value.data = -v->data;

  ROS_INFO_STREAM("Value: " << value.data);
  //If value inside DRIVE_THRESHOLD and not currently reorientating -> Drive forward
  if ((abs(value.data) < config_.drive_threshold) && !mode_reorientate)
  {
    //Robot is oriented correctly and can drive forward
    linear.x = config_.linear_speed;
    angular.z = std::min(config_.angular_speed_factor * value.data, config_.angular_speed_max);
  }
  else if (abs(value.data) > config_.drive_threshold && !mode_reorientate)
  {
    //Robot is not oriented correctly and should reorientate
    mode_reorientate = true;
    linear.x = std::max(linear.x - config_.stop_step, 0.0);
    angular.z = std::min(config_.angular_speed_factor * value.data, config_.angular_speed_max);
  }
  else if (mode_reorientate)
  {
    //Robot is currently reorienting and therfore not driving forward
    linear.x = std::max(linear.x - config_.stop_step, 0.0);
    angular.z = config_.angular_speed_factor * value.data;

    //Reorientation complete
    if (value.data < config_.resume_threshold)
    {
      mode_reorientate = false;
    }
  }

  twist.angular = angular;
  twist.linear = linear;

  pub.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "naviagtion");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<reconfros_software::NAVConfig> server;
  dynamic_reconfigure::Server<reconfros_software::NAVConfig>::CallbackType f;
  f = boost::bind(&dyn_cfg_callback, _1, _2);
  server.setCallback(f);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ros::Subscriber sub = n.subscribe<std_msgs::Float32>("target_angle", 1, callback);


  ros::spin();
}
