// ROS stuff
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <reconfros_software/PIPELINEConfig.h>

// OpenCV stuff
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>

#define TIME_MEASUREMENT 0
#define DEBUG 1

ros::Publisher target_pub_;

image_transport::Publisher direction_pub_;
image_transport::Publisher threshold_pub_;
image_transport::Publisher morph_pub_;
image_transport::Publisher overlay_pub_;

reconfros_software::PIPELINEConfig config_;

std::mutex mutex_;

#if TIME_MEASUREMENT == 1
    ros::WallTime start_, end_;
    double time_sum_ = 0.0;
    int time_counter_ = 0;
    double time_maximum_ = 0.0;
#endif

void dyn_cfg_callback(reconfros_software::PIPELINEConfig &config, uint32_t level)
{
    mutex_.lock();
    config_ = config;

    config_.gauss_kernel_size += (config_.gauss_kernel_size + 1) % 2;
    config_.morph_kernel_size_closing += (config_.morph_kernel_size_closing + 1) % 2;
    config_.morph_kernel_size_opening += (config_.morph_kernel_size_opening + 1) % 2;

    #if TIME_MEASUREMENT == 1
        ROS_INFO("Restarting time counter.");
        time_counter_ = 0;
        time_sum_ = 0.0;
        time_maximum_ = 0.0;
    #endif

    mutex_.unlock();

    ROS_INFO_STREAM("Parameters have changed:\n"
                            << "\t                     Scale:\t" << config.scale                        << "\n"
                            << "\t                       Dry:\t" << config.dry                          << "\n"
                            << "\t  Difference Green to Blue:\t" << config.difference_gb                << "\n"
                            << "\t         Gauss Kernel Size:\t" << config.gauss_kernel_size            << "\n"
                            << "\t            Gray Threshold:\t" << config.gray_thresh                  << "\n"
                            << "\t                Num Points:\t" << config.num_points                   << "\n"
                            << "\t                 Use Morph:\t" << config.use_morph                    << "\n"
                            << "\t Closing Morph Kernel Size:\t" << config.morph_kernel_size_closing    << "\n"
                            << "\t Opening Morph Kernel Size:\t" << config.morph_kernel_size_opening);
}

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    #if TIME_MEASUREMENT == 1
        start_ = ros::WallTime::now();
    #endif

    // Try to convert the received image to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mutex_.lock();

    // Resize image
    cv::Mat input_image = cv_ptr->image.clone();
    cv::Mat rgb_image;

    int w = (cv_ptr->image.cols * config_.scale / 100), h = (cv_ptr->image.rows * config_.scale / 100); // Image width and height
    cv::resize(input_image, rgb_image, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);

    /********** Trail Border Detection **********/
    // Separate trail(should turn white) and border/environment(should turn black)
    // This shall be achieved by replacing pixels that have a typical environment color (e.g. green ones for grass) with black ones.
    // To improve the result (remove noise) morph operations can be applied.

    cv::GaussianBlur(rgb_image, rgb_image, cv::Size(config_.gauss_kernel_size, config_.gauss_kernel_size), 0.0, 0.0);

    // Only considering gray values for the rest of the pipeline
    cv::Mat gray;
    cv::cvtColor(rgb_image, gray, CV_RGB2GRAY);

    cv::Mat thresh(gray.size(), CV_8UC1);

    // Set plants, grass and other parts of the environment to black
    // Do so by checking the color of every pixel in the image
    for (auto i = 0u; i < rgb_image.rows; i++)
    {
        for (auto j = 0u; j < rgb_image.cols; j++)
        {
            auto &pixel = rgb_image.at<cv::Vec3b>(i, j);

            if ((pixel[1] >= pixel[0] || config_.dry) && pixel[1] >= pixel[2] + config_.difference_gb || gray.at<uchar>(i, j) <= config_.gray_thresh)
            {
                thresh.at<uchar>(i, j) = 0;
            } else 
            {
                thresh.at<uchar>(i, j) = 255;
            }
        }
    }

    /********** Object Extraction **********/

    // Perform morphological transformations on image to remove noise
    // Use closing (dilation followed by erosion) first to remove small holes on the trail which can be caused by stones on the trail
    // Use opening (erosion followed by dilation) after to remove noise in the background
    cv::Mat morph;

    if (config_.use_morph)
    {
        cv::Mat morph_kernel_closing = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                                 cv::Size(config_.morph_kernel_size_closing,
                                                                          config_.morph_kernel_size_closing));
        cv::Mat morph_kernel_opening = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                                 cv::Size(config_.morph_kernel_size_opening,
                                                                          config_.morph_kernel_size_opening));
        cv::dilate(thresh, morph, morph_kernel_closing);
        cv::erode(morph, morph, morph_kernel_closing);
        cv::erode(morph, morph, morph_kernel_opening);
        cv::dilate(morph, morph, morph_kernel_opening);
    }
    else
    {
        morph = thresh;
    }

    /****************** Average x-Coordinate ******************/
    // Calculates the average of all Pixels that belong to the Path
    // The target is simply that average in a certain region

    cv::Mat direction = rgb_image.clone();

    uint64_t sumpos = 0;
    uint64_t cnt = 0;
    double sum_x_weighted = 0.0;
    double sum_y_weighted = 0.0;
    int num_lines = config_.num_points;
    int line_counter_weighted = 0;
    double weight_sum = 0.0;
    int point_diff = ceil((double) h / (double) (config_.num_points + 1.0));

    for (int y = 0; y < morph.rows; y++)
    {
        if (y > 0 && y % point_diff == 0)
        {
            if (cnt)
            {
                #if DEBUG == 1
                    circle(direction, cv::Point(sumpos / cnt, y), (int) 12 * config_.scale / 100, cv::Scalar(255, 255, 255),
                           -1);
                    circle(direction, cv::Point(sumpos / cnt, y), (int) 8 * config_.scale / 100, cv::Scalar(0, 255, 0), -1);
                #endif

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
    #if DEBUG == 1
        cv::arrowedLine(direction, cv::Point(w / 2, h), cv::Point(sum_x_weighted / weight_sum, sum_y_weighted / weight_sum), cv::Scalar(0, 0, 255), (int) 4 * config_.scale / 100, cv::LineTypes::LINE_8, 0, 0.05);
    #endif

    // Calculate the angle which is published to the navigation node
    float dir_vec_x = sum_x_weighted / weight_sum - w / 2;
    float dir_vec_y = sum_y_weighted / weight_sum - h;

    float center_vec_x = 0;
    float center_vec_y = -1;
    int sign = dir_vec_x < 0 ? -1 : 1;

    float length_dir = sqrt(pow(dir_vec_x, 2) + pow(dir_vec_y, 2));
    float length_center = sqrt(pow(center_vec_x, 2) + pow(center_vec_y, 2));

    float vec_dot = center_vec_x * dir_vec_x + center_vec_y * dir_vec_y;
    float angle = sign * acos(vec_dot / (length_dir * length_center)) * 180 / M_PI;

    std_msgs::Float32 angle_msg;
    angle_msg.data = angle;
    target_pub_.publish(angle_msg);

    /********** Pipeline Visualization **********/
    #if DEBUG == 1
        // Center guide (vertical line in the middle of the image)
        cv::line(direction, cv::Point(w / 2, 0), cv::Point(w / 2, h), cv::Scalar(128, 128, 128), (int) 2 * config_.scale / 100);

        // Morph visualization
        cv::cvtColor(morph, morph, CV_GRAY2RGB);
        cv_ptr->image = morph;
        morph_pub_.publish(cv_ptr->toImageMsg());

        // Direction visualization
        cv_ptr->image = direction;
        direction_pub_.publish( cv_ptr->toImageMsg()); // this has to be published first for some reason, otherwise there won't be color

        // Threshold visualization
        cv::cvtColor(thresh, cv_ptr->image, CV_GRAY2RGB);
        threshold_pub_.publish(cv_ptr->toImageMsg());

        // Morph on input image
        cv::Mat morph_overlay;
        cv::addWeighted(rgb_image, 0.7, morph, 0.3, 0.0, morph_overlay);
        cv_ptr->image = morph_overlay;
        overlay_pub_.publish(cv_ptr->toImageMsg());
    #endif

    #if TIME_MEASUREMENT == 1
        end_ = ros::WallTime::now();
        double duration = (end_ - start_).toNSec() * 1e-6;
        time_sum_ += duration;
        time_counter_++;
        time_maximum_ = std::max(time_maximum_, duration);
        ROS_INFO_STREAM("Mean Exectution time (ms): " << time_sum_ / time_counter_ << " max: " << time_maximum_);
    #endif

    mutex_.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_pipeline");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<reconfros_software::PIPELINEConfig> server;
    dynamic_reconfigure::Server<reconfros_software::PIPELINEConfig>::CallbackType f;
    f = boost::bind(&dyn_cfg_callback, _1, _2);
    server.setCallback(f);

    image_transport::ImageTransport it(n);

    // Subscribes on the camera topic
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    #if DEBUG == 1
        threshold_pub_ = it.advertise("/threshold", 1);
        direction_pub_ = it.advertise("/direction", 1);
        morph_pub_ = it.advertise("/morph", 1);
        overlay_pub_ = it.advertise("/morphOverlay", 1);

        ROS_INFO("Using Debug Mode.");
    #endif

    // Publishes navigation commands to the robot system
    target_pub_ = n.advertise<std_msgs::Float32>("/target_angle", 1);

    ros::spin();
}