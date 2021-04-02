#include "ImageCapturer.hpp"

const std::string ImageCapturer::IMG_TOPIC = "/robot1/odom";

ImageCapturer::ImageCapturer(ros::NodeHandle const& n) : nh(n), image_transport(nh)
{
    sub = image_transport.subscribe(IMG_TOPIC, 1, &ImageCapturer::image_callback, this);
}

void ImageCapturer::image_callback(sensor_msgs::ImageConstPtr const& msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(msg);

    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;

    if (depth_mono8_img.rows != depth_float_img.rows || depth_mono8_img.cols != depth_float_img.cols)
        depth_mono8_img = cv::Mat(depth_float_img.size(), CV_8UC1);
    
    cv::convertScaleAbs(depth_float_img, depth_mono8_img, 55, -45);
    this->img = depth_mono8_img;
}