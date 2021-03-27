#include "ImageCapturer.hpp"

const std::string ImageCapturer::IMG_TOPIC = "/robot1/camera/rgb/image_raw";

ImageCapturer::ImageCapturer(ros::NodeHandle const& n) : nh(n), image_transport(nh)
{
    sub = image_transport.subscribe(IMG_TOPIC, 1, &ImageCapturer::image_callback, this);
}

void ImageCapturer::image_callback(sensor_msgs::ImageConstPtr const& msg)
{
    this->img = cv_bridge::toCvShare(msg, "mono8")->image;
}