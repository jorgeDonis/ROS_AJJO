#include "RGBImageCapturer.hpp"

const std::string RGBImageCapturer::IMG_TOPIC = "/robot1/camera/rgb/image_raw";

RGBImageCapturer::RGBImageCapturer(ros::NodeHandle const &n) : nh(n), image_transport(nh)
{
    sub = image_transport.subscribe(IMG_TOPIC, 1, &RGBImageCapturer::image_callback, this);
}

void RGBImageCapturer::image_callback(sensor_msgs::ImageConstPtr const &msg)
{
    this->img = cv_bridge::toCvShare(msg, "mono8")->image;
}