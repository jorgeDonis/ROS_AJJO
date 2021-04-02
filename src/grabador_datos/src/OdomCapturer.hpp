#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

class OdomCapturer
{
    private:
        static const std::string IMG_TOPIC;

        ros::NodeHandle nh;
        image_transport::Subscriber sub;
        image_transport::ImageTransport image_transport;
        cv::Mat img;

        void image_callback(sensor_msgs::ImageConstPtr const& msg);
    public:
        OdonCapturer(ros::NodeHandle const& n);
        cv::Mat get_odom() const { return img; }
};