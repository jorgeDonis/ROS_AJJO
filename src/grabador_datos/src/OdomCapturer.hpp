#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

struct OdomInfo
{
    float x, y, yaw, x_speed, angular_speed;

    std::string to_string() const;
};

class OdomCapturer
{
    private:
        static const std::string ODOM_TOPIC;

        OdomInfo odom_info;

        ros::NodeHandle nh;
        ros::Subscriber sub;

        void odom_callback(nav_msgs::Odometry::ConstPtr const& msg);
    public:
        OdomCapturer(ros::NodeHandle const& n);
        OdomInfo const { return odom_info; }
};