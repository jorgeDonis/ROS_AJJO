#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>

//This turtlebot moves /robot2 'pa un lao y pa otro' okay?
class Opponent
{
    private:
        ros::Subscriber laser_sub;

        static double constexpr rad_to_deg(double rad) { return (double) rad * 180 / M_PI; }

        static constexpr uint16_t RATE = 1;
        ros::NodeHandle node_handle;
        void laser_handler(sensor_msgs::LaserScan::ConstPtr const& msg);
    public:
        Opponent();
        void run();
};