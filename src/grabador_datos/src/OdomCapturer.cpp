#include "OdomCapturer.hpp"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sstream>

const std::string OdomCapturer::ODOM_TOPIC = "/robot1/odom";

OdomCapturer::OdomCapturer(ros::NodeHandle const& n) : nh(n)
{
    sub = nh.subscribe(ODOM_TOPIC, 100, &OdomCapturer::odom_callback, this);
}

void OdomCapturer::odom_callback(nav_msgs::Odometry::ConstPtr const& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    odom_info.x = msg->pose.pose.position.x;
    odom_info.y = msg->pose.pose.position.y;
    odom_info.yaw = yaw;
}

std::string OdomInfo::to_string() const
{
    std::stringstream ss;

    ss << x << "_" << y << "_" << yaw;

    return ss.str();
}
