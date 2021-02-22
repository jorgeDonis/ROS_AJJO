#include "Opponent.hpp"

Opponent::Opponent()
{
    laser_sub = node_handle.subscribe("base_scan", 1, &Opponent::laser_handler, this);
}

//Makes /robot2 move 'pa un lao y pa otro' ok?
void Opponent::run()
{
    ros::Rate rate(Opponent::RATE);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void Opponent::laser_handler(sensor_msgs::LaserScan::ConstPtr const& msg)
{
    ROS_INFO_STREAM("AngleMin: " << rad_to_deg(msg->angle_min));             
    ROS_INFO_STREAM("AngleMax: " << rad_to_deg(msg->angle_max));             
    ROS_INFO_STREAM("AngleIncrement: " << rad_to_deg(msg->angle_increment)); 
    ROS_INFO_STREAM("RangeMin: " << msg->range_min);             
    ROS_INFO_STREAM("RangeMax: " << msg->range_max);
}