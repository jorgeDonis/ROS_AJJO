#include <ros/ros.h>

#include "RobotController.hpp"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "manejador_gazebo");
    ros::NodeHandle nh;
    RobotController robot(nh);
    robot.main_loop();
    return 0;
}