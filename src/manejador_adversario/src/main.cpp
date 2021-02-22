#include <ros/ros.h>

#include "MiTipo.hpp"

int main(int argc, char** argv)
{
    MiTipo tipo;
    ros::init(argc, argv, "manejador_adversario");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    while (ros::ok())
    {
        sleep(1);
        tipo.decir_hola();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}