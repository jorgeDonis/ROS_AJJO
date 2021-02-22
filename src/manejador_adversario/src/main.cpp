#include <ros/ros.h>

#include "Opponent.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manejador_adversario");
    Opponent opponent;
    opponent.run();
    return 0;
}