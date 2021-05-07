#include "MapBuilder.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "sub_pcl");
  	ros::NodeHandle nh;
	MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag");
	const auto t_0 = std::chrono::high_resolution_clock::now();
	map_builder.build_map();
	const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Terminado reconstrucción 3D en %.4f segundos!\n", seconds_spent);
	while (ros::ok())
		ros::spinOnce();
}
