#include "MapBuilder.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "sub_pcl");
  	ros::NodeHandle nh;
	MapBuilder map_builder("point_cloud_messages.bag");
	map_builder.build_map();
	std::cout << "Terminado reconstrucción 3D!" << std::endl;
	while (ros::ok())
		ros::spinOnce();
}
