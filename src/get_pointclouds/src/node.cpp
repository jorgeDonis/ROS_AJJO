#include "MapBuilder.hpp"
#include "Plotter.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <exception>

int main(int argc, char** argv)
{
	try
	{
		ros::init(argc, argv, "sub_pcl");

		MapBuilder map_builder1("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.02,		//Voxel grid leaf size
		0.05,		//FFPH search radius
		0.001,		//SIFT3D minimmum scale
		6,			//SIFT3D nº octaves
		8,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.1,		//inlier threshold
		2000,		//random_sample nº keypoints
		100000		//RANSAC iterations
		);
	
		map_builder1.build_map();
		cout << "Terminado reconstrucción con error total: " << map_builder1.accumulated_distance << endl;
	}
	catch (std::exception const& e)
	{
		cout << e.what() << endl;
	}
}
