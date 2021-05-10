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
			0.03,		//Voxel grid leaf size
			0.24,		//FFPH search radius
			0.001,		//SIFT3D minimmum scale
			8,			//SIFT3D nº octaves
			11,			//SIFT3D number of scales per octave
			0.001,		//SIFT3D minimmum contrast
			0.09,		//inlier threshold
			2000,		//random_sample nº keypoints
			50000,		//RANSAC iterations
			100,		//ICP max iterations
			1e-7,		//ICP maximmum epsilon for convergence
			0.3			//ICP maximmum correspondence distance
		);
	
		map_builder1.build_map();
		cout << "Terminado reconstrucción con error total: " << map_builder1.accumulated_distance << endl;

		while (ros::ok())
			ros::spinOnce();
	}
	catch (std::exception const& e)
	{
		cout << e.what() << endl;
	}
}
