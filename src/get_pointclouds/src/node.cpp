#include "MapBuilder.hpp"

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
		ros::NodeHandle nh;
		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();
				MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.12,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		MapBuilder map_builder("/home/jorge/catkin_ws/point_cloud_messages.bag",
		0.01,		//Voxel grid leaf size
		0.21,		//FFPH search radius
		0.003,		//SIFT3D minimmum scale
		8,			//SIFT3D nº octaves
		10,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.02		//RANSAC inlier threshold
		);
		map_builder.build_map();

		while (ros::ok())
			ros::spinOnce();
	}
	catch (std::exception const& e)
	{
		cout << e.what() << endl;
	}
}
