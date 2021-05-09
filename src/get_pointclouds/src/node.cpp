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
		0.015,		//Voxel grid leaf size
		0.22,		//FFPH search radius
		0.001,		//SIFT3D minimmum scale
		9,			//SIFT3D nº octaves
		11,			//SIFT3D number of scales per octave
		0.001,		//SIFT3D minimmum contrast
		0.001		//RANSAC inlier threshold
		);
		map_builder1.build_map();

		// 		MapBuilder map_builder2("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 0.01,		//Voxel grid leaf size
		// 0.12,		//FFPH search radius
		// 0.003,		//SIFT3D minimmum scale
		// 8,			//SIFT3D nº octaves
		// 10,			//SIFT3D number of scales per octave
		// 0.001,		//SIFT3D minimmum contrast
		// 0.08		//RANSAC inlier threshold
		// );
		// map_builder2.build_map();

		// MapBuilder map_builder3("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 0.01,		//Voxel grid leaf size
		// 0.08,		//FFPH search radius
		// 0.003,		//SIFT3D minimmum scale
		// 8,			//SIFT3D nº octaves
		// 10,			//SIFT3D number of scales per octave
		// 0.001,		//SIFT3D minimmum contrast
		// 0.02		//RANSAC inlier threshold
		// );
		// map_builder3.build_map();

		// MapBuilder map_builder4("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 0.03,		//Voxel grid leaf size
		// 0.21,		//FFPH search radius
		// 0.0003,		//SIFT3D minimmum scale
		// 12,			//SIFT3D nº octaves
		// 11,			//SIFT3D number of scales per octave
		// 0.0001,		//SIFT3D minimmum contrast
		// 0.02		//RANSAC inlier threshold
		// );
		// map_builder4.build_map();

		// MapBuilder map_builder5("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.02,   //Voxel grid leaf size
		// 					   0.21,   //FFPH search radius
		// 					   0.0003, //SIFT3D minimmum scale
		// 					   12,	   //SIFT3D nº octaves
		// 					   11,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.02	   //RANSAC inlier threshold
		// );
		// map_builder5.build_map();

		// MapBuilder map_builder6("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.02,   //Voxel grid leaf size
		// 					   0.21,   //FFPH search radius
		// 					   0.0003, //SIFT3D minimmum scale
		// 					   12,	   //SIFT3D nº octaves
		// 					   11,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.15	   //RANSAC inlier threshold
		// );
		// map_builder6.build_map();

		// MapBuilder map_builder7("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.02,   //Voxel grid leaf size
		// 					   0.21,   //FFPH search radius
		// 					   0.0003, //SIFT3D minimmum scale
		// 					   12,	   //SIFT3D nº octaves
		// 					   11,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.1	   //RANSAC inlier threshold
		// );
		// map_builder7.build_map();

		// MapBuilder map_builder8("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.02,   //Voxel grid leaf size
		// 					   0.21,   //FFPH search radius
		// 					   0.0003, //SIFT3D minimmum scale
		// 					   12,	   //SIFT3D nº octaves
		// 					   11,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.2	   //RANSAC inlier threshold
		// );
		// map_builder8.build_map();

		// MapBuilder map_builder9("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.03,   //Voxel grid leaf size
		// 					   0.21,   //FFPH search radius
		// 					   0.0003, //SIFT3D minimmum scale
		// 					   8,	   //SIFT3D nº octaves
		// 					   6,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.07	   //RANSAC inlier threshold
		// );
		// map_builder9.build_map();

		// MapBuilder map_builder10("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.025,   //Voxel grid leaf size
		// 					   0.25,   //FFPH search radius
		// 					   0.003, //SIFT3D minimmum scale
		// 					   8,	   //SIFT3D nº octaves
		// 					   6,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.07	   //RANSAC inlier threshold
		// );
		// map_builder10.build_map();

		// MapBuilder map_builder11("/home/jorge/catkin_ws/point_cloud_messages.bag",
		// 					   0.025,  //Voxel grid leaf size
		// 					   0.25,   //FFPH search radius
		// 					   0.003,  //SIFT3D minimmum scale
		// 					   8,	   //SIFT3D nº octaves
		// 					   6,	   //SIFT3D number of scales per octave
		// 					   0.0001, //SIFT3D minimmum contrast
		// 					   0.07	   //RANSAC inlier threshold
		// );
		// map_builder11.build_map();

		while (ros::ok())
			ros::spinOnce();
	}
	catch (std::exception const& e)
	{
		cout << e.what() << endl;
	}
}
