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
			0.02,						//Voxel grid leaf size
			0.22,						//Feature search radius
			0.001,						//SIFT3D minimmum scale
			8,							//SIFT3D nº octaves
			11,							//SIFT3D number of scales per octave
			0.001,						//SIFT3D minimmum contrast
			3,							//inlier threshold
			2000,						//random_sample nº keypoints
			60000,						//RANSAC iterations
			300,						//ICP max iterations
			1e-11,						//ICP maximmum epsilon for convergence
			0.2,						//ICP maximmum correspondence distance
			"SIFT_SHOT_ICP",		//Output reconstruction cloud filename
			true,						//Use ICP				
			KeypointDetector::SIFT3D,	//Keypoint detection algorithm
			FeatureDetector::SHOT		//Feature detection algorithm
		);
		map_builder1.build_map();
		map_builder1.~MapBuilder();

		MapBuilder map_builder2("/home/jorge/catkin_ws/point_cloud_messages.bag",
			0.02,						//Voxel grid leaf size
			0.22,						//Feature search radius
			0.001,						//SIFT3D minimmum scale
			8,							//SIFT3D nº octaves
			11,							//SIFT3D number of scales per octave
			0.001,						//SIFT3D minimmum contrast
			3,							//inlier threshold
			2000,						//random_sample nº keypoints
			60000,						//RANSAC iterations
			300,						//ICP max iterations
			1e-11,						//ICP maximmum epsilon for convergence
			0.2,						//ICP maximmum correspondence distance
			"ISS_FFPH_NO_ICP",		//Output reconstruction cloud filename
			false,						//Use ICP				
			KeypointDetector::ISS,	//Keypoint detection algorithm
			FeatureDetector::FFPH		//Feature detection algorithm
		);
		map_builder2.build_map();
		map_builder2.~MapBuilder();

		while (ros::ok())
			ros::spinOnce();
	}
	catch (std::exception const& e)
	{
		cout << e.what() << endl;
	}
}
