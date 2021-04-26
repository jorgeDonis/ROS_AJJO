#include "MapBuilder.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

static PointCloud::Ptr visu_pc(new PointCloud);

void simpleVis()
{
  	pcl::visualization::CloudViewer viewer ("Reconstrucción 3D");
	while (!viewer.wasStopped())
	{
	  viewer.showCloud(visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "sub_pcl");
  	ros::NodeHandle nh;
	boost::thread t(simpleVis);
	MapBuilder map_builder("point_cloud_messages.bag", visu_pc);
	map_builder.build_map();
	std::cout << "Terminado reconstrucción 3D!" << std::endl;
	while (ros::ok())
		ros::spinOnce();
}
