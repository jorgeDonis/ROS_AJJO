#pragma once

#include <string>
#include <rosbag/bag.h>
#include <cinttypes>
#include <pcl_ros/point_cloud.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

class MapBuilder
{
    private:
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr previous_pc_features;
        PointCloud::Ptr previous_pc_keypoints;
        PointCloud::Ptr previous_pc;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        rosbag::Bag bag;
        
        void process_cloud(PointCloud::Ptr cloud);
    public:
        MapBuilder(std::string const& cloud_bag_filename);
        void build_map();
        ~MapBuilder() { bag.close(); }
};