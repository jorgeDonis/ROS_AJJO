#pragma once

#include <string>
#include <rosbag/bag.h>
#include <cinttypes>
#include <pcl_ros/point_cloud.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

class MapBuilder
{
    private:
        static const uint8_t FPS = 5;

        PointCloud::Ptr& pc_visu;
        rosbag::Bag bag;

        inline void sleep(uint16_t milliseconds) { boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds)); }

        void process_cloud(PointCloud::ConstPtr cloud);
    public:
        MapBuilder(std::string const& cloud_bag_filename, PointCloud::Ptr& point_cloud_to_visualize);
        void build_map();
        ~MapBuilder() { bag.close(); }
};