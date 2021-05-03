#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <functional>
#include <pcl/correspondence.h>
#include <vector>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

class Plotter
{
    public:
        static void plot_correspondences(PointCloud::Ptr cloud_1, PointCloud::Ptr cloud_2, pcl::CorrespondencesConstPtr correspondences);
        static void plot_normals(PointCloud::ConstPtr surface, pcl::PointCloud<pcl::Normal>::Ptr normals);
        static void plot_transformation();

        static void simple_vis();

        static PointCloud::Ptr unmerged_clouds;
        static PointCloud::Ptr new_cloud;
        static PointCloud::Ptr merged_clouds;
        static PointCloud::Ptr simple_vis_cloud;
};