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
        static void plot_transformation(PointCloud::Ptr cloud_1, PointCloud::Ptr cloud_2, PointCloud::Ptr cloud_merge);
};