#pragma once

#include <string>
#include <rosbag/bag.h>
#include <cinttypes>
#include <pcl_ros/point_cloud.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
using PointT = pcl::PointXYZRGB;
using DescriptorT = pcl::FPFHSignature33;
using DescriptorsCloud = pcl::PointCloud<DescriptorT>;

class MapBuilder
{
    private:
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr previous_pc_features;
        PointCloud::Ptr previous_pc_keypoints;
        PointCloud::Ptr previous_pc;

        PointCloud::Ptr M;
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        rosbag::Bag bag;

        float accumulated_distance = 0.0f;

        double vg_leaf;
        double ffph_r;
        double sift_min_scale;
        double sift_octaves;
        double sift_scales_per_octave;
        double sift_min_contrast;
        double inliner_th;
        double random_sample_keypoints;
        double RANSAC_iters;

        DescriptorsCloud::Ptr get_descriptors(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
        PointCloud::Ptr get_keypoints(PointCloud::Ptr cloud);
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(PointCloud::Ptr cloud);

        Eigen::Matrix4f
        align_points(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
        DescriptorsCloud::Ptr t_0_descriptors, DescriptorsCloud::Ptr t_1_descriptors, float& avg_distance);

        void process_cloud(PointCloud::Ptr cloud);
        std::string get_filename() const;
    public:
        MapBuilder(std::string const &cloud_bag_filename,
        double vg_leaf = 0.02, double ffph_r = 0.03, double sift_min_scale = 0.005, double sift_octaves = 9, double sift_scales_per_octave = 11,
        double sift_min_contrast = 0, double inliner_th = 0.02, double random_sample_keypoints = 1400, double RANSAC_iters = 10000);
        void build_map();
        ~MapBuilder() { bag.close(); }
};