#pragma once

#include "Clock.hpp"

#include <string>
#include <rosbag/bag.h>
#include <cinttypes>
#include <pcl_ros/point_cloud.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
using PointT = pcl::PointXYZRGB;
using NormalT = pcl::Normal;
using SHOT = pcl::SHOT352;
using SHOTCloud = pcl::PointCloud<SHOT>;
using FFPH = pcl::FPFHSignature33;
using FFPHCloud = pcl::PointCloud<FFPH>;
using PointNormalT = pcl::PointXYZRGBNormal;
using PointNormalCloud = pcl::PointCloud<PointNormalT>;

enum class KeypointDetector
{
    SIFT3D,
    RANDOM,
    ISS
};

enum class FeatureDetector
{
    FFPH,
    SHOT
};

class MapBuilder
{
    private:
        Clock clock;

        FFPHCloud::Ptr previous_pc_ffph_features;
        SHOTCloud::Ptr previous_pc_shot_features;
        PointCloud::Ptr previous_pc_keypoints;
        PointCloud::Ptr previous_pc;
        PointNormalCloud::Ptr previous_point_normal_c;

        PointCloud::Ptr M;
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        rosbag::Bag bag;

        double total_time_spent_reconstruction;
        const bool use_ICP;
        std::string filename;
        const KeypointDetector kp_detector;
        const FeatureDetector feature_detector;

        const double vg_leaf;
        const double feature_r;
        const double sift_min_scale;
        const double sift_octaves;
        const double sift_scales_per_octave;
        const double sift_min_contrast;
        const double inliner_th;
        const double random_sample_keypoints;
        const double RANSAC_iters;
        const double ICP_iters;
        const double ICP_e;
        const double ICP_correspondence_distance;

        PointCloud::Ptr get_keypoints(PointCloud::Ptr cloud);
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(PointCloud::Ptr cloud);

        SHOTCloud::Ptr get_descriptors_shot(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
        FFPHCloud::Ptr get_descriptors_ffph(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

        Eigen::Matrix4f
        align_points_ffph(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
        FFPHCloud::Ptr t_0_descriptors, FFPHCloud::Ptr t_1_descriptors);

        Eigen::Matrix4f
        align_points_shot(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
        SHOTCloud::Ptr t_0_descriptors, SHOTCloud::Ptr t_1_descriptors);

        Eigen::Matrix4f ICP(PointNormalCloud::Ptr cloud_t1, PointNormalCloud::Ptr cloud_t0, Eigen::Matrix4f const &transform_coarse);

        void process_cloud(PointCloud::Ptr& cloud);
        std::string get_filename();
    public:
        float accumulated_distance = 0.0f;
        
        MapBuilder(std::string const &cloud_bag_filename,
        double vg_leaf = 0.02, double feature_r = 0.03, double sift_min_scale = 0.005, double sift_octaves = 9, double sift_scales_per_octave = 11,
        double sift_min_contrast = 0, double inliner_th = 0.02, double random_sample_keypoints = 14000, double RANSAC_iters = 10000,
        double ICP_iters = 500, double ICP_e = 1e-9, double ICP_correspondence_distance = 0.05, std::string filename = "", bool use_ICP = true,
        KeypointDetector kp_detector = KeypointDetector::SIFT3D, FeatureDetector = FeatureDetector::FFPH);
        void build_map();
        ~MapBuilder() { bag.close(); }
};