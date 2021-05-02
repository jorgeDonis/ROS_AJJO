#include "MapBuilder.hpp"
#include "Plotter.hpp"

#include <chrono>
#include <rosbag/view.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/shot.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/pfh.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>

using namespace std;

MapBuilder::MapBuilder(std::string const &cloud_bag_filename)
{
    bag.open("point_cloud_messages.bag");
}

// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double
computeCloudResolution(const PointCloud::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}

PointCloud::Ptr get_keypoints(PointCloud::Ptr cloud)
{

    const auto t_0 = std::chrono::high_resolution_clock::now();
    // Parameters for sift computation
    const float min_scale = 0.008f;
    const int n_octaves = 7;
    const int n_scales_per_octave = 8;
    const float min_contrast = 0.02f;

    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);
    PointCloud::Ptr cloud_temp(new PointCloud);
    pcl::copyPointCloud(result, *cloud_temp);

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado keypoints en %.4f segundos\n", seconds_spent);

    return cloud_temp;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr get_descriptors(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	// FPFH estimation object.
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(key_points);
    fpfh.setSearchSurface(full_cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.18);

	fpfh.compute(*descriptors);

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado descriptores en %.4f segundos\n", seconds_spent);

    return descriptors;
}

PointCloud::Ptr downsample(PointCloud::ConstPtr cloud, float leaf_size)
{
	PointCloud::Ptr cloud_filtered(new PointCloud);
	pcl::VoxelGrid<pcl::PointXYZRGB> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	v_grid.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr get_normals_integral(PointCloud::ConstPtr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.06);
    ne.compute(*normals);
    return normals;
}

void print(Eigen::Matrix4f const& m)
{
    using namespace Eigen;
    using namespace std;
    IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    cout << m.format(CleanFmt) << endl;
}

Eigen::Matrix4f align_points()

void MapBuilder::process_cloud(PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered = downsample(cloud, 0.02);
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimate_normals(cloud_filtered);
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors = get_descriptors(keypoints, cloud_filtered, normals);

    if (previous_pc_keypoints != nullptr)
    {
        cout << "Keypoints[i-1]: " << previous_pc_keypoints->size() << endl;
        cout << "Características[i-1]: " << previous_pc_features->size() << endl;
        cout << "Keypoints[i]: " << keypoints->size() << endl;
        cout << "Características[i]: " << descriptors->size() << endl;

        pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> scia;
        scia.setInputTarget(previous_pc_keypoints);
        scia.setInputSource(keypoints);
        scia.setTargetFeatures(previous_pc_features);
        scia.setSourceFeatures(descriptors);
        scia.setMinSampleDistance(0.025f);
        scia.setMaxCorrespondenceDistance(0.005f);
        scia.setMaximumIterations(1000);
        PointCloud r;
        scia.align(r);
        PointCloud::Ptr transformed_cloud(new PointCloud);
        auto transform = scia.getFinalTransformation();
        print(transform);
        T *= transform;
        pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T);
        *transformed_cloud += *previous_pc;
        PointCloud::Ptr old_pc = previous_pc;
        previous_pc = transformed_cloud;

        *cloud_filtered += *old_pc;

        Plotter::plot_transformation(cloud_filtered, old_pc, transformed_cloud);
    }
    else
        previous_pc = cloud_filtered;
	previous_pc_features = descriptors;
    previous_pc_keypoints = keypoints;
}

void MapBuilder::build_map()
{
    ros::Rate rate(FPS);
    static uint16_t i = 0;
    for (rosbag::MessageInstance const& m : rosbag::View(bag))
    {
        ++i;
        PointCloud::Ptr cloud = m.instantiate<PointCloud>();
        if (i <= 20)
            continue;
        process_cloud(cloud);
        sleep(1000 / FPS);
    }
}
