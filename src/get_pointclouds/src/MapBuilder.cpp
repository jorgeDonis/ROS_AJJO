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
#include <pcl/registration/correspondence_rejection_distance.h>
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

using PointT = pcl::PointXYZRGB;
using DescriptorsCloud = pcl::PointCloud<pcl::FPFHSignature33>;

MapBuilder::MapBuilder(std::string const &cloud_bag_filename)
{
    bag.open("point_cloud_messages.bag");
    boost::thread(Plotter::simple_vis);
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
    pcl::search::KdTree<PointT> tree;
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
    const int n_octaves = 8;
    const int n_scales_per_octave = 8;
    const float min_contrast = 0.5f;

    // Estimate the sift interest points using Intensity values from RGB values
    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
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

DescriptorsCloud::Ptr get_descriptors(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    // Object for storing the PFH descriptors for each point.
    DescriptorsCloud::Ptr descriptors(new DescriptorsCloud());
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

	// FPFH estimation object.
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(key_points);
    fpfh.setSearchSurface(full_cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.21);

	fpfh.compute(*descriptors);

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado descriptores en %.4f segundos\n", seconds_spent);

    return descriptors;
}

PointCloud::Ptr downsample(PointCloud::ConstPtr cloud, float leaf_size)
{
	PointCloud::Ptr cloud_filtered(new PointCloud);
	pcl::VoxelGrid<PointT> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	v_grid.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr get_normals_integral(PointCloud::ConstPtr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
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
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
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

double distance(PointCloud::Ptr cloud_a, PointCloud::Ptr cloud_b)
{
    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud(cloud_b);
    float max_dist_a = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud_a->points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_b.nearestKSearch(cloud_a->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
            max_dist_a = sqr_distances[0];
    }

    // compare B to A
    pcl::search::KdTree<PointT> tree_a;
    tree_a.setInputCloud(cloud_a);
    float max_dist_b = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud_b->points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_a.nearestKSearch(cloud_b->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
            max_dist_b = sqr_distances[0];
    }

    max_dist_a = std::sqrt(max_dist_a);
    max_dist_b = std::sqrt(max_dist_b);

    const float dist = std::max(max_dist_a, max_dist_b);
    return dist;
}

Eigen::Matrix4f
align_points(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
DescriptorsCloud::Ptr t_0_descriptors, DescriptorsCloud::Ptr t_1_descriptors)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> scia;
    scia.setInputTarget(t_0_keypoints);
    scia.setInputSource(t_1_keypoints);
    scia.setTargetFeatures(t_0_descriptors);
    scia.setSourceFeatures(t_1_descriptors);
    scia.setMinSampleDistance(0.04f);
    scia.setMaxCorrespondenceDistance(0.01f);
    scia.setMaximumIterations(1500);
    pcl::registration::CorrespondenceRejectorDistance::Ptr rej(new pcl::registration::CorrespondenceRejectorDistance);
    rej->setMaximumDistance(0.05f);
    rej->setInputSource<PointT>(t_1_keypoints);
    rej->setInputTarget<PointT>(t_0_keypoints);
    scia.addCorrespondenceRejector(rej);
    
    PointCloud r;
    scia.align(r);
    auto transform = scia.getFinalTransformation();

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado alineación en %.4f segundos\n", seconds_spent);
    return transform;
}

void MapBuilder::process_cloud(PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered = downsample(cloud, 0.025);
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimate_normals(cloud_filtered);
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    DescriptorsCloud::Ptr descriptors = get_descriptors(keypoints, cloud_filtered, normals);

    if (previous_pc_keypoints != nullptr)
    {
        cout << "Keypoints[i-1]: " << previous_pc_keypoints->size() << endl;
        cout << "Características[i-1]: " << previous_pc_features->size() << endl;
        cout << "Keypoints[i]: " << keypoints->size() << endl;
        cout << "Características[i]: " << descriptors->size() << endl;

        const auto transform = align_points(previous_pc_keypoints, keypoints, previous_pc_features, descriptors);

        //measure transformation accuracy
        PointCloud::Ptr transformed_cloud_t0(new PointCloud);
        pcl::transformPointCloud(*cloud_filtered, *transformed_cloud_t0, transform);
        printf("Distancia Hausdorff después de transformación: %.4f\n", distance(transformed_cloud_t0, previous_pc));

        print(transform);
        T *= transform;
        PointCloud::Ptr transformed_cloud(new PointCloud);
        pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T);
        *transformed_cloud += *previous_pc;
        PointCloud::Ptr old_pc = previous_pc;
        previous_pc = transformed_cloud;
        PointCloud::Ptr t_1_cloud(new PointCloud);
        pcl::copyPointCloud(*cloud_filtered, *t_1_cloud);
        *cloud_filtered += *old_pc;

        // Plotter::new_cloud = t_1_cloud;
        // Plotter::unmerged_clouds = cloud_filtered;
        // Plotter::merged_clouds = transformed_cloud;
        // Plotter::plot_transformation();
        Plotter::simple_vis_cloud = transformed_cloud;
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
        if (i <= 0)
            continue;
        process_cloud(cloud);
        sleep(1000 / FPS);
    }
}
