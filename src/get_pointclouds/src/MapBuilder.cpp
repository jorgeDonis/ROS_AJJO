#include "MapBuilder.hpp"
#include "Plotter.hpp"

#include <sstream>
#include <chrono>
#include <rosbag/view.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/point_types_conversion.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

using namespace std;

MapBuilder::MapBuilder(std::string const &cloud_bag_filename,
           double vg_leaf, double ffph_r, double sift_min_scale, double sift_octaves, double sift_scales_per_octave,
           double sift_min_contrast, double inliner_th, double random_sample_keypoints, double RANSAC_iters)
{
    this->vg_leaf = vg_leaf;
    this->ffph_r = ffph_r;
    this->sift_min_scale = sift_min_scale;
    this->sift_octaves = sift_octaves;
    this->sift_scales_per_octave = sift_scales_per_octave;
    this->sift_min_contrast = sift_min_contrast;
    this->inliner_th = inliner_th;
    this->random_sample_keypoints = random_sample_keypoints;
    this->RANSAC_iters = RANSAC_iters;

    bag.open(cloud_bag_filename);
    M = PointCloud::Ptr(new PointCloud);
    boost::thread(Plotter::simple_vis);
    Plotter::simple_vis_cloud = M;
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

void purge_features(DescriptorsCloud::Ptr& features, PointCloud::Ptr& keypoints)
{
    DescriptorsCloud::Ptr purgedFeatures(new DescriptorsCloud);
    PointCloud::Ptr purgedKeypoints(new PointCloud);
    bool notnan;
    // Loop histogram of each feature
    for (int i = 0; i < features->size(); i++)
    {
        notnan = true;
        // Check for nan values
        for (int j = 0; j < 33; j++)
        {
            if (std::isnan(features->points[i].histogram[j]))
            {
                notnan = false;
                break;
            }
        }
        // If notnan, add to purged
        if (notnan)
        {
            purgedFeatures->push_back(features->points[i]);
            purgedKeypoints->push_back(keypoints->points[i]);
        }
    }
    features = purgedFeatures;
    keypoints = purgedKeypoints;
}

DescriptorsCloud::Ptr MapBuilder::get_descriptors(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    DescriptorsCloud::Ptr descriptors(new DescriptorsCloud());
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(key_points);
    fpfh.setSearchSurface(full_cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	fpfh.setRadiusSearch(ffph_r);
    fpfh.setNumberOfThreads(4);

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

PointCloud::Ptr MapBuilder::get_keypoints(PointCloud::Ptr cloud)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    PointCloud::Ptr keypoints(new PointCloud);

    const double model_resolution = computeCloudResolution(cloud);

    const double salient_radius = 6 * model_resolution;
    const double non_max_radius = 4 * model_resolution;
    const double normal_radius = 4 * model_resolution;
    const double border_radius = 1 * model_resolution;
    const double gamma_21 = 0.975;
    const double gamma_32 = 0.975;
    const double min_neighbors = 5;
    const int threads = 4;
    // Prepare detector
    kdtree->setInputCloud(cloud);

    iss.setSearchMethod(kdtree);
    iss.setSalientRadius(salient_radius);
    iss.setNonMaxRadius(non_max_radius);
    iss.setNormalRadius(normal_radius);
    iss.setBorderRadius(border_radius);
    iss.setThreshold21(gamma_21);
    iss.setThreshold32(gamma_32);
    iss.setMinNeighbors(min_neighbors);
    iss.setNumberOfThreads(threads);
    iss.setInputCloud(cloud);

    // Compute Keypoints
    iss.compute(*keypoints);

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado keypoints en %.4f segundos\n", seconds_spent);

    return keypoints;
}

pcl::PointCloud<pcl::Normal>::Ptr MapBuilder::estimate_normals(PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.04);
    ne.setNumberOfThreads(4);
    ne.compute(*normals);
    return normals;
}

void print(Eigen::Matrix4f const& m)
{
    using namespace Eigen;
    IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    cout << m.format(CleanFmt) << endl;
}

float calculate_distance(Eigen::Matrix4f const& transform, PointCloud::Ptr t_1_keypoints, PointCloud::Ptr t_0_keypoints)
{
    float avg_distance = 0;
    PointCloud::Ptr moved_t_1(new PointCloud);
    pcl::transformPointCloud(*t_1_keypoints, *moved_t_1, transform);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(moved_t_1);
    for (const auto point : t_0_keypoints->points)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            avg_distance += pointNKNSquaredDistance[0];
    }
    avg_distance /= t_0_keypoints->size();
}

Eigen::Matrix4f
MapBuilder::align_points(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
DescriptorsCloud::Ptr t_0_descriptors, DescriptorsCloud::Ptr t_1_descriptors, float& avg_distance)
{
    const auto t_0 = std::chrono::high_resolution_clock::now();

    pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT> ce;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crsc;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> te_svd;
    pcl::CorrespondencesPtr initial_correspondences(new pcl::Correspondences);
    pcl::CorrespondencesPtr filtered_correspondences(new pcl::Correspondences);

    ce.setInputSource(t_1_descriptors);
    ce.setInputTarget(t_0_descriptors);
    ce.determineReciprocalCorrespondences(*initial_correspondences);

    cout << "Correspondencias antes del filtrado: " << initial_correspondences->size() << endl;

    crsc.setInputSource(t_1_keypoints);
    crsc.setInputTarget(t_0_keypoints);
    crsc.setInlierThreshold(inliner_th);
    crsc.setMaximumIterations(RANSAC_iters);
    crsc.setRefineModel(true);
    crsc.setInputCorrespondences(initial_correspondences);
    crsc.getCorrespondences(*filtered_correspondences);

    cout << "Correspondencias tras el filtrado: " << filtered_correspondences->size() << endl;

    Eigen::Matrix4f transform;
    te_svd.estimateRigidTransformation(*t_1_keypoints, *t_0_keypoints, *filtered_correspondences, transform);

    avg_distance = calculate_distance(transform, t_1_keypoints, t_0_keypoints);

    const auto t_1 = std::chrono::high_resolution_clock::now();
    const double seconds_spent = std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
    printf("Calculado alineación en %.4f segundos\n", seconds_spent);
    return transform;
}

void MapBuilder::process_cloud(PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered = downsample(cloud, vg_leaf); //menor tamaño de hoja, más puntos
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimate_normals(cloud_filtered);
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    DescriptorsCloud::Ptr descriptors = get_descriptors(keypoints, cloud_filtered, normals);
    purge_features(descriptors, keypoints); //quita los keypoints que tienen descriptores con NaN (también quita los descriptores)

    cout << "Keypoints: " << keypoints->size() << endl;

    if (previous_pc_keypoints != nullptr)
    {   
        float avg_distance;
        const auto transform = align_points(previous_pc_keypoints, keypoints, previous_pc_features, descriptors, avg_distance);
        accumulated_distance += avg_distance;
        T *= transform;
        PointCloud::Ptr aligned_t_1_pc_global_frame(new PointCloud);
        pcl::transformPointCloud(*cloud_filtered, *aligned_t_1_pc_global_frame, T);

        *M += *aligned_t_1_pc_global_frame;
        Plotter::simple_vis_cloud = M;
        M = downsample(M, 0.02);
    }
    previous_pc = cloud_filtered;
	previous_pc_features = descriptors;
    previous_pc_keypoints = keypoints;
}

void MapBuilder::build_map()
{
    for (rosbag::MessageInstance const& m : rosbag::View(bag))
    {
        PointCloud::Ptr cloud = m.instantiate<PointCloud>();
        process_cloud(cloud);
    }
    pcl::io::savePCDFileASCII(get_filename(), *Plotter::simple_vis_cloud);
}

std::string MapBuilder::get_filename() const
{
    stringstream ss;
    ss << "vg_" << vg_leaf << "|"
       << "ffph_r_" << ffph_r << "|"
       << "sift_contrast_" << sift_min_contrast << "|"
       << "sift_scale_" << sift_min_scale << "|"
       << "sift_scales_per_oc_" << sift_scales_per_octave << "|"
       << "sift_n_oc_" << sift_octaves << "|"
       << "ransac_iters" << RANSAC_iters << "|"
       << "random_sample_kp" << random_sample_keypoints << "|"
       << "inliner_th_" << inliner_th << ".pcd";
    return ss.str();
}
