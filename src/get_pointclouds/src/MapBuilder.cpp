#include "MapBuilder.hpp"
#include "Plotter.hpp"

#include <sstream>
#include <chrono>
#include <rosbag/view.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>

using namespace std;

MapBuilder::MapBuilder(std::string const &cloud_bag_filename,
           double vg_leaf, double ffph_r, double sift_min_scale, double sift_octaves, double sift_scales_per_octave,
           double sift_min_contrast, double inliner_th)
{
    this->vg_leaf = vg_leaf;
    this->ffph_r = ffph_r;
    this->sift_min_scale = sift_min_scale;
    this->sift_octaves = sift_octaves;
    this->sift_scales_per_octave = sift_scales_per_octave;
    this->sift_min_contrast = sift_min_contrast;
    this->inliner_th = inliner_th;

    bag.open(cloud_bag_filename);
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

    // Object for storing the PFH descriptors for each point.
    DescriptorsCloud::Ptr descriptors(new DescriptorsCloud());
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

	// FPFH estimation object.
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
    // Parameters for sift computation
    const float min_scale = sift_min_scale;
    const int n_octaves = sift_octaves;
    const int n_scales_per_octave = sift_scales_per_octave;
    const float min_contrast = sift_min_contrast;

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
    crsc.setMaximumIterations(10000);
    crsc.setRefineModel(true);
    crsc.setInputCorrespondences(initial_correspondences);
    crsc.getCorrespondences(*filtered_correspondences);

    cout << "Correspondencias tras el filtrado: " << filtered_correspondences->size() << endl;

    Eigen::Matrix4f transform;
    te_svd.estimateRigidTransformation(*t_1_keypoints, *t_0_keypoints, *filtered_correspondences, transform);

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

    // Plotter::plot_correspondences(moved_t_1, t_0_keypoints, filtered_correspondences);

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
        cout << "distancia media entre nubes de puntos después de transformación: " << avg_distance << endl;
        T *= transform;
        PointCloud::Ptr transformed_cloud(new PointCloud);
        pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T);

        // PointCloud::Ptr new_cloud(new PointCloud);
        // PointCloud::Ptr unmerged_clouds(new PointCloud);
        // PointCloud::Ptr merged_clouds(new PointCloud);
        // pcl::copyPointCloud(*cloud_filtered, *new_cloud);
        // *merged_clouds = *transformed_cloud + *previous_pc;
        // *unmerged_clouds = *cloud_filtered + *previous_pc;
        // Plotter::new_cloud = new_cloud;
        // Plotter::unmerged_clouds = unmerged_clouds;
        // Plotter::merged_clouds = merged_clouds;
        // Plotter::plot_transformation();
        
        *transformed_cloud += *previous_pc;
        PointCloud::Ptr old_pc = previous_pc;
        previous_pc = transformed_cloud;

        if (transformed_cloud->size() > 30000)
            transformed_cloud = downsample(transformed_cloud, 0.03);
        Plotter::simple_vis_cloud = transformed_cloud;
    }
    else
        previous_pc = cloud_filtered;
	previous_pc_features = descriptors;
    previous_pc_keypoints = keypoints;
}

void MapBuilder::build_map()
{
    static uint16_t i = 0;
    for (rosbag::MessageInstance const& m : rosbag::View(bag))
    {
        ++i;
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
       << "inliner_th_" << inliner_th << ".pcd";
    return ss.str();
}
