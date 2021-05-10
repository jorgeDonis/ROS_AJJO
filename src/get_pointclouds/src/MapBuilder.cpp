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
#include <pcl/registration/icp.h>
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
           double sift_min_contrast, double inliner_th, double random_sample_keypoints, double RANSAC_iters,
           double ICP_iters, double ICP_correspondence_distance, double ICP_e)
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
    this->ICP_e = ICP_e;
    this->ICP_iters = ICP_iters;
    this->ICP_correspondence_distance = ICP_correspondence_distance;

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
    clock.tik();

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

    clock.tok();
    printf("Calculado descriptores en %.4f segundos\n", clock.seconds_spent());

    return descriptors;
}

PointCloud::Ptr downsample(PointCloud::ConstPtr cloud, double leaf_size)
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
    clock.tik();

    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    sift.setSearchMethod(tree);
    sift.setScales(sift_min_scale, sift_octaves, sift_scales_per_octave);
    sift.setMinimumContrast(sift_min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);
    PointCloud::Ptr cloud_temp(new PointCloud);
    pcl::copyPointCloud(result, *cloud_temp);

    clock.tok();
    printf("Calculado %lu keypoints en %.4f segundos\n", cloud_temp->size(), clock.seconds_spent());

    return cloud_temp;
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
DescriptorsCloud::Ptr t_0_descriptors, DescriptorsCloud::Ptr t_1_descriptors)
{
    clock.tik();

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

    clock.tok();
    printf("Calculado alineación (gruesa) en %.4f segundos\n", clock.seconds_spent());
    return transform;
}

Eigen::Matrix4f MapBuilder::ICP(PointCloud::Ptr cloud_t1, PointCloud::Ptr cloud_t0, Eigen::Matrix4f const& transform_coarse)
{
    clock.tik();

    Eigen::Matrix4f transform_fine = Eigen::Matrix4f::Identity();
    PointCloud::Ptr foo_cloud(new PointCloud);
    static pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(ICP_correspondence_distance);
    icp.setMaximumIterations(ICP_iters);
    icp.setTransformationEpsilon(ICP_e);
    icp.setInputSource(cloud_t1);
    icp.setInputTarget(cloud_t0);
    icp.align(*foo_cloud, transform_coarse);
    transform_fine = icp.getFinalTransformation();
    print(transform_fine);
    print(transform_coarse);

    clock.tok();
    printf("Calculado alineado (fino) en %.4f segundos\n", clock.seconds_spent());
}

void MapBuilder::process_cloud(PointCloud::Ptr& cloud)
{
    PointCloud::Ptr cloud_filtered = downsample(cloud, vg_leaf); //menor tamaño de hoja, más puntos
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimate_normals(cloud_filtered);
    DescriptorsCloud::Ptr descriptors = get_descriptors(keypoints, cloud_filtered, normals);
    purge_features(descriptors, keypoints); //quita los keypoints que tienen descriptores con NaN (también quita los descriptores)

    if (previous_pc_keypoints != nullptr)
    {   
        const auto transform_coarse = align_points(previous_pc_keypoints, keypoints, previous_pc_features, descriptors);
        const auto transform_fine = ICP(cloud_filtered, previous_pc, transform_coarse);

        accumulated_distance += calculate_distance(transform_fine, keypoints, previous_pc_keypoints);
        T *= transform_fine;
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
