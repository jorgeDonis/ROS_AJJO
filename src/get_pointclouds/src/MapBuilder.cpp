#include "MapBuilder.hpp"
#include "Plotter.hpp"

#include <sstream>
#include <chrono>
#include <rosbag/view.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
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
                       double vg_leaf, double feature_r, double sift_min_scale, double sift_octaves, double sift_scales_per_octave,
                       double sift_min_contrast, double inliner_th, double random_sample_keypoints, double RANSAC_iters,
                       double ICP_iters, double ICP_e, double ICP_correspondence_distance, std::string filename, bool use_ICP,
                       KeypointDetector kp_detector, FeatureDetector feature_detector) 
                       :    vg_leaf(vg_leaf),
                            feature_r(feature_r),
                            sift_min_scale(sift_min_scale),
                            sift_octaves(sift_octaves),
                            sift_scales_per_octave(sift_scales_per_octave),
                            sift_min_contrast(sift_min_contrast),
                            inliner_th(inliner_th),
                            random_sample_keypoints(random_sample_keypoints),
                            RANSAC_iters(RANSAC_iters),
                            ICP_e(ICP_e),
                            ICP_iters(ICP_iters),
                            ICP_correspondence_distance(ICP_correspondence_distance),
                            filename(filename),
                            use_ICP(use_ICP),
                            kp_detector(kp_detector),
                            feature_detector(feature_detector)
{
    bag.open(cloud_bag_filename);
    M = PointCloud::Ptr(new PointCloud);
    Plotter::print_simple_vis = true;
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

void purge_features_ffph(FFPHCloud::Ptr& features, PointCloud::Ptr& keypoints)
{
    FFPHCloud::Ptr purgedFeatures(new FFPHCloud);
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

void purge_features_shot(SHOTCloud::Ptr &features, PointCloud::Ptr &keypoints)
{
    SHOTCloud::Ptr purgedFeatures(new SHOTCloud);
    PointCloud::Ptr purgedKeypoints(new PointCloud);
    bool notnan;
    // Loop histogram of each feature
    for (int i = 0; i < features->size(); i++)
    {
        notnan = true;
        // Check for nan values
        for (int j = 0; j < 352; j++)
        {
            if (std::isnan(features->points[i].descriptor[j]))
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

SHOTCloud::Ptr MapBuilder::get_descriptors_shot(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    clock.tik();

    SHOTCloud::Ptr descriptors(new SHOTCloud());
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

    pcl::SHOTEstimationOMP<PointT, NormalT, SHOT> shot;
    shot.setInputCloud(key_points);
    shot.setSearchSurface(full_cloud);
	shot.setInputNormals(normals);
	shot.setSearchMethod(kdtree);
    shot.setRadiusSearch(feature_r);
    shot.setNumberOfThreads(4);

	shot.compute(*descriptors);

    clock.tok();
    printf("Calculado descriptores en %.4f segundos\n", clock.seconds_spent());

    return descriptors;
}

FFPHCloud::Ptr MapBuilder::get_descriptors_ffph(PointCloud::Ptr key_points, PointCloud::ConstPtr full_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    clock.tik();

    FFPHCloud::Ptr descriptors(new FFPHCloud());
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

    pcl::FPFHEstimationOMP<PointT, NormalT, FFPH> shot;
    shot.setInputCloud(key_points);
    shot.setSearchSurface(full_cloud);
    shot.setInputNormals(normals);
    shot.setSearchMethod(kdtree);
    shot.setRadiusSearch(feature_r);
    shot.setNumberOfThreads(4);

    shot.compute(*descriptors);

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

    switch (kp_detector)
    {
        case KeypointDetector::SIFT3D:
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
        case KeypointDetector::ISS:
        {
            clock.tik();
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
            iss.compute(*keypoints);
            clock.tok();
            printf("Calculado %lu keypoints en %.4f segundos\n", keypoints->size(), clock.seconds_spent());
            return keypoints;
        }
        case KeypointDetector::RANDOM:
        {
            clock.tik();
            PointCloud::Ptr keypoints(new PointCloud);
            pcl::RandomSample<PointT> rs;
            rs.setInputCloud(cloud);
            rs.setSample(17000);
            rs.filter(*keypoints);
            clock.tok();
            printf("Calculado %lu keypoints en %.4f segundos\n", keypoints->size(), clock.seconds_spent());
            return keypoints;
        }
    }
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
MapBuilder::align_points_shot(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
SHOTCloud::Ptr t_0_descriptors, SHOTCloud::Ptr t_1_descriptors)
{
    clock.tik();

    pcl::registration::CorrespondenceEstimation<SHOT, SHOT> ce;
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
    printf("Calculado alineado (grueso) en %.4f segundos\n", clock.seconds_spent());
    return transform;
}

Eigen::Matrix4f
MapBuilder::align_points_ffph(PointCloud::Ptr t_0_keypoints, PointCloud::Ptr t_1_keypoints,
                              FFPHCloud::Ptr t_0_descriptors, FFPHCloud::Ptr t_1_descriptors)
{
    clock.tik();

    pcl::registration::CorrespondenceEstimation<FFPH, FFPH> ce;
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
    printf("Calculado alineado (grueso) en %.4f segundos\n", clock.seconds_spent());
    return transform;
}

Eigen::Matrix4f MapBuilder::ICP(PointNormalCloud::Ptr cloud_t1, PointNormalCloud::Ptr cloud_t0, Eigen::Matrix4f const &transform_coarse)
{
    clock.tik();

    Eigen::Matrix4f transform_fine = Eigen::Matrix4f::Identity();
    PointNormalCloud::Ptr foo_cloud(new PointNormalCloud);
    pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icp;
    icp.setMaxCorrespondenceDistance(ICP_correspondence_distance);
    icp.setMaximumIterations(ICP_iters);
    icp.setTransformationEpsilon(ICP_e);
    icp.setInputSource(cloud_t1);
    icp.setInputTarget(cloud_t0);
    icp.align(*foo_cloud, transform_coarse);
    transform_fine = icp.getFinalTransformation();

    clock.tok();
    printf("Calculado alineado (fino) en %.4f segundos\n", clock.seconds_spent());

    return transform_fine;
}

void MapBuilder::process_cloud(PointCloud::Ptr& cloud)
{
    PointCloud::Ptr cloud_filtered = downsample(cloud, vg_leaf); //menor tamaño de hoja, más puntos
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    if (keypoints->empty())
        return;
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimate_normals(cloud_filtered);
    PointNormalCloud::Ptr point_normal_c(new PointNormalCloud);
    pcl::concatenateFields(*cloud_filtered, *normals, *point_normal_c);
    FFPHCloud::Ptr ffph_features(new FFPHCloud);
    SHOTCloud::Ptr shot_features(new SHOTCloud);
    if (feature_detector == FeatureDetector::FFPH)
    {
        ffph_features = get_descriptors_ffph(keypoints, cloud_filtered, normals);
        purge_features_ffph(ffph_features, keypoints); //quita los keypoints que tienen descriptores con NaN (también quita los descriptores)
    }
    else
    {
        shot_features = get_descriptors_shot(keypoints, cloud_filtered, normals);
        purge_features_shot(shot_features, keypoints);
    }
    if (previous_pc_keypoints != nullptr)
    {   
        if (use_ICP)
        {
            Eigen::Matrix4f transform_coarse;
            if (feature_detector == FeatureDetector::FFPH)
                transform_coarse = align_points_ffph(previous_pc_keypoints, keypoints, previous_pc_ffph_features, ffph_features);
            else
                transform_coarse = align_points_shot(previous_pc_keypoints, keypoints, previous_pc_shot_features, shot_features);
            const auto transform_fine = ICP(point_normal_c, previous_point_normal_c, transform_coarse);

            accumulated_distance += calculate_distance(transform_fine, keypoints, previous_pc_keypoints);
            T *= transform_fine;
            PointCloud::Ptr aligned_t_1_pc_global_frame(new PointCloud);
            pcl::transformPointCloud(*cloud_filtered, *aligned_t_1_pc_global_frame, T);

            *M += *aligned_t_1_pc_global_frame;
            Plotter::simple_vis_cloud = M;
            M = downsample(M, 0.02);
        }
        else
        {
            Eigen::Matrix4f transform_coarse;
            if (feature_detector == FeatureDetector::FFPH)
                transform_coarse = align_points_ffph(previous_pc_keypoints, keypoints, previous_pc_ffph_features, ffph_features);
            else
                transform_coarse = align_points_shot(previous_pc_keypoints, keypoints, previous_pc_shot_features, shot_features);
            const auto transform_fine = ICP(point_normal_c, previous_point_normal_c, transform_coarse);

            accumulated_distance += calculate_distance(transform_coarse, keypoints, previous_pc_keypoints);
            T *= transform_coarse;
            PointCloud::Ptr aligned_t_1_pc_global_frame(new PointCloud);
            pcl::transformPointCloud(*cloud_filtered, *aligned_t_1_pc_global_frame, T);

            *M += *aligned_t_1_pc_global_frame;
            Plotter::simple_vis_cloud = M;
            M = downsample(M, 0.02);
        }
    }
    previous_pc = cloud_filtered;
	previous_pc_ffph_features = ffph_features;
    previous_pc_shot_features = shot_features;
    previous_pc_keypoints = keypoints;
    previous_point_normal_c = point_normal_c;
}

void MapBuilder::build_map()
{
    Clock global_clock;
    global_clock.tik();
    int i = 0;
    for (rosbag::MessageInstance const& m : rosbag::View(bag))
    {
        ++i;
        PointCloud::Ptr cloud = m.instantiate<PointCloud>();
        if (i <= 3)
            process_cloud(cloud);
    }
    global_clock.tok();
    total_time_spent_reconstruction = global_clock.seconds_spent();
    pcl::io::savePCDFileASCII(get_filename(), *Plotter::simple_vis_cloud);
    Plotter::print_simple_vis = false;
}

std::string MapBuilder::get_filename()
{
    if (filename.empty())
    {
        stringstream ss;
        ss << "vg_" << vg_leaf << "|"
        << "feature_r_" << feature_r << "|"
        << "sift_contrast_" << sift_min_contrast << "|"
        << "sift_scale_" << sift_min_scale << "|"
        << "sift_scales_per_oc_" << sift_scales_per_octave << "|"
        << "sift_n_oc_" << sift_octaves << "|"
        << "ransac_iters_" << RANSAC_iters << "|"
        << "random_sample_kp_" << random_sample_keypoints << "|"
        << "icp_i_" << ICP_iters << "|"
        << "icp_e_" << ICP_e << "|"
        << "icp_th_" << ICP_correspondence_distance << "|"
        << "inliner_th_" << inliner_th;
        filename = ss.str();
    }
    return filename + "_" + std::to_string(total_time_spent_reconstruction) + "s_" + std::to_string(accumulated_distance) + "_total_error.pcd";
}
