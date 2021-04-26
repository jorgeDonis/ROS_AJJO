#include "MapBuilder.hpp"

#include <rosbag/view.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/shot.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/pfh.h>

MapBuilder::MapBuilder(std::string const &cloud_bag_filename, PointCloud::Ptr& point_cloud_to_visualize)
    : pc_visu(point_cloud_to_visualize)
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

PointCloud::Ptr get_keypoints(PointCloud::ConstPtr cloud)
{
    // Object for storing the keypoints.
    PointCloud::Ptr keypoints(new PointCloud);

    // ISS keypoint detector object.
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setSalientRadius(6 * resolution);
    // Set the radius for the application of the non maxima supression algorithm.
    detector.setNonMaxRadius(4 * resolution);
    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    detector.setMinNeighbors(5);
    // Set the upper bound on the ratio between the second and the first eigenvalue.
    detector.setThreshold21(0.975);
    // Set the upper bound on the ratio between the third and the second eigenvalue.
    detector.setThreshold32(0.975);
    // Set the number of prpcessing threads to use. 0 sets it to automatic.
    detector.setNumberOfThreads(0);

    detector.compute(*keypoints);
    return keypoints;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr get_descriptors(PointCloud::Ptr cloud)
{
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.05);

    pfh.compute(*descriptors);
    return descriptors;
}

PointCloud::Ptr downsample(PointCloud::ConstPtr cloud)
{
	PointCloud::Ptr cloud_filtered(new PointCloud);
	pcl::VoxelGrid<pcl::PointXYZRGB> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(0.025f, 0.025f, 0.025f);
	v_grid.filter(*cloud_filtered);
	return cloud_filtered;
}

void MapBuilder::process_cloud(PointCloud::ConstPtr cloud)
{
    using namespace std;

	PointCloud::Ptr cloud_filtered = downsample(cloud);
    PointCloud::Ptr keypoints = get_keypoints(cloud_filtered);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors = get_descriptors(keypoints);

    if (previous_pc_features != nullptr)
    {
		cout << "Keypoints[i-1]: " << previous_pc_keypoints->size() << endl;
		cout << "Características[i-1]: " << previous_pc_features->size() << endl;
		cout << "Keypoints[i]: " << keypoints->size() << endl;
		cout << "Características[i]: " << descriptors->size() << endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedModel(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Note: here you would compute or load the descriptors for both
        // the scene and the model. It has been omitted here for simplicity.

        // Object for pose estimation.
        pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::PFHSignature125> pose;
        pose.setInputSource(previous_pc_keypoints);
        pose.setInputTarget(keypoints);
        pose.setSourceFeatures(previous_pc_features);
        pose.setTargetFeatures(descriptors);
        // // Instead of matching a descriptor with its nearest neighbor, choose randomly between
        // // the N closest ones, making it more robust to outliers, but increasing time.
        pose.setCorrespondenceRandomness(20);
        // // Set the fraction (0-1) of inlier points required for accepting a transformation.
        // // At least this number of points will need to be aligned to accept a pose.
        pose.setInlierFraction(0.55f);
        // // Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
        pose.setNumberOfSamples(3);
        // // Set the similarity threshold (0-1) between edge lengths of the polygons. The
        // // closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
        pose.setSimilarityThreshold(0.8f);
        // // Set the maximum distance threshold between two correspondent points in source and target.
        // // If the distance is larger, the points will be ignored in the alignment process.
        pose.setMaxCorrespondenceDistance(0.0001f);

        
        pose.align(*alignedModel);

        if (pose.hasConverged())
        {
            Eigen::Matrix4f transformation = pose.getFinalTransformation();
            Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
            Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

            std::cout << "Transformation matrix:" << std::endl
                      << std::endl;
            printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
            printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
            printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
            std::cout << std::endl;
            printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
        }
        else
            std::cout << "Did not converge." << std::endl;
    }
	previous_pc_features = descriptors;
    previous_pc_keypoints = keypoints;
	
    pc_visu = cloud_filtered;
}

void MapBuilder::build_map()
{
    ros::Rate rate(FPS);
    for (rosbag::MessageInstance const& m : rosbag::View(bag))
    {
        PointCloud::ConstPtr cloud = m.instantiate<PointCloud>();
        process_cloud(cloud);
        sleep(1000 / FPS);
    }
}
