#include "MapBuilder.hpp"

#include <rosbag/view.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

MapBuilder::MapBuilder(std::string const &cloud_bag_filename, PointCloud::Ptr& point_cloud_to_visualize)
    : pc_visu(point_cloud_to_visualize)
 {
    bag.open("point_cloud_messages.bag");
}

void MapBuilder::process_cloud(PointCloud::ConstPtr cloud)
{
    using namespace std;

    PointCloud::Ptr cloud_filtered(new PointCloud);

    cout << "Puntos originales: " << cloud->size() << endl;

    pcl::VoxelGrid<pcl::PointXYZRGB> v_grid;
    v_grid.setInputCloud(cloud);
    v_grid.setLeafSize(0.05f, 0.05f, 0.05f);
    v_grid.filter(*cloud_filtered);

    cout << "Puntos tras VG: " << cloud_filtered->size() << endl;

    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_filtered);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud_filtered);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.05);

    pfh.compute(*descriptors);

    pc_visu = cloud_filtered;

    cout << "nº descriptores: " << descriptors->size() << "\n";
    cout << "nº puntos voxel grid: " << cloud_filtered->size() << endl;
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
