#include "Plotter.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

PointCloud::Ptr Plotter::unmerged_clouds = PointCloud::Ptr(new PointCloud);
PointCloud::Ptr Plotter::new_cloud = PointCloud::Ptr(new PointCloud);
PointCloud::Ptr Plotter::merged_clouds = PointCloud::Ptr(new PointCloud);
PointCloud::Ptr Plotter::simple_vis_cloud = PointCloud::Ptr(new PointCloud);


void Plotter::plot_correspondences(PointCloud::Ptr cloud_1, PointCloud::Ptr cloud_2, pcl::CorrespondencesConstPtr correspondences)
{
    std::function<void()> f1 = [cloud_1, cloud_2, correspondences] ()
    {
        pcl::visualization::PCLVisualizer::Ptr custom_viewer(new pcl::visualization::PCLVisualizer);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_A(cloud_1, 37, 204, 81); //verde
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_B(cloud_2, 257, 50, 59); //rojo
        custom_viewer->setBackgroundColor(24, 12, 2);
        custom_viewer->addPointCloud(cloud_1, color_A, "previous_cloud");
        custom_viewer->addPointCloud(cloud_2, color_B, "current_cloud");
        custom_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "previous_cloud");
        custom_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "current_cloud");

        for (auto correspondence = correspondences->cbegin(); correspondence < correspondences->end(); ++correspondence)
        {
            pcl::PointXYZRGB p1 = cloud_1->points.at(correspondence->index_query);
            pcl::PointXYZRGB p2 = cloud_2->points.at(correspondence->index_match);
            custom_viewer->addLine(p1, p2, "line_" + std::to_string(correspondence->index_query) + "_" + std::to_string(correspondence->index_match));
        }

        custom_viewer->addCoordinateSystem(1.0);
        custom_viewer->initCameraParameters();
        custom_viewer->setCameraPosition(0, 0, 0, 0, 0, 0);
        custom_viewer->spin();
    };
    boost::thread t(f1);
    t.join();
}

void Plotter::plot_normals(PointCloud::ConstPtr surface, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    std::function<void()> f1 = [surface, normals]()
    {
        pcl::visualization::PCLVisualizer viewer("Normales");
        viewer.setBackgroundColor(0.0, 0.0, 0.5);
        viewer.addPointCloud(surface, "surface");
        viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(surface, normals);
        viewer.spin();
    };
    boost::thread t(f1);
    t.join();
}

void Plotter::plot_transformation()
{
    std::function<void()> f1 = [] () {
        pcl::visualization::PCLVisualizer viewer("Merge");
        
        int viewport_1(0);
        viewer.createViewPort(0.5, 0.5, 1, 1, viewport_1);
        viewer.setBackgroundColor(0.0, 0.0, 0.0, viewport_1);
        printf("%lu puntos\n", Plotter::unmerged_clouds->size());
        viewer.addPointCloud(Plotter::unmerged_clouds, "cloud_1", viewport_1);
        viewer.addText("Known map + new cloud (before transformation)", 10, 10, "text_t0", viewport_1);
        viewer.addCoordinateSystem(1.0, "1-1", viewport_1);

        int viewport_2(0);
        viewer.createViewPort(0.5, 0, 1, 0.5, viewport_2);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, viewport_2);
        viewer.addPointCloud(Plotter::new_cloud, "cloud_2", viewport_2);
        viewer.addText("New cloud", 10, 10, "text_t1", viewport_2);
        viewer.addCoordinateSystem(1.0, "2-2", viewport_2);

        int viewport_3(0);
        viewer.createViewPort(0, 0, 0.5, 1, viewport_3);
        viewer.setBackgroundColor(0.2, 0.2, 0.3, viewport_3);
        viewer.addPointCloud(Plotter::merged_clouds, "cloud_3", viewport_3);
        viewer.addText("Known map + new cloud (after transformation)", 10, 10, "text_merge", viewport_3);
        viewer.addCoordinateSystem(1.0, "3-3", viewport_3);

        // viewer.initCameraParameters();
        viewer.spin();
    };
    boost::thread t(f1);
    t.join();
}

void Plotter::simple_vis()
{
    pcl::visualization::CloudViewer viewer("Reconstrucción 3D");
    while (!viewer.wasStopped())
    {
        viewer.showCloud(Plotter::simple_vis_cloud);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}