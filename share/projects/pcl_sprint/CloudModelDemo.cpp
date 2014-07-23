#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "generate_cylinder_on_table.h"
#include "CloudModel.h"

using std::cout;
using std::endl;

int main(int argn, char ** args) {

    // random seed
    srand(time(nullptr));

    // set up viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(100,100,100,-100,-100,100);
    viewer->setBackgroundColor (0.3, 0.3, 0.3);

    // get input point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud = generate_cylinder_on_table::get_point_cloud();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb_input(input_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud, rgb_input, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input cloud");

    // get model point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_model(model_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(model_cloud, rgb_model, "model cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "model cloud");

    // initialize cloud model
    int N = 100;         // number of points in model
    for(int i=0; i<N; ++i) {
        pcl::PointXYZRGB point(0,0,0);
        point.x = 0;
        point.y = 0;
        point.z = 0;
        model_cloud->points.push_back(point);
    }

    // display loop
    while(!viewer->wasStopped()) {
        CloudModel::update_model(input_cloud, model_cloud);
        viewer->updatePointCloud(input_cloud, "input cloud");
        viewer->updatePointCloud(model_cloud, "model cloud");
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return 0;
}
