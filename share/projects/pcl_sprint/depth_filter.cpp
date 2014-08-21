#include "depth_filter.h"

#include <pcl/io/pcd_io.h>

using std::cout;
using std::endl;

void depth_filter(double depth,
                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) {
    if(input_cloud==nullptr) {
        cout << "Error: invalid input cloud" << endl;
        return;
    }
    int counter = 0;
    output_cloud->clear();
    for(auto point : input_cloud->points) {
        if(point.z<depth && point.z>0) {
            output_cloud->points.push_back(point);
            ++counter;
        }
    }
    //cout << "filter: " << counter << " points left" << endl;
}

void depth_filter(double depth,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    depth_filter(depth,cloud,output_cloud);
    cloud = output_cloud;
}
