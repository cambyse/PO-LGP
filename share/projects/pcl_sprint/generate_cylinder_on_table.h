#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace generate_cylinder_on_table {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(double noise = 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cylinder_model();

    void add_table(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, double noise = 1);

    void add_cylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, double noise = 1);

}
