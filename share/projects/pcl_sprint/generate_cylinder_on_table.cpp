#include "generate_cylinder_on_table.h"

namespace generate_cylinder_on_table {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(double noise) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        add_table(cloud, noise);
        add_cylinder(cloud, noise);
        cloud->width = (int)cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cylinder_model() {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        add_cylinder(cloud,0);
        cloud->width = (int)cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    void add_table(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, double noise) {
        for(int x=-50; x<50; ++x) {
            for(int y=-50; y<50; ++y) {
                pcl::PointXYZRGB point(255,0,0);
                point.x = x + (2*drand48()-1)*noise;
                point.y = y + (2*drand48()-1)*noise;
                point.z = (2*drand48()-1)*noise;
                cloud->points.push_back(point);
            }
        }
    }

    void add_cylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, double noise) {
        for(double phi=0; phi<360; phi+=3) {
            for(int z=20; z<50; ++z) {
                pcl::PointXYZRGB point(0,255,0);
                point.x = 10 * cosf (pcl::deg2rad(phi)) + (2*drand48()-1)*noise;
                point.y = 10 * sinf (pcl::deg2rad(phi)) + (2*drand48()-1)*noise;
                point.z = z + (2*drand48()-1)*noise;
                cloud->points.push_back(point);
            }
        }
    }

}
