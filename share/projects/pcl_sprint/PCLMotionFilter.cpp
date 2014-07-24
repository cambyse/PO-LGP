#include "PCLMotionFilter.h"

#include "util.h"

#include <pcl/io/pcd_io.h>

using std::cout;
using std::endl;

PCLMotionFilter::PCLMotionFilter():
    threshold(100),
    filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    last_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{}

void PCLMotionFilter::new_input(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud) {
    if(input_cloud==nullptr) {
        cout << "Error: invalid input cloud" << endl;
    } else if(!input_cloud->isOrganized()) {
        cout << "Error: expecting structured input" << endl;
    } else if(input_cloud->height!=last_input_cloud->height || input_cloud->width!=last_input_cloud->width) {
        cout << "Copied input cloud" << endl;
        pcl::copyPointCloud(*input_cloud,*last_input_cloud);
    } else {
        filtered_cloud->clear();
        for(int col=0; col<(int)input_cloud->width; ++col) {
            for(int row=0; row<(int)input_cloud->height; ++row) {
                if(!pcl_is_valid_point(input_cloud->at(col,row))) {
                    continue;
                }
                double value = metric(input_cloud->at(col,row),last_input_cloud->at(col,row));
                if(value>=threshold) {
                    filtered_cloud->points.push_back(input_cloud->at(col,row));
                }
            }
        }
        pcl::copyPointCloud(*input_cloud,*last_input_cloud);
    }
}

void PCLMotionFilter::get_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const {
    pcl::copyPointCloud(*filtered_cloud,*output_cloud);
}

double PCLMotionFilter::metric(const pcl::PointXYZRGB & p1, const pcl::PointXYZRGB & p2) const {
    auto rgb = p1.getRGBVector3i();
    double delta_r = rgb.x();
    double delta_g = rgb.y();
    double delta_b = rgb.z();
    rgb = p2.getRGBVector3i();
    delta_r -= rgb.x();
    delta_g -= rgb.y();
    delta_b -= rgb.z();
    //return sqrt(pow(delta_r,2)+pow(delta_g,2)+pow(delta_b,2));
    return fabs(delta_r)+fabs(delta_g)+fabs(delta_b);
    // double delta_x = p1.x - p2.x;
    // double delta_y = p1.y - p2.y;
    // double delta_z = p1.z - p2.z;
    //return sqrt(pow(delta_x,2)+pow(delta_y,2)+pow(delta_z,2));
}
