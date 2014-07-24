#include "pcl_smoothing.h"

#include <pcl/io/pcd_io.h>

#include <deque>

#include "arithmetic.h"
#include "util.h"

using std::deque;
using std::cout;
using std::endl;
using std::tuple;

using namespace arithmetic;

void box_smoothing(int width, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) {
    // sanity checks
    if(input_cloud==nullptr) {
        cout << "Error: invalid input cloud" << endl;
        return;
    } else if(!input_cloud->isOrganized()) {
        cout << "Error: expecting structured input" << endl;
        return;
    } else if(width%2!=1 || width<=0) {
        pcl::copyPointCloud(*input_cloud,*output_cloud);
        cout << "Error: width must be an odd larger than zero number" << endl;
        return;
    }

    // copy
    pcl::copyPointCloud(*input_cloud,*output_cloud);

    // perform smoothing
    int margin = (width-1)/2;
    int in_width = output_cloud->width;
    int in_height = output_cloud->height;
    deque<tuple<double,double,double,double,double,double> > value_box;
    deque<double> weight_box;
    // row box
    for(int col=0; col<in_width; ++col) {                                   // XXX
        for(int row=-margin; row<in_height; ++row) {                        // XXX
            // add new point
            if(row+margin<in_height) {                                      // XXX
                // determine weight
                double weight = 1;
                if(!pcl_is_valid_point(output_cloud->at(col,row+margin))) {   // XXX
                    weight = 0;
                }
                // determine value
                auto value = multiply_tuple(PointXYZRGB_to_tuple(output_cloud->at(col,row+margin)),weight); // XXX
                // increase box
                value_box.push_back(value);
                weight_box.push_back(weight);
            }
            // reduce box
            if((int)value_box.size()>width) {
                value_box.pop_front();
                weight_box.pop_front();
            }
            // assign value
            if(row>=0) {                                                    // XXX
                if(pcl_is_valid_point(output_cloud->at(col,row))) {
                    // calculate mean value
                    double weight_sum = 0;
                    for(double w : weight_box) {
                        weight_sum += w;
                    }
                    auto value_sum = tuple<double,double,double,double,double,double>(0,0,0,0,0,0);
                    for(auto v : value_box) {
                        value_sum = add_tuples(value_sum,v);
                    }
                    value_sum = multiply_tuple(value_sum,1./weight_sum);
                    // assign
                    output_cloud->at(col,row) = tuple_to_PointXYZRGB(value_sum);
                }
            }
        }
        value_box.clear();
        weight_box.clear();
    }
    // col box -- copy/past from above (except XXX lines) and swap for-loops
    for(int row=0; row<in_height; ++row) {                                  // XXX
        for(int col=-margin; col<in_width; ++col) {                         // XXX
            // add new point
            if(col+margin<in_width) {                                       // XXX
                // determine weight
                double weight = 1;
                if(!pcl_is_valid_point(output_cloud->at(col+margin,row))) {   // XXX
                    weight = 0;
                }
                // determine value
                auto value = multiply_tuple(PointXYZRGB_to_tuple(output_cloud->at(col+margin,row)),weight); // XXX
                // increase box
                value_box.push_back(value);
                weight_box.push_back(weight);
            }
            // reduce box
            if((int)value_box.size()>width) {
                value_box.pop_front();
                weight_box.pop_front();
            }
            // assign value
            if(col>=0) {                                                    // XXX
                if(pcl_is_valid_point(output_cloud->at(col,row))) {
                    // calculate mean value
                    double weight_sum = 0;
                    for(double w : weight_box) {
                        weight_sum += w;
                    }
                    auto value_sum = tuple<double,double,double,double,double,double>(0,0,0,0,0,0);
                    for(auto v : value_box) {
                        value_sum = add_tuples(value_sum,v);
                    }
                    value_sum = multiply_tuple(value_sum,1./weight_sum);
                    // assign
                    output_cloud->at(col,row) = tuple_to_PointXYZRGB(value_sum);
                }
            }
        }
        value_box.clear();
        weight_box.clear();
    }
}
