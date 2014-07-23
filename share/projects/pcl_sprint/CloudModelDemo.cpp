#include <vector>
#include <tuple>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "generate_cylinder_on_table.h"

using std::cout;
using std::endl;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::get;

// uniform random double in [-1:1]
double rand11() {
    return 2*(double)rand()/RAND_MAX - 1;
}

// uniform random double in [0:1]
double rand01() {
    return (double)rand()/RAND_MAX;
}

// PointXYZRGB to tuple
tuple<double,double,double,double,double,double> PointXYZRGB_to_tuple(const pcl::PointXYZRGB & point) {
    tuple<double,double,double,double,double,double> ret;
    get<0>(ret) = point.x;
    get<1>(ret) = point.y;
    get<2>(ret) = point.z;
    auto rgb = point.getRGBVector3i();
    get<3>(ret) = (double)rgb.x();
    get<4>(ret) = (double)rgb.y();
    get<5>(ret) = (double)rgb.z();
    return ret;
}

// tuple to PointXYZRGB
pcl::PointXYZRGB  PointXYZRGB_to_tuple(const tuple<double,double,double,double,double,double> & t) {
    pcl::PointXYZRGB point(get<3>(t),get<4>(t),get<5>(t));
    point.x = get<0>(t);
    point.y = get<1>(t);
    point.z = get<2>(t);
    return point;
}

// add tuples
tuple<double,double,double,double,double,double> add_tuples(const tuple<double,double,double,double,double,double> & t1, const tuple<double,double,double,double,double,double> & t2) {
    return make_tuple(get<0>(t1)+get<0>(t2),get<1>(t1)+get<1>(t2),get<2>(t1)+get<2>(t2),get<3>(t1)+get<3>(t2),get<4>(t1)+get<4>(t2),get<5>(t1)+get<5>(t2));
}

// subtract tuples
tuple<double,double,double,double,double,double> subtract_tuples(const tuple<double,double,double,double,double,double> & t1, const tuple<double,double,double,double,double,double> & t2) {
    return make_tuple(get<0>(t1)-get<0>(t2),get<1>(t1)-get<1>(t2),get<2>(t1)-get<2>(t2),get<3>(t1)-get<3>(t2),get<4>(t1)-get<4>(t2),get<5>(t1)-get<5>(t2));
}

// divide tuple
tuple<double,double,double,double,double,double> multiply_tuple(const tuple<double,double,double,double,double,double> & t, const double & d) {
    return make_tuple(get<0>(t)*d,get<1>(t)*d,get<2>(t)*d,get<3>(t)*d,get<4>(t)*d,get<5>(t)*d);
}

double weight_function(int i) {
    return 1./pow(0.1*(i+1),2);

    // if(i==0) {
    //     return 1;
    // } else if(i==1) {
    //     return 0.5;
    // } else {
    //     return 0;
    // }
}

// update
void update_model(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & model_cloud) {
    // build tree for model
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(model_cloud);
    // go through input data
    vector<tuple<double,double,double,double,double,double> > new_points(model_cloud->points.size(),make_tuple(0,0,0,0,0,0));
    vector<double> point_weights(model_cloud->points.size(),0);
    for(int idx=0; idx<(int)input_cloud->points.size(); ++idx) {
        //int K = model_cloud->points.size();
        int K = 10;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if(kdtree.nearestKSearch(input_cloud->points[idx], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            for(size_t i = 0; i<pointIdxNKNSearch.size (); ++i) {
                double weight = weight_function(i);
                // std::cout << "    "  <<   model_cloud->points[ pointIdxNKNSearch[i] ].x
                //           << " " << model_cloud->points[ pointIdxNKNSearch[i] ].y
                //           << " " << model_cloud->points[ pointIdxNKNSearch[i] ].z
                //           << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
                point_weights[pointIdxNKNSearch[i]] += weight;
                new_points[pointIdxNKNSearch[i]] = add_tuples(new_points[pointIdxNKNSearch[i]],multiply_tuple(PointXYZRGB_to_tuple(input_cloud->points[idx]),weight));
            }
        } else {
            cout << "Error could not find nearest neighbor" << endl;
        }
    }
    // update model
    for(int idx=0; idx<(int)model_cloud->points.size(); ++idx) {
        if(point_weights[idx]==0) {
            //cout << "idx=" << idx << " : (" << model_cloud->points[idx].x << "," << model_cloud->points[idx].y << "," << model_cloud->points[idx].z << ") --> <no points>" << endl;
        } else {
            auto source = PointXYZRGB_to_tuple(model_cloud->points[idx]);
            auto target = multiply_tuple(new_points[idx],1./point_weights[idx]);
            auto difference = subtract_tuples(target,source);
            //difference = multiply_tuple(difference,1e-1); // smoothing
            target = add_tuples(source,difference);
            //pcl::PointXYZRGB point(get<3>(target),get<4>(target),get<5>(target));
            pcl::PointXYZRGB point(0,0,0);
            point.x = get<0>(target) + rand11()*1e-5;
            point.y = get<1>(target) + rand11()*1e-5;
            point.z = get<2>(target) + rand11()*1e-5;
            //cout << "idx=" << idx << " : (" << model_cloud->points[idx].x << "," << model_cloud->points[idx].y << "," << model_cloud->points[idx].z << ") --> (" << point.x << "," << point.y << "," << point.z << ")" << endl;
            model_cloud->points[idx] = point;
        }
    }
    //cout << endl;
}

int main(int argn, char ** args) {

    // tuple<double,double,double,double,double,double> t1, t2, t3;

    // get<0>(t1) = 1;
    // get<1>(t1) = 2;
    // get<2>(t1) = 3;
    // get<3>(t1) = 4;
    // get<4>(t1) = 5;
    // get<5>(t1) = 6;

    // get<0>(t2) = 10;
    // get<1>(t2) = 20;
    // get<2>(t2) = 30;
    // get<3>(t2) = 40;
    // get<4>(t2) = 50;
    // get<5>(t2) = 60;

    // t3 = add_tuples(t1,t2);
    // cout << "(" << get<0>(t3) << "," << get<1>(t3) << "," << get<2>(t3) << "," << get<3>(t3) << "," << get<4>(t3) << "," << get<5>(t3) << ")" << endl;

    // t3 = multiply_tuple(t1,100);
    // cout << "(" << get<0>(t3) << "," << get<1>(t3) << "," << get<2>(t3) << "," << get<3>(t3) << "," << get<4>(t3) << "," << get<5>(t3) << ")" << endl;

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
    double noise = 0;    // edge length of cube for random points
    for(int i=0; i<N; ++i) {
        pcl::PointXYZRGB point(0,0,0);
        point.x = rand11()*noise;
        point.y = rand11()*noise;
        point.z = rand11()*noise;
        model_cloud->points.push_back(point);
    }

    // display loop
    while(!viewer->wasStopped()) {
        update_model(input_cloud, model_cloud);
        viewer->updatePointCloud(input_cloud, "input cloud");
        viewer->updatePointCloud(model_cloud, "model cloud");
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return 0;
}
