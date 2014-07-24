#include "CloudModel.h"

using std::cout;
using std::endl;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::get;

CloudModel::CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c):
    model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    if(c!=nullptr) {
        pcl::copyPointCloud(*c,*model_cloud);
    }
}

void CloudModel::setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c) {
    if(c!=nullptr) {
        pcl::copyPointCloud(*c,*model_cloud);
    }
}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudModel::getModelCloud() const {
    return model_cloud;
}

tuple<double,double,double,double,double,double> CloudModel::PointXYZRGB_to_tuple(const pcl::PointXYZRGB & point) {
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

pcl::PointXYZRGB  CloudModel::PointXYZRGB_to_tuple(const tuple<double,double,double,double,double,double> & t) {
    pcl::PointXYZRGB point(get<3>(t),get<4>(t),get<5>(t));
    point.x = get<0>(t);
    point.y = get<1>(t);
    point.z = get<2>(t);
    return point;
}

tuple<double,double,double,double,double,double> CloudModel::add_tuples(
    const tuple<double,double,double,double,double,double> & t1,
    const tuple<double,double,double,double,double,double> & t2) {
    return make_tuple(get<0>(t1)+get<0>(t2),
                      get<1>(t1)+get<1>(t2),
                      get<2>(t1)+get<2>(t2),
                      get<3>(t1)+get<3>(t2),
                      get<4>(t1)+get<4>(t2),
                      get<5>(t1)+get<5>(t2));
}

tuple<double,double,double,double,double,double> CloudModel::subtract_tuples(const tuple<double,double,double,double,double,double> & t1,
                                                                             const tuple<double,double,double,double,double,double> & t2) {
    return make_tuple(get<0>(t1)-get<0>(t2),
                      get<1>(t1)-get<1>(t2),
                      get<2>(t1)-get<2>(t2),
                      get<3>(t1)-get<3>(t2),
                      get<4>(t1)-get<4>(t2),
                      get<5>(t1)-get<5>(t2));
}

tuple<double,double,double,double,double,double> CloudModel::multiply_tuple(const tuple<double,double,double,double,double,double> & t, const double & d) {
    return make_tuple(get<0>(t)*d,get<1>(t)*d,get<2>(t)*d,get<3>(t)*d,get<4>(t)*d,get<5>(t)*d);
}

double CloudModel::weight_function(int i) {
    return 1./pow(0.1*(i+1),2);

    // if(i==0) {
    //     return 1;
    // } else if(i==1) {
    //     return 0.5;
    // } else {
    //     return 0;
    // }
}

void CloudModel::update_model(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud) {
    // build tree for model
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(model_cloud);
    // go through input data
    vector<tuple<double,double,double,double,double,double> > new_points(model_cloud->points.size(),make_tuple(0,0,0,0,0,0));
    vector<double> point_weights(model_cloud->points.size(),0);
    for(int idx=0; idx<(int)input_cloud->points.size(); ++idx) {
        //int K = model_cloud->points.size();
        int K = 10;
        vector<int> pointIdxNKNSearch(K);
        vector<float> pointNKNSquaredDistance(K);
        if(kdtree.nearestKSearch(input_cloud->points[idx], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            for(size_t i = 0; i<pointIdxNKNSearch.size (); ++i) {
                double weight = weight_function(i);
                // cout << "    "  <<   model_cloud->points[ pointIdxNKNSearch[i] ].x
                //           << " " << model_cloud->points[ pointIdxNKNSearch[i] ].y
                //           << " " << model_cloud->points[ pointIdxNKNSearch[i] ].z
                //           << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << endl;
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
