#include "CloudModel.h"

#include "arithmetic.h"

using std::cout;
using std::endl;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::get;

using namespace arithmetic;

CloudModel::CloudModel():
    dying_prob(0),
    smoothing(1),
    model_size(0),
    model_target_size(0),
    model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{}

CloudModel::CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c):
    dying_prob(0),
    smoothing(1),
    model_size(0),
    model_target_size(0),
    model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    if(c!=nullptr) {
        setModelCloud(c);
    }
}

void CloudModel::setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c) {
    if(c!=nullptr) {
        pcl::copyPointCloud(*c,*model_cloud);
        model_size = model_target_size = model_cloud->points.size();
    }
}

void CloudModel::getModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const {
    pcl::copyPointCloud(*model_cloud,*output_cloud);
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

void CloudModel::check_model_size(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud) {
    int input_size = input_cloud->points.size();
    if(!input_cloud || input_size<=0) {
        cout << "Error: invalid or emtpy input cloud" << endl;
        return;
    }
    int current_model_size = model_cloud->points.size();
    if(current_model_size>model_target_size) {
        cout << "Shrink model " << current_model_size << " --> " << model_target_size << endl;
        for(int i=current_model_size; i>model_target_size; --i) {
            int rand_idx = rand()%i;
            auto random_point_position = model_cloud->points.begin();
            auto random_count_position = persistence_counts.begin();
            random_point_position+=rand_idx;
            random_count_position+=rand_idx;
            model_cloud->points.erase(random_point_position);
            persistence_counts.erase(random_count_position);
        }
    } else if(current_model_size<model_target_size) {
        cout << "Grow model " << current_model_size << " --> " << model_target_size << endl;
        for(int i=current_model_size; i<model_target_size; ++i) {
            model_cloud->points.push_back(input_cloud->points[rand()%input_size]);
            persistence_counts.push_back(0);
        }
    } else {
        return;
    }
    model_size = model_cloud->points.size();
    cout << "model size: " << current_model_size << " --> " << model_size << " (" << model_target_size << ")" << endl;
}

void CloudModel::update_model(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud) {
    // check for valid input cloud
    if(!input_cloud || input_cloud->points.size()<=0) {
        cout << "Error: invalid or emtpy input cloud" << endl;
        return;
    }
    // model size
    check_model_size(input_cloud);
    if(model_size==0) {
        cout << "Error: empty model" << endl;
        return;
    }
    // build kd-tree for model
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(model_cloud);
    // go through input data
    vector<tuple<double,double,double,double,double,double> > target_points(model_size,make_tuple(0,0,0,0,0,0));
    vector<double> target_weights(model_size,0);
    for(int idx=0; idx<(int)input_cloud->points.size(); ++idx) {
        //int K = model_size;
        int K = 1;
        vector<int> neighbor_indices(K);
        vector<float> neighbor_squared_distances(K);
        if(kdtree.nearestKSearch(input_cloud->points[idx], K, neighbor_indices, neighbor_squared_distances) > 0 ) {
            for(size_t i = 0; i<neighbor_indices.size (); ++i) {
                double weight = weight_function(i);
                target_weights[neighbor_indices[i]] += weight;
                target_points[neighbor_indices[i]] = add_tuples(target_points[neighbor_indices[i]],multiply_tuple(PointXYZRGB_to_tuple(input_cloud->points[idx]),weight));
            }
        } else {
            cout << "Error could not find nearest neighbor" << endl;
        }
    }
    // compute targets
    for(int idx=0; idx<model_size; ++idx) {
        if(target_weights[idx]==0) {
            //target_points[idx] = PointXYZRGB_to_tuple(input_cloud->points[rand()%input_size]);
            target_points[idx] = PointXYZRGB_to_tuple(model_cloud->points[idx]);
            ++persistence_counts[idx];
        } else {
            persistence_counts[idx] = 0;
            auto source = PointXYZRGB_to_tuple(model_cloud->points[idx]);
            auto target = multiply_tuple(target_points[idx],1./target_weights[idx]);
            auto difference = subtract_tuples(target,source);
            difference = multiply_tuple(difference,smoothing);
            target_points[idx] = add_tuples(source,difference);
        }
    }
    // update
    kMeansUpdate(input_cloud,target_points);
}

void CloudModel::kMeansUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
                              std::vector<std::tuple<double,double,double,double,double,double> > target_points) {
    int input_size = input_cloud->points.size();
    for(int idx=0; idx<model_size; ++idx) {
        if(rand01()<dying_prob || persistence_counts[idx]>=persistence) {
            model_cloud->points[idx] = input_cloud->points[rand()%input_size];
        } else {
            auto target = target_points[idx];
            pcl::PointXYZRGB point(get<3>(target),get<4>(target),get<5>(target));
            //pcl::PointXYZRGB point(0,0,0); // black
            //pcl::PointXYZRGB point(255,0,0); // red
            point.x = get<0>(target) + rand11()*1e-5;
            point.y = get<1>(target) + rand11()*1e-5;
            point.z = get<2>(target) + rand11()*1e-5;
            model_cloud->points[idx] = point;
        }
    }
}
