#include "CloudModel.h"

#include "arithmetic.h"

#include <limits>

#include <pcl/registration/icp.h>

using std::cout;
using std::endl;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::get;

using namespace arithmetic;

CloudModel::CloudModel():
    dying_prob(0),
    sol_dying_prob(0),
    persistence(numeric_limits<int>::max()),
    smoothing(1),
    model_size(0),
    model_target_size(0),
    model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
    model_points(),
    use_k_means(true),
    use_ICP(true),
    nn_dist(-1)
{}

CloudModel::CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c):
    dying_prob(0),
    sol_dying_prob(0),
    persistence(numeric_limits<int>::max()),
    smoothing(1),
    model_size(0),
    model_target_size(0),
    model_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
    model_points(),
    use_k_means(true),
    use_ICP(true),
    nn_dist(-1)
{
    if(c!=nullptr) {
        setModelCloud(c);
    }
}

void CloudModel::setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c) {
    build_points_from_cloud(c);
    build_cloud_from_points();
    model_size = model_target_size = model_points.size();
}

void CloudModel::getModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const {
    pcl::copyPointCloud(*model_cloud,*output_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr CloudModel::getModelCloud() const {
    return model_cloud;
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

void CloudModel::check_model_size(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud) {
    int input_size = input_cloud->points.size();
    if(!input_cloud || input_size<=0) {
        cout << "Error: invalid or emtpy input cloud" << endl;
        return;
    }
    int current_model_size = model_points.size();
    if(current_model_size>model_target_size) {
        cout << "Shrink model " << current_model_size << " --> " << model_target_size << endl;
        for(int i=current_model_size; i>model_target_size; --i) {
            int rand_idx = rand()%i;
            auto random_point_position = model_points.begin();
            random_point_position+=rand_idx;
            model_points.erase(random_point_position);
        }
        build_cloud_from_points();
    } else if(current_model_size<model_target_size) {
        cout << "Grow model " << current_model_size << " --> " << model_target_size << endl;
        for(int i=current_model_size; i<model_target_size; ++i) {
            model_points.push_back(ModelPoint(input_cloud->points[rand()%input_size],0));
        }
        build_cloud_from_points();
    } else {
        return;
    }
    model_size = model_points.size();
    cout << "model size: " << current_model_size << " --> " << model_size << " (" << model_target_size << ")" << endl;
}

void CloudModel::update_model(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud) {
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
    // ICP step
    if(use_ICP) {
        icpUpdate(input_cloud);
    }
    // k-means step
    if(use_k_means) {
        // add/remove model points based on desired NN-distance
        if(nn_dist>0) {
            // build kd-tree for current model
            pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
            kdtree.setInputCloud(model_cloud);
            // find NN
            decltype(model_points) new_model_points;
            for(int idx=0; idx<model_size; ++idx) {
                // NN search
                int K = 2; // because the NN is the point itself
                vector<int> neighbor_indices(K);
                vector<float> neighbor_squared_distances(K);
                int n_neighbors = kdtree.nearestKSearch(model_points[idx].pcl_point, K, neighbor_indices, neighbor_squared_distances);
                // insert/delete
                if(n_neighbors==K && neighbor_squared_distances[1]<nn_dist*nn_dist/2) { // too close
                    // delete
                    if(drand48()<0.1) {
                        // delete
                        //cout << "-";
                    } else {
                        new_model_points.push_back(model_points[idx]);
                        //cout << "o";
                    }
                } else if(n_neighbors<K || neighbor_squared_distances[1]>nn_dist*nn_dist*2) { // too far
                    // reinsert current point in a random cube of desired distance
                    auto p = model_points[idx];
                    new_model_points.push_back(p);
                    if(drand48()<0.1) {
                        p.x += rand11()*nn_dist;
                        p.y += rand11()*nn_dist;
                        p.z += rand11()*nn_dist;
                        new_model_points.push_back(p);
                        //cout << "+";
                    } else {
                        //cout << "o";
                    }
                } else { // ok
                    new_model_points.push_back(model_points[idx]);
                    //cout << "o";
                }
                //cout << "(" << neighbor_squared_distances[1] << "/" << nn_dist*nn_dist << ")";
                //cout << "(" << model_points[idx].x << "," << model_points[idx].y << "," << model_points[idx].z << ")";
                if(n_neighbors<K) {
                    cout << "Found only " << n_neighbors << " neighbors" << endl;
                }
            }
            //cout << endl;
            std::swap(model_points,new_model_points);
            // take change into account
            build_cloud_from_points();
            if(model_size!=(int)model_points.size()) {
                cout << "model size: " << model_size << " --> " << model_points.size() << endl;
                model_target_size = model_size = model_points.size();
            }
            if(model_size<1) {
                setModelSize(1);
                check_model_size(input_cloud);
            }
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
        int input_size = input_cloud->points.size();
        for(int idx=0; idx<model_size; ++idx) {
            if(target_weights[idx]==0) {
                if(rand01()<sol_dying_prob) {
                    target_points[idx] = PointXYZRGB_to_tuple(input_cloud->points[rand()%input_size]);
                    model_points[idx].persistence_counts = 0;
                } else {
                    target_points[idx] = PointXYZRGB_to_tuple(model_cloud->points[idx]);
                    model_points[idx].persistence_counts += 1;
                }
            } else {
                model_points[idx].persistence_counts = 0;
                auto source = PointXYZRGB_to_tuple(model_cloud->points[idx]);
                auto target = multiply_tuple(target_points[idx],1./target_weights[idx]);
                auto difference = subtract_tuples(target,source);
                difference = multiply_tuple(difference,smoothing);
                target_points[idx] = add_tuples(source,difference);
            }
        }
        // construct target cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for(auto target : target_points) {
            pcl::PointXYZRGB point(get<3>(target),get<4>(target),get<5>(target));
            //pcl::PointXYZRGB point(0,0,0); // black
            //pcl::PointXYZRGB point(255,0,0); // red
            point.x = get<0>(target) + rand11()*1e-5;
            point.y = get<1>(target) + rand11()*1e-5;
            point.z = get<2>(target) + rand11()*1e-5;
            target_cloud->points.push_back(point);
        }
        // update
        kMeansUpdate(input_cloud,target_cloud);
    }
}

void CloudModel::kMeansUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud,
                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & target_cloud) {
    int input_size = input_cloud->points.size();
    for(int idx=0; idx<model_size; ++idx) {
        if(rand01()<dying_prob || model_points[idx].persistence_counts>=persistence) {
            model_cloud->points[idx] = input_cloud->points[rand()%input_size];
        } else {
            model_cloud->points[idx] = target_cloud->points[idx];
        }
    }
    update_points_from_cloud();
}

void CloudModel::icpUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    // set thresholds
    // cout << "old thresholds (corr.dist./ransac) = (" <<
    //     icp.getMaxCorrespondenceDistance() << "/" <<
    //     icp.getRANSACOutlierRejectionThreshold() << ")" << endl;
    // icp.setRANSACOutlierRejectionThreshold(10);
    //icp.setMaxCorrespondenceDistance(0.1);
    // cout << "new thresholds (corr.dist./ransac) = (" <<
    //     icp.getMaxCorrespondenceDistance() << "/" <<
    //     icp.getRANSACOutlierRejectionThreshold() << ")" << endl;
    // set max iterations
    // cout << "old iterations (corr.dist./ransac) = (" <<
    //     icp.getMaximumIterations() << "/" <<
    //     icp.getRANSACIterations() << ")" << endl;
    icp.setMaximumIterations(1);
    icp.setRANSACIterations(0);
    // cout << "new iterations (corr.dist./ransac) = (" <<
    //     icp.getMaximumIterations() << "/" <<
    //     icp.getRANSACIterations() << ")" << endl;
    // set input and target
    icp.setInputCloud(model_cloud);
    icp.setInputTarget(input_cloud);
    // perform alignment
    icp.align(*model_cloud);
    // print some info
    // cout << "has converged:" << icp.hasConverged() << " score: " <<
    //     icp.getFitnessScore() << endl;
    // cout << "Final Transform:" << endl << icp.getFinalTransformation() << endl;
    update_points_from_cloud();
}

void CloudModel::build_cloud_from_points() {
    model_cloud->clear();
    for(ModelPoint p : model_points) {
        model_cloud->points.push_back(p.pcl_point);
    }
}

void CloudModel::build_points_from_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) {
    model_points.clear();
    for(auto p : cloud->points) {
        model_points.push_back(ModelPoint(p,0));
    }
    model_size = model_points.size();
}

void CloudModel::update_points_from_cloud() {
    if(model_points.size()!=model_cloud->points.size()) {
        cout << "Error: model points and model cloud have unequal size" << endl;
        build_points_from_cloud(model_cloud);
    } else {
        for(int idx=0; idx<model_size; ++idx) {
            model_points[idx] = ModelPoint(model_cloud->points[idx],model_points[idx].persistence_counts);
        }
    }
}
