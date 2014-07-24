#ifndef CLOUDMODEL_H_
#define CLOUDMODEL_H_

#include <vector>
#include <tuple>

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>

class CloudModel {
    //----typedefs/classes----//

    //----members----//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;

    //----methods----//
public:
    CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c = nullptr);
    virtual ~CloudModel() {}
    virtual void setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
    virtual const pcl::PointCloud<pcl::PointXYZRGB>::Ptr getModelCloud() const;

protected:
    // uniform random double in [-1:1]
    static double rand11() { return 2*(double)rand()/RAND_MAX - 1; }

    // uniform random double in [0:1]
    static double rand01() { return (double)rand()/RAND_MAX; }

    // PointXYZRGB to tuple
    static std::tuple<double,double,double,double,double,double> PointXYZRGB_to_tuple(const pcl::PointXYZRGB & point);

    // tuple to PointXYZRGB
    static pcl::PointXYZRGB  PointXYZRGB_to_tuple(const std::tuple<double,double,double,double,double,double> & t);

    // add tuples
    static std::tuple<double,double,double,double,double,double> add_tuples(const std::tuple<double,double,double,double,double,double> & t1,
                                                                            const std::tuple<double,double,double,double,double,double> & t2);

    // subtract tuples
    static std::tuple<double,double,double,double,double,double> subtract_tuples(const std::tuple<double,double,double,double,double,double> & t1,
                                                                                 const std::tuple<double,double,double,double,double,double> & t2);

    // divide tuple
    static std::tuple<double,double,double,double,double,double> multiply_tuple(const std::tuple<double,double,double,double,double,double> & t, const double & d);

    static double weight_function(int i);

public:

    // update
    virtual void update_model(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud);
};

#endif /* CLOUDMODEL_H_ */
