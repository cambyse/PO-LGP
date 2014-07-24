#ifndef CLOUDMODEL_H_
#define CLOUDMODEL_H_

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>

class CloudModel {
    //----typedefs/classes----//

    //----members----//
    double dying_prob;     // in [0:1]
    double sol_dying_prob; // in [0:1]
    int persistence;       // in {0,1,...}
    double smoothing;      // in [0:1] 0 being infinitely smooth
    int model_size, model_target_size;
    std::vector<int> persistence_counts;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;

    //----methods----//
public:
    CloudModel();

    CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);

    virtual ~CloudModel() {}

    virtual void setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
    virtual void getModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const;

    virtual void setDyingProb(double p) { dying_prob = p; }
    virtual double getDyingProb() { return dying_prob; }

    virtual void setSolDyingProb(double p) { sol_dying_prob = p; }
    virtual double getSolDyingProb() { return sol_dying_prob; }

    virtual void setPersistence(int p) { persistence = p; }
    virtual int getPersistence() { return persistence; }

    virtual void setSmoothing(double s) { smoothing = s; }
    virtual double getSmoothing() { return smoothing; }

    virtual void setModelSize(double s) { model_target_size = s; }
    virtual double getModelSize() { return model_size; }

protected:
    // uniform random double in [-1:1]
    static double rand11() { return 2*(double)rand()/RAND_MAX - 1; }

    // uniform random double in [0:1]
    static double rand01() { return (double)rand()/RAND_MAX; }

    static double weight_function(int i);

    virtual void check_model_size(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud);

public:

    // update
    virtual void update_model(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud);

protected:

    // k-means update
    virtual void kMeansUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & target_cloud);

    // ICP update
    virtual void icpUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud);

};

#endif /* CLOUDMODEL_H_ */
