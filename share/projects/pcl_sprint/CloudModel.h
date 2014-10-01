#ifndef CLOUDMODEL_H_
#define CLOUDMODEL_H_

#include <list>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>

class CloudModel {
    //----typedefs/classes----//

    class ModelPoint {
    public:
        double x, y, z;
        int r, g, b;
        int persistence_counts;
        pcl::PointXYZRGB pcl_point;
        ModelPoint(double x_, double y_, double z_, int r_, int g_, int b_, int p = 0):
        x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), persistence_counts(p), pcl_point(r_,g_,b_)
        {
            pcl_point.x = x;
            pcl_point.y = y;
            pcl_point.z = z;
        }
        ModelPoint(const pcl::PointXYZRGB & point, int p = 0):
        x(point.x), y(point.y), z(point.z), persistence_counts(p)
        {
            auto rgb = point.getRGBVector3i();
            r = (double)rgb.x();
            g = (double)rgb.y();
            b = (double)rgb.z();
        }
    };

    //----members----//
    double dying_prob;     // in [0:1]
    double sol_dying_prob; // in [0:1]
    int persistence;       // in {0,1,...}
    double smoothing;      // in [0:1] 0 being infinitely smooth
    int model_size, model_target_size;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;
    std::vector<ModelPoint> model_points;
    bool use_k_means, use_ICP;
    double nn_dist;        // desired distance to nearest neighbor in model cloud

    //----methods----//
public:
    CloudModel();

    CloudModel(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c);

    virtual ~CloudModel() {}

    virtual void setModelCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c);
    virtual void getModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const;
    virtual pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getModelCloud() const;

    virtual void setDyingProb(double p) { dying_prob = p; }
    virtual double getDyingProb() const { return dying_prob; }

    virtual void setSolDyingProb(double p) { sol_dying_prob = p; }
    virtual double getSolDyingProb() const { return sol_dying_prob; }

    virtual void setPersistence(int p) { persistence = p; }
    virtual int getPersistence() const { return persistence; }

    virtual void setSmoothing(double s) { smoothing = s; }
    virtual double getSmoothing() const { return smoothing; }

    virtual void setModelSize(double s) { model_target_size = s; }
    virtual double getModelSize() const { return model_size; }

    virtual void setKMeans(bool b) { use_k_means = b; }
    virtual bool getKMeans() const { return use_k_means; }

    virtual void setICP(bool b) { use_ICP = b; }
    virtual bool getICP() const { return use_ICP; }

    virtual void setNNDist(double d) { nn_dist = d; }
    virtual double getNNDist() const { return nn_dist; }

protected:
    // uniform random double in [-1:1]
    static double rand11() { return 2*(double)rand()/RAND_MAX - 1; }

    // uniform random double in [0:1]
    static double rand01() { return (double)rand()/RAND_MAX; }

    static double weight_function(int i);

    virtual void check_model_size(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud);

public:

    // update
    virtual void update_model(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud);

protected:

    // k-means update
    virtual void kMeansUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud,
                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & target_cloud);

    // ICP update
    virtual void icpUpdate(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud);

    // build model cloud from points
    virtual void build_cloud_from_points();

    // build model points from cloud
    virtual void build_points_from_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);
    virtual void update_points_from_cloud();

};

#endif /* CLOUDMODEL_H_ */
