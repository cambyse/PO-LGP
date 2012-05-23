#include <JK/utils/masterWorker.h>
#include <hardware/kinect.h>

typedef MT::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;

class PointCloudSet : public Variable {
  FIELD(PointCloudL, point_clouds);
};
class ObjectSet : public Variable {
  FIELD(arr, point_clouds);
};
class ObjectClusterer : public Process {
  public:
    PointCloud* data_3d;
    PointCloudSet* point_clouds;

    void open();
    void step();
    void close();
};
  
class ObjectFitterMaster : public Master<FittingJob, FittingResult> {
  PointCloudSet* point_clouds;
  ObjectSet* objects;
  //virtual int hasNextJob();
  //virtual int hasWorkingJob();
  //virtual FittingJob createJob();
  virtual void integrateResult(const FittingResult &r);
};

class ObjectFitterWorker : public Worker<FittingJob, FittingResult> {
  public:
    void doWork(FittingResult &r, const FittingJoba &j);
};
