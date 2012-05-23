#ifndef _PERCEPTION_H_
#define _PERCEPTION_H_

#include <JK/utils/masterWorker.h>
#include <hardware/kinect.h>
#include <pcl/ModelCoefficients.h>

typedef MT::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;

//forward Declaration
class ObjectFitterMaster;

class PointCloudSet : public Variable {
  public:
    PointCloudSet(const char* name) : Variable(name) { reg_point_clouds(); }
    FIELD(PointCloudL, point_clouds);
};
class ObjectSet : public Variable {
  public:
    ObjectSet(const char *name) : Variable(name) { reg_objects(); }
    FIELD(arr, objects);
};
class ObjectClusterer : public Process {
  public:
    ObjectClusterer();
    PointCloudVar* data_3d;
    PointCloudSet* point_clouds;

    void open();
    void step();
    void close();
};

//class ObjectFitter : public Process {
  //public:
    //ObjectFitter() : Process("ObjectFitter") {}
    //PointCloudSet* point_clouds;
    //ObjectSet* objects;
    //ObjectFitterMaster* master;
    //void open();
    //void step();
    //void close();
    
//};
  
//class ObjectFitterMaster : public Master<FittingJob, FittingResult> {
  //public:
    //ObjectFitterMaster(WorkerFactory<FittingJob, FittingResult> *factory, int num_of_workers) : Master<FittingJob, FittingResult>("ObjectFitter (Master)", factory, num_of_workers) {}
    //PointCloudSet* point_clouds;
    //ObjectSet* objects;
    //virtual int hasNextJob();
    //virtual int hasWorkingJob();
    //virtual FittingJob createJob();
    //virtual void integrateResult(const FittingResult &r);
//};

//class ObjectFitterWorker : public Worker<FittingJob, FittingResult> {
  //public:
    //ObjectFitterWorker() : Worker<FittingJob, FittingResult>("ObjectFitter (Worker)") {}
    //void doWork(FittingResult &r, const FittingJob &j);
//};

#endif
