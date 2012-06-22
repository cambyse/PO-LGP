#ifndef _PERCEPTION_H_
#define _PERCEPTION_H_

#include <JK/utils/masterWorker.h>
#include <biros/logging.h>
#include <hardware/kinect.h>
#include <motion/motion.h>
#include <pcl/ModelCoefficients.h>

SET_LOG(pointcloud, DEBUG);

typedef MT::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;
typedef MT::Array<FittingResult> FittingResultL;

class PointCloudSet : public Variable {
  public:
    PointCloudSet(const char* name) : Variable(name) { reg_point_clouds(); }
    FIELD(PointCloudL, point_clouds);
};
class ObjectSet : public Variable {
  public:
    ObjectSet(const char *name) : Variable(name) { reg_objects(); }
    FIELD(FittingResultL, objects);
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
  
class ObjectFitterIntegrator : public Integrator<FittingResult> {
  public:
    ObjectFitterIntegrator() : Integrator<FittingResult>("ObjectFitter (Integrator)") { birosInfo.getVariable(objects, "Objects", this); }
    PointCloudSet* point_clouds;
    ObjectSet* objects;
    void restart();
    virtual void integrateResult(const FittingResult &r);
};

class ObjectFitterWorker : public Worker<FittingJob, FittingResult> {
  public:
    ObjectFitterWorker();
    void doWork(FittingResult &r, const FittingJob &j);
  private:
    class sObjectFitterWorker *s;

};

class ObjectFitterWorkerFactory : public WorkerFactory<FittingJob, FittingResult> {
  public:
    Worker<FittingJob, FittingResult>* createWorker() { return new ObjectFitterWorker(); }
};

class ObjectFitter : public Master<FittingJob, FittingResult> {
  public:
    ObjectFitter(ObjectFitterWorkerFactory *factory, ObjectFitterIntegrator *integrator, int num_of_workers) :
      Master<FittingJob, FittingResult>(factory, integrator, num_of_workers) {
        birosInfo.getVariable(integrator->point_clouds, "ObjectClusters", integrator);
      };
};

struct ObjectBeliefSet;

class ObjectFilter : public Process {
  public:
    struct sObjectFilter *s;
    ObjectFilter(const char *name) ;
    void open();
    void step();
    void close() {};

    ObjectSet* in_objects;
    ObjectBeliefSet* out_objects;

    
};

class ObjectTransformator : public Process {
  public:
    ObjectTransformator(const char *name) : Process(name) {};
    void open();
    void step();
    void close() {};

    ObjectSet* kinect_objects;
    WorkingCopy<GeometricState> geo;
};

#endif
