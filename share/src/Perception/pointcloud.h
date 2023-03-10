#ifndef _POINTCLOUD_H__
#define _POINTCLOUD_H__

#ifdef MLR_PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <Core/thread.h>
#include <devTools/logging.h>
//#include <Hardware/kinect.h>
#include <KOMO/komo.h>

SET_LOG(pointcloud, INFO)

typedef pcl::PointXYZRGBA PointT;
typedef mlr::Array<pcl::PointCloud<PointT>::Ptr> PointCloudL;
typedef pcl::PointCloud<PointT>::Ptr FittingJob;
typedef pcl::ModelCoefficients::Ptr FittingResult;
typedef mlr::Array<FittingResult> FittingResultL;
const int RADIUS = 2;
const int HEIGHT = 3;

struct ObjectBelief {

  ObjectBelief() {
    shapeParams.resize(4);
  }
  //pose
  // TODO: make pointers
  mlr::Vector position;
  mlr::Quaternion rotation;

  arr poseCov;

  // primitive shapes
  mlr::ShapeType shapeType;
  arr shapeParams;

  // TODO: make pointer, such that the using app does not need to implicitly
  // include half of the PCL?
  //pcl::ModelCoefficients::Ptr pcl_object;

  //pcl::PointCloud<PointT>* pointCloud;
  arr vertices;
  uintA triangles;
};

typedef mlr::Array<ObjectBelief*> ObjectBeliefSet;
typedef pcl::PointCloud<PointT>::Ptr PointCloudVar;
typedef PointCloudL PointCloudSet;
typedef FittingResultL ObjectSet;

// -- Processes

struct ObjectClusterer : public Module {
  ObjectClusterer();
  ACCESS(PointCloudVar, data_3d)
  ACCESS(PointCloudSet, point_clouds)

  void open();
  void step();
  void close();
};

struct ObjectFitter : public Module {
  struct sObjectFitter* s;

  ObjectFitter();

  void open();
  void step();
  void close();

  ACCESS(PointCloudSet, objectClusters)
  ACCESS(ObjectSet, objects)
};


struct ObjectFilter : public Module {
  struct sObjectFilter *s;
  ObjectFilter(const char *name) ;
  void open();
  void step();
  void close() {}

  ACCESS(ObjectSet, in_objects)
  ACCESS(ObjectBeliefSet, out_objects)
};

struct ObjectTransformator : public Module {
  ObjectTransformator(const char *name);
  void open();
  void step();
  void close() {}

  ACCESS(ObjectBeliefSet, kinect_objects)
  ACCESS(mlr::KinematicWorld, geoState)
  mlr::KinematicWorld geo;
};

#endif // MLR_PCL
#endif // _POINTCLOUD_H__
