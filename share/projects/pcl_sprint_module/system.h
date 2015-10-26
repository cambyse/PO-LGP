#pragma once

#include <System/engine.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Gui/opengl.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <pr2/roscom.h>

#include "methods.h"

//===========================================================================

struct ArrCloud2PclCloud;

struct PCL_ModuleSystem:System{
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(pcl::PointCloud<PointT>::Ptr, pcl_cloud)

  PCL_ModuleSystem(){
#ifdef MLR_ROS
    if(mlr::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module::loopWithBeat, 1.);
    }else{
#endif
      addModule<KinectPoller>(NULL, Module::loopWithBeat, .1); //this is callback driven...
#ifdef MLR_ROS
    }
#endif

    addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
//      addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
//      addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
    addModule<ArrCloud2PclCloud>(NULL, Module::listenFirst);
    connect();
  }
};

//===========================================================================

// THIS IS AN EXAMPLE FOR HOW TO MAKE A MODULE FROM A ROUTINE
struct ArrCloud2PclCloud:Module{
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(pcl::PointCloud<PointT>::Ptr, pcl_cloud)
  virtual ~ArrCloud2PclCloud() {}
  virtual void open(){}
  virtual void step(){ conv_ArrCloud_PclCloud(pcl_cloud.set(),kinect_points.get(),kinect_pointColors.get()); }
  virtual void close(){}
};

inline void operator>>(istream&,pcl::PointCloud<PointT>::Ptr&){ NIY; }
