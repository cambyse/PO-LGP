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
    if(MT::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }else{
      addModule<KinectPoller>(NULL, Module_Thread::loopWithBeat, .1); //this is callback driven...
    }

    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
//      addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
    addModule<ArrCloud2PclCloud>(NULL, Module_Thread::listenFirst);
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
