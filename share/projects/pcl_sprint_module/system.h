#pragma once

//#include <System/engine.h>
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
      new RosCom_Spinner();
      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
      new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
    }else{
#endif
      new KinectThread; //this is callback driven...
#ifdef MLR_ROS
    }
#endif

    new ImageViewer("kinect_rgb");
//      addModule<KinectDepthPacking>("KinectDepthPacking" /*,Module::listenFirst*/ );
//      new ImageViewer("kinect_depthRgb");
    new Kinect2PointCloud;
    new PointCloudViewer("kinect_points", "kinect_pointColors");
    addModule<ArrCloud2PclCloud>(NULL /*,Module::listenFirst*/ );
    //connect();
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
