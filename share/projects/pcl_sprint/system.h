#include <System/engine.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Gui/opengl.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Core/module.h>

void operator>>(istream&,pcl::PointCloud<PointT>::Ptr&){ NIY; }

// THIS IS AN EXAMPLE FOR A PLAIN COMPUTATIONAL ROUTINE
void convertPointCloud2PCLCloud(pcl::PointCloud<PointT>::Ptr& pcl_cloud,
                         const arr& kinect_points,
                         const arr& kinect_pointColors);

// THIS IS AN EXAMPLE FOR HOW TO MAKE A MODULE FROM A ROUTINE
struct PointCloud2PCLCloud:Module{
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(pcl::PointCloud<PointT>::Ptr, pcl_cloud)
  virtual ~PointCloud2PCLCloud() {}
  virtual void open(){ pcl_cloud.set() = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(640,480)); }
  virtual void step(){ convertPointCloud2PCLCloud(pcl_cloud.set(),kinect_points.get(),kinect_pointColors.get()); }
  virtual void close(){}
};

struct PCL_ModuleSystem:System{
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(pcl::PointCloud<PointT>::Ptr, pcl_cloud)

  PCL_ModuleSystem(){
    addModule<KinectPoller>(NULL, Module_Thread::loopWithBeat, .1); //this is callback driven...
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
//      addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
    addModule<PointCloud2PCLCloud>(NULL, Module_Thread::listenFirst);
    connect();
  }
};

void convertPointCloud2PCLCloud(pcl::PointCloud<PointT>::Ptr& pcl_cloud,
                         const arr& kinect_points,
                         const arr& kinect_pointColors){
  if(kinect_points.d0<=640*480 && kinect_pointColors.d0<=640*480) return;
  uint i=0;
  for(PointT& p:*pcl_cloud){
    p.x = kinect_points(i,0);
    p.y = kinect_points(i,1);
    p.z = kinect_points(i,2);
    p.r = 255.*kinect_pointColors(i,0);
    p.g = 255.*kinect_pointColors(i,1);
    p.b = 255.*kinect_pointColors(i,2);
    i++;
  }
}
