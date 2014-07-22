#include <pcl/point_types.h>
typedef pcl::PointXYZRGB PointT;
#include "../pcl_sprint/system.h"

#include "plane.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


void TEST(KinectModules) {
  PCL_ModuleSystem S;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  //viewer->setBackgroundColor (255, 255, 255);
  //viewer->addCoordinateSystem (.1);
  //viewer->initCameraParameters ();
  viewer.addPointCloud<PointT>(cloud, "cloud");

  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;

    S.kinect_points.var->waitForNextRevision();

#if 1
    S.kinect_points.readAccess();
    S.kinect_pointColors.readAccess();
    if(S.kinect_points().d0==640*480 && S.kinect_pointColors().d0==640*480){
      uint i=0;
      for(PointT& p:*cloud){
        p.x = S.kinect_points()(i,0);
        p.y = S.kinect_points()(i,1);
        p.z = S.kinect_points()(i,2);
        p.r = 255.*S.kinect_pointColors()(i,0);
        p.g = 255.*S.kinect_pointColors()(i,1);
        p.b = 255.*S.kinect_pointColors()(i,2);
        i++;
      }
    }
    S.kinect_pointColors.deAccess();
    S.kinect_points.deAccess();
#else
    cloud = S.pcl_cloud.get();
#endif

    viewer.updatePointCloud(cloud, "cloud");
    viewer.spinOnce();

  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  testKinectModules();

  return 0;
};

