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

typedef pcl::PointXYZRGB PointT;

void TEST(KinectModules) {
  struct MySystem:System{
    ACCESS(byteA, kinect_rgb)
    ACCESS(uint16A, kinect_depth)
    ACCESS(arr, kinect_points)
    ACCESS(arr, kinect_pointColors)

    MySystem(){
      addModule<KinectPoller>(NULL, Module_Thread::loopWithBeat, .1); //this is callback driven...
//      addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
      addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
      addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
      addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
      connect();
    }
  } S;

//  cout <<S <<endl;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  //viewer->setBackgroundColor (255, 255, 255);
  //viewer->addCoordinateSystem (.1);
  //viewer->initCameraParameters ();
  viewer.addPointCloud<PointT>(cloud, "cloud");


  engine().enableAccessLog();
  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;

    S.kinect_points.var->waitForNextRevision();

    S.kinect_points.readAccess();
    S.kinect_pointColors.readAccess();
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
    S.kinect_pointColors.deAccess();
    S.kinect_points.deAccess();

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

