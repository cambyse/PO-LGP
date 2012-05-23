#include <perception/pointcloud.h>

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {

  // Variables
  PointCloudVar kinectData3d("KinectData3D");
  PointCloudSet objectClusters("ObjectClusters");

  // Processes
  KinectInterface kinect("Kinect");
  ObjectClusterer clusterer;


  kinect.threadLoop();
  clusterer.threadLoop();

  // TODO: put into Viewer<PointCloudVar>
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (.1);
  viewer->initCameraParameters ();
  //viewer->addPointCloud(kinectData3d.get_point_cloud_copy(NULL), "cloud");
  while(objectClusters.get_point_clouds(NULL).N == 0) MT::wait(0.1);
  
  viewer->addPointCloud(objectClusters.get_point_clouds(NULL)(0)->makeShared(), "cluster");

  for(int i = 0; i < 1000; ++i) {
    MT::wait(.1);
    //viewer->updatePointCloud(kinectData3d.get_point_cloud_copy(NULL), "cloud");
    viewer->updatePointCloud(objectClusters.get_point_clouds(NULL)(0)->makeShared(), "cluster");
    viewer->spinOnce();
  }

  kinect.threadStop();

}
