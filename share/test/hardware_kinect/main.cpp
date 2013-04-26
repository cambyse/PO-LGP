#ifdef PCL

#include <hardware/kinect.h>
#include <perception/pointcloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
  PointCloudVar p("KinectData3D");
  KinectInterface k("Kinect");
  std::cout<<"Device initialized."<<std::endl;
  k.threadOpen();
  k.threadLoopWithBeat(0.3);

  pcl::visualization::PCLVisualizer viewer("Viewer");
  viewer.setBackgroundColor(1,1,1);
  viewer.addPointCloud(p.get_point_cloud(NULL), "cloud");

  while (true) {
    viewer.updatePointCloud(p.get_point_cloud(NULL), "cloud");
    viewer.spinOnce();
  }

}

#else

#include <iostream>

int main(int, char**) {
  std::cout << "This test needs a working pointcloud library. Abort." << std::endl; 
  std::cout << "Add PCL = 1 to the app section of your make-config as well as the lib section to activate correct linking against the PCL." << std::endl; 
}

#endif
