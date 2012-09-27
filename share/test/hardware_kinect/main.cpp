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
