#include "../pcl_sprint_module/system.h"
#include "../pcl_sprint_module/methods.h"
#include <pcl/visualization/pcl_visualizer.h>

void TEST(KinectModules) {
  PCL_ModuleSystem S;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addPointCloud<PointT>(cloud, "cloud");

  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;
    S.kinect_points.var->waitForNextRevision();

    cloud = S.pcl_cloud.get();

    if(cloud){
      viewer.updatePointCloud(cloud, "cloud");
      viewer.spinOnce();
    }
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  testKinectModules();

  return 0;
};
