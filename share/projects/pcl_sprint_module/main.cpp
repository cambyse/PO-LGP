#include "system.h"
#include "methods.h"
#include <pcl/visualization/pcl_visualizer.h>

#include "plane.h"

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
//      viewer.updatePointCloud(cloud, "cloud");

      pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
      passthroughFilter(cloud,cloud_filtered,2.,3.5);

      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
      normalEstimator(cloud_filtered,normal_cloud,50);

      pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
      planeDetector(cloud_filtered,normal_cloud,coefficients_plane,inliers_plane);

      pcl::PointCloud<PointT>::Ptr cloud_substracted_plane(new pcl::PointCloud<PointT>);
      substractPlane(cloud_filtered,inliers_plane,cloud_substracted_plane);

      if (coefficients_plane->values.size()>0) {
        coefficients_plane->values[3] = coefficients_plane->values[3];
//        viewer.removeShape("plane");
        viewer.addPlane(*coefficients_plane,"plane");
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.removePointCloud("cloud_filtered");
        viewer.addPointCloud(cloud_substracted_plane,"cloud_substracted_plane");
        viewer.spin();
      }
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

