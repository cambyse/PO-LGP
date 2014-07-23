#include "system.h"
#include "methods.h"
#include "object_detector.h"
#include <pcl/visualization/pcl_visualizer.h>

#include "plane.h"
#include "dataStructures.h"
#include <vector>

void TEST(KinectModules) {

//  DisplayPrimitives primitives;
//  OpenGL gl;
//  gl.camera = kinectCam;
//  gl.add(glStandardScene, NULL);
//  gl.add(glDrawPrimitives, &primitives);
//  primitives.P.append(new Plane(1,1,1,2.));
//  ors::Shape *s = new ors::Shape(primitives.G, NoBody)->type=ors::boxST;
//  gl.update();

  PCL_ModuleSystem S;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addPointCloud<PointT>(cloud, "cloud");

//  CloudView *cv = new CloudView(S.pcl_cloud.get());
//  primitives.P.append(cv);

  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;
    S.kinect_points.var->waitForNextRevision();

    cloud = S.pcl_cloud.get();

    if(cloud){
      viewer.updatePointCloud(cloud, "cloud");

//       pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
//       passthroughFilter(cloud,cloud_filtered,2.,3.5);

//       viewer.removeAllPointClouds();
//       viewer.addPointCloud(cloud_filtered);
       viewer.spin();

       std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients;
       std::vector<pcl::PointIndices::Ptr> planeInliers;
       pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
       extractPlanes(cloud,cloud_extracted,planeCoefficients,planeInliers,3);

//       pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//       pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//       pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
//       extractPlane(cloud,cloud_extracted,coefficients_plane,inliers_plane);

       viewer.removeAllPointClouds();
       viewer.addPointCloud(cloud_extracted);
       viewer.spin();

//       pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
//       normalEstimator(cloud_filtered,normal_cloud,50);

//       pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//       pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//       planeDetector(cloud_filtered,normal_cloud,coefficients_plane,inliers_plane);

//       pcl::PointCloud<PointT>::Ptr cloud_substracted_plane(new pcl::PointCloud<PointT>);
//       substractPlane(cloud_filtered,inliers_plane,cloud_substracted_plane);

//       if (coefficients_plane->values.size()>0) {
//         coefficients_plane->values[3] = coefficients_plane->values[3];
// //        viewer.removeShape("plane");
//         viewer.addPlane(*coefficients_plane,"plane");
//         viewer.addCoordinateSystem(1.0);
//         viewer.initCameraParameters();
//         viewer.removePointCloud("cloud_filtered");
//         viewer.addPointCloud(cloud_substracted_plane,"cloud_substracted_plane");
//         viewer.spin();
//       }
      viewer.spinOnce();

    }

//    cv->cloud = S.pcl_cloud.get();
//    gl.update();

  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  testKinectModules();

  return 0;
};
