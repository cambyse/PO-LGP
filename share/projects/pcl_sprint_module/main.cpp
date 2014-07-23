#include "system.h"
#include "methods.h"
#include "object_detector.h"
#include <pcl/visualization/pcl_visualizer.h>

#include "plane.h"
#include "object.h"
#include "dataStructures.h"
#include "ros.h"
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

  Ros_publishPrimitives rospub;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addPointCloud<PointT>(cloud, "cloud");
  //viewer.addCoordinateSystem(1.0);
  //  CloudView *cv = new CloudView(S.pcl_cloud.get());
  //  primitives.P.append(cv);

  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;
    S.kinect_points.var->waitForNextRevision();

    cloud = S.pcl_cloud.get();

    if(cloud){
      viewer.updatePointCloud(cloud, "cloud");

      // extract background points
      pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
      passthroughFilter(cloud,cloud_filtered,0.,2.);
      //viewer.removeAllPointClouds();
      //viewer.addPointCloud(cloud_filtered);
      //viewer.spin();


      // detect planes and remove them from pointcloud
      uint numPlanes = 1;
      std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients;
      std::vector<pcl::PointIndices::Ptr> planeInliers;
      pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);
      extractPlanes(cloud_filtered,cloud_extracted,planeCoefficients,planeInliers,numPlanes);

      //viewer.removeAllPointClouds();
      //viewer.addPointCloud(cloud_extracted);
      //viewer.spin();

      //extract and display some biggest clusters
      std::vector<pcl::PointCloud<PointT>::Ptr> list_extracted_cloud;
      int numCluster = 10;
      clusterObject(cloud_extracted, numCluster, list_extracted_cloud);
      /*/ display the extracted clusters
      viewer.removeAllPointClouds();
      cout<< "list_extracted_cloud "<<list_extracted_cloud.size() <<endl;

      for (int num=0;num < list_extracted_cloud.size();num++)
      {
          std::string s = std::to_string(num);
          viewer.addPointCloud(list_extracted_cloud[num],s);
      }
      viewer.spin();
      /*/

      std::vector<std::pair<pcl::ModelCoefficients::Ptr,int>> list_primitives;
      extractPrimitives(list_extracted_cloud, list_primitives);

      cout<< "list_primitives "<<list_primitives.size() <<endl;

      //display the fitted models
/*/      viewer.removeAllPointClouds();

      for (int num=0;num < list_primitives.size();num++)
      {
          std::pair<pcl::ModelCoefficients::Ptr,int> temp = list_primitives[num];
          std::string s = std::to_string(num);
          if(temp.second == 0)
              viewer.addSphere(*temp.first,s);
          else if(temp.second == 1)
              viewer.addCylinder(*temp.first,s);
      }

      /*/
      //viewer.spin();


      //-- publish to ros
      rospub.publish(list_primitives);

/*/
      // compute normals
      pcl::PointCloud<pcl::Normal>::Ptr normal_extracted (new pcl::PointCloud<pcl::Normal>);
      normalEstimator(cloud_extracted,normal_extracted,50);

      // detect sphere
      pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
      sphereDetector(cloud_extracted,normal_extracted,coefficients_sphere,inliers_sphere,0.05,0.12);
      viewer.addSphere(*coefficients_sphere,"sphere");
      viewer.spin();

      // detect cylinder
      pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
      cylinderDetector(cloud_extracted,normal_extracted,coefficients_cylinder,inliers_cylinder,0.01,0.05);
      viewer.addCylinder(*coefficients_cylinder,"cylinder");
      viewer.spin();
/*/
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
