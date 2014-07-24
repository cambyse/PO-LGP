#include "object_detector.h"


bool sphereDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane,double min_radius, double max_radius)
{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_SPHERE);
//  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2800);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (inCloud);
  seg.setRadiusLimits(min_radius,max_radius);

  seg.setInputNormals (inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  //std::cerr << "Sphere coefficients: " << *outCoefficients << std::endl;
  //std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();
  if(inCloud->points.size()<=0)
      return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.9)
      return false;
  else
      return true;

}



bool cylinderDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane,double min_radius, double max_radius)
{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setNormalDistanceWeight (0.001);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2800);
  seg.setDistanceThreshold (0.001);
  seg.setInputCloud (inCloud);
  seg.setRadiusLimits(min_radius,max_radius);
  seg.setInputNormals (inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  //std::cerr << "Cylinder coefficients: " << *outCoefficients << std::endl;

  std::cerr << "inCloud->points.size()"<<inCloud->points.size() <<"  "<< outInliersPlane->indices.size();

  if(inCloud->points.size()<=0)
      return false;
  else if((double)outInliersPlane->indices.size()/(double)inCloud->points.size() < 0.15)
      return false;
  else{
      //computing the height of the cylinder.
      PointT min_pt, max_pt;
      pcl::getMinMax3D (*inCloud, min_pt, max_pt);
      double x,y,z; //direction of the fitted cylinder (is a normal vector);
      x = outCoefficients->values[3];
      y = outCoefficients->values[4];
      z = outCoefficients->values[5];

      double normal_length = (x*x + y*y + z*z);

      double product_min = min_pt.x*x + min_pt.y*y + min_pt.z*z;
      double product_max = max_pt.x*x + max_pt.y*y + max_pt.z*z;

      double proj_min_x = product_min*x/normal_length;
      double proj_min_y = product_min*y/normal_length;
      double proj_min_z = product_min*z/normal_length;

      double proj_max_x = product_max*x/normal_length;
      double proj_max_y = product_max*y/normal_length;
      double proj_max_z = product_max*z/normal_length;

      double height = sqrt((proj_min_x-proj_max_x)*(proj_min_x-proj_max_x) +
                          (proj_min_y-proj_max_y)*(proj_min_y-proj_max_y) +
                           (proj_min_z-proj_max_z)*(proj_min_z-proj_max_z) ) ;


     // multiply the height to the normal direction (very easily to be extracted later)
      //re-centering the base

     outCoefficients->values[0] = min_pt.x + (max_pt.x - min_pt.x)/2.0;
     outCoefficients->values[1] = min_pt.y + (max_pt.y - min_pt.y)/2.0;
     outCoefficients->values[2] = min_pt.z + (max_pt.z - min_pt.z)/2.0;

     outCoefficients->values[3] = height*outCoefficients->values[3];
     outCoefficients->values[4] = height*outCoefficients->values[4];
     outCoefficients->values[5] = height*outCoefficients->values[5];




      return true;
  }
}
