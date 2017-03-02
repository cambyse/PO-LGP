#include "pipeline.h"

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <Perception/pclPlaneExtraction.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Geo/mesh.h>

void detectPlane(pcl::ModelCoefficients& plane_coefficients, pcl::PointIndices& inliers, const Pcl& input);
void filterPointsByIndex(Pcl& output, const pcl::PointIndices& inliers,  const Pcl& input, bool positive);
void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl& input);

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Pcl;

PclPipeline::PclPipeline(const char* input_name)
  : Thread("PclPipeline"),
    inputPcl(this, input_name, true),
    processedPcl(this, "pcl_processed"),
    visionDisplay(this, "visionDisplay"){
  threadOpen();
}

void PclPipeline::step(){
  Pcl input = inputPcl.get();

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(boost::make_shared<const Pcl>(input));
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.5, .7);

  Pcl filtered;
  pass.filter(filtered);

  Pcl remains;
  Pcl planePts;

  visionDisplay.writeAccess();
  visionDisplay().clear();

  floatA planeCoeffs;
  if(filtered.size()){
    for (uint i=0; i<1; i++) {
      pcl::ModelCoefficients plane_coefficients;
      pcl::PointIndices inliers;

      detectPlane(plane_coefficients, inliers, filtered);
      if(inliers.indices.size()<200) break;
      filterPointsByIndex(remains, inliers, filtered, false);
      filterPointsByIndex(planePts, inliers, filtered, true);

      planeCoeffs = conv_stdvec2arr(plane_coefficients.values);
      cout <<i <<' ' <<planeCoeffs <<endl;
//    outCoefficients.push_back(plane_coefficients);
//    outInliers.push_back(inliers);

      mlr::Mesh plane;
      conv_PclCloud_ArrCloud(plane.V, plane.C, planePts);
      plane.makeConvexHull();

      visionDisplay().append(plane);

      std::vector<pcl::PointIndices> cluster_indices;
      getClusters(cluster_indices, remains);
      cout <<"#clusters=" <<cluster_indices.size() <<endl;
      for(pcl::PointIndices& ci: cluster_indices){
        Pcl cluster;
        filterPointsByIndex(cluster, ci, remains, true);

        pcl::ModelCoefficients plane_coefficients;
        pcl::PointIndices inliers;
        detectPlane(plane_coefficients, inliers, cluster);
        if(inliers.indices.size()<200) continue;
        Pcl planePts;
        filterPointsByIndex(planePts, inliers, cluster, true);

        conv_PclCloud_ArrCloud(plane.V, plane.C, planePts);
        plane.makeConvexHull();
          visionDisplay().append(plane);
      }
    }
  }

  visionDisplay.deAccess();
  processedPcl.set() = planePts;
}

void detectPlane(pcl::ModelCoefficients& plane_coefficients, pcl::PointIndices& inliers, const Pcl& input){
  pcl::SACSegmentation<PointT> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (boost::make_shared<const Pcl>(input));
  // Obtain the plane inliers and coefficients
  seg.segment (inliers, plane_coefficients);
  //cerr << "Plane coefficients: " << *outCoefficients << endl;
}


void filterPointsByIndex(Pcl& output, const pcl::PointIndices& inliers,  const Pcl& input, bool positive) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (boost::make_shared<const Pcl>(input));
  extract.setIndices (boost::make_shared<const pcl::PointIndices>(inliers));
  extract.setNegative (!positive);
  extract.filter(output);
}

void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl& input){
  Pcl::ConstPtr inputPtr = boost::make_shared<const Pcl>(input);
  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud(inputPtr);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (boost::make_shared<pcl::search::KdTree<PointT> >(tree));
  ec.setInputCloud (inputPtr);
  ec.extract(cluster_indices);
}
