#include "pipeline.h"

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <Perception/pclPlaneExtraction.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Geo/mesh.h>

void detectPlane(pcl::ModelCoefficients& plane_coefficients, pcl::PointIndices& inliers, const Pcl& input);
void filterPointsByIndex(Pcl& output, const pcl::PointIndices& inliers,  const Pcl& input, bool subtract);

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

  floatA planeCoeffs;
  if(filtered.size()){
    for (uint i=0; i<1; i++) {
      pcl::ModelCoefficients plane_coefficients;
      pcl::PointIndices inliers;

      detectPlane(plane_coefficients, inliers, filtered);
      if(inliers.indices.size()<200) break;
      filterPointsByIndex(remains, inliers, filtered, true);
      filterPointsByIndex(planePts, inliers, filtered, false);

      planeCoeffs = conv_stdvec2arr(plane_coefficients.values);
      cout <<i <<' ' <<planeCoeffs <<endl;
//    outCoefficients.push_back(plane_coefficients);
//    outInliers.push_back(inliers);

      mlr::Mesh plane;
      conv_PclCloud_ArrCloud(plane.V, plane.C, planePts);
      plane.makeConvexHull();

      visionDisplay.set() = {plane};

      filtered = remains;
    }
  }

  processedPcl.set() = planePts;
}

void detectPlane(pcl::ModelCoefficients& plane_coefficients, pcl::PointIndices& inliers, const Pcl& input){
  pcl::SACSegmentation<PointT> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (boost::make_shared<const Pcl>(input));
  // Obtain the plane inliers and coefficients
  seg.segment (inliers, plane_coefficients);
  //cerr << "Plane coefficients: " << *outCoefficients << endl;
}


void filterPointsByIndex(Pcl& output, const pcl::PointIndices& inliers,  const Pcl& input, bool subtract) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (boost::make_shared<const Pcl>(input));
  extract.setIndices (boost::make_shared<const pcl::PointIndices>(inliers));
  extract.setNegative (subtract);
  extract.filter(output);
}
