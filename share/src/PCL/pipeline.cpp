#include "pipeline.h"

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <Perception/pclPlaneExtraction.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <Geo/mesh.h>
#include <Geo/qhull.h>

#include <Perception/percept.h>

void detectPlane(
    pcl::ModelCoefficients::Ptr& plane_coefficients,
    pcl::PointIndices::Ptr& inliers,
    Pcl::Ptr& hull,
    arr& meanColor,
    const Pcl::ConstPtr& input);
void filterPointsByIndex(Pcl::Ptr& output, const pcl::PointIndices::ConstPtr& inliers,  const Pcl::ConstPtr& input, bool positive);
void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl::ConstPtr& input);

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Pcl;

PclPipeline::PclPipeline(const char* input_name)
  : Thread("PclPipeline"),
    inputPcl(this, input_name, true),
    processedPcl(this, "pcl_processed"),
    percepts_input(this, "percepts_input"){
  threadOpen();
}

PclPipeline::~PclPipeline(){
  threadClose();
}

void PclPipeline::step(){
  Pcl::Ptr input(new Pcl(inputPcl.get()));

  Pcl::Ptr filtered(new Pcl);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.5, .7);
  pass.filter(*filtered);

  PerceptL percepts;

  if(filtered->size()){
    Pcl::Ptr remains(new Pcl);
    Pcl::Ptr hull(new Pcl);
    Pcl::Ptr cluster(new Pcl);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    arr color;
    mlr::Mesh plane;

    for (uint i=0; i<1; i++) {

      //detect plane
      detectPlane(plane_coefficients, inliers, hull, color, filtered);
      filterPointsByIndex(remains, inliers, filtered, false);

      //generate a Percept
      conv_PclCloud_ArrCloud(plane.V, plane.C, *hull);
      plane.makeTriangleFan();
      plane.C = color;
      arr normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
      percepts.append(new Plane(normal, mean(plane.V), plane, ""));

      //cluster the remains
      std::vector<pcl::PointIndices> cluster_indices;
      getClusters(cluster_indices, remains);

      //fit each cluster
      for(pcl::PointIndices& ci: cluster_indices){
        //select
        filterPointsByIndex(cluster, boost::make_shared<const pcl::PointIndices>(ci), remains, true);

        //detect plane
        detectPlane(plane_coefficients, inliers, hull, color, cluster);
        if(inliers->indices.size()<200) continue;

        //generate a Percept
        conv_PclCloud_ArrCloud(plane.V, plane.C, *hull);
        plane.makeTriangleFan();
        plane.C = color;
        arr normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
        percepts.append(new Plane(normal, mean(plane.V), plane, ""));
      }
    }
  }

#if 1
  percepts_input.set()->append(percepts);
#else
  percepts_input.writeAccess();
  listDelete(percepts_input());
  percepts_input() = percepts;
  percepts_input.deAccess();
#endif
}

void detectPlane(
    pcl::ModelCoefficients::Ptr& plane_coefficients,
    pcl::PointIndices::Ptr& inliers,
    Pcl::Ptr& hull,
    arr& meanColor,
    const Pcl::ConstPtr& input){

  pcl::SACSegmentation<PointT> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (input);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers, *plane_coefficients);
  //cerr << "Plane coefficients: " << *outCoefficients << endl;

  if(hull){
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (input);
    proj.setIndices(inliers);
    proj.setModelCoefficients (plane_coefficients);
    Pcl::Ptr projected(new Pcl);
    proj.filter(*projected);

    pcl::ConvexHull<PointT> chull;
    chull.setInputCloud (projected);
    chull.reconstruct (*hull);
  }

  if(&meanColor){
    meanColor.resize(3);
    for(uint i=0;i<inliers->indices.size();i++){
      int j=inliers->indices[i];
      meanColor.p[0] += input->points[j].r;
      meanColor.p[1] += input->points[j].g;
      meanColor.p[2] += input->points[j].b;
    }
    meanColor /= 255.;
    meanColor /= (double)inliers->indices.size();
  }
}


void filterPointsByIndex(Pcl::Ptr& output, const pcl::PointIndices::ConstPtr& inliers,  const Pcl::ConstPtr& input, bool positive) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (!positive);
  extract.filter(*output);
}

void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl::ConstPtr& input){
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(input);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input);
  ec.extract(cluster_indices);
}
