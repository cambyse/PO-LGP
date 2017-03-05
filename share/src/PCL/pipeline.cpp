#include "pipeline.h"

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <Perception/pclPlaneExtraction.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <Perception/opencv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <Geo/mesh.h>
#include <Geo/qhull.h>

#include <Perception/percept.h>

void detectPlane(pcl::ModelCoefficients::Ptr& plane_coefficients,
                 pcl::PointIndices::Ptr& inliers,
                 Pcl::Ptr& hull,
                 arr& meanPoint,
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
  PerceptL percepts = PclScript_Z_plane_cluster_planes_boxes(new Pcl(inputPcl.get()));
  percepts_input.set()->append(percepts);
}

PerceptL PclScript_Z_plane_cluster_planes_boxes(const Pcl* newInput){
  Pcl::ConstPtr input(newInput);

  PerceptL percepts; //all percepts that are generated as output of the pipeline

  if(!input->size()) return percepts;

  //-- z-filtering
  Pcl::Ptr z_filtered(new Pcl);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.5, .7);
  pass.filter(*z_filtered);

  //-- voxel grid filter -> smaller resolution
  Pcl::Ptr filtered(new Pcl);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (z_filtered);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*filtered);

  //-- 1st percept: the filtered cloud itself
  mlr::Mesh mesh;
  byteA meshRgb;
  conv_PclCloud_ArrCloud(mesh.V, meshRgb, *filtered);
  copy(mesh.C, meshRgb);   mesh.C /= 255.;
  percepts.append(new PercMesh(mesh));

  //-- detect main plane
  Pcl::Ptr hull(new Pcl);
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  arr meanPts, meanCol;
  detectPlane(plane_coefficients, inliers, hull, meanPts, meanCol, filtered);

  //-- 2nd percept: the main plane
  arr normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
  mlr::Transformation T;
  T.pos.set(meanPts);
  T.rot.setDiff(Vector_z, mlr::Vector(normal));
  conv_PclCloud_ArrCloud(mesh.V, NoByteA, *hull);
//      mesh.makeTriangleFan();
  mesh.makeLineStrip();
  mesh.C = meanCol;
  mesh.transform(-T);
//  percepts.append(new Plane(normal, meanPts, mesh, ""));
  percepts.append(new Plane(T, mesh));

  //-- project hull in 2D and apply OpenCV rotated box fitting
  floatA X;
  copy(X, mesh.V);
  X.delColumns(2);
  cv::RotatedRect rect = cv::minAreaRect( conv_Arr2CvRef(X) );

  //-- 3rd percept: the plane's box -- with fixed thickness
  double thick=.04;
  T.addRelativeTranslation(rect.center.x, rect.center.y, -.5*thick);
  T.addRelativeRotationDeg(rect.angle, 0.,0.,1.);
  percepts.append(new PercBox(T, ARR(rect.size.width, rect.size.height, thick) ));

  if(true){
    //-- cluster the remains
    Pcl::Ptr remains(new Pcl);
    filterPointsByIndex(remains, inliers, filtered, false);
    std::vector<pcl::PointIndices> cluster_indices;
    getClusters(cluster_indices, remains);

    //-- fit each cluster with a plane
    for(pcl::PointIndices& ci: cluster_indices){
      //select
      Pcl::Ptr cluster(new Pcl);
      filterPointsByIndex(cluster, boost::make_shared<const pcl::PointIndices>(ci), remains, true);

      //detect plane
      detectPlane(plane_coefficients, inliers, hull, meanPts, meanCol, cluster);
      if(inliers->indices.size()<200) continue;

      //-- 2nd percept: the main plane
      normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
      T.pos.set(meanPts);
      T.rot.setDiff(Vector_z, mlr::Vector(normal));
      conv_PclCloud_ArrCloud(mesh.V, NoByteA, *hull);
      mesh.makeLineStrip();
      mesh.C = meanCol;
      mesh.transform(-T);
    //  percepts.append(new Plane(normal, meanPts, mesh, ""));
      percepts.append(new Plane(T, mesh));

      //-- project hull in 2D and apply OpenCV rotated box fitting
      copy(X, mesh.V);
      X.delColumns(2);
      cv::RotatedRect rect = cv::minAreaRect( conv_Arr2CvRef(X) );
      if(rect.size.height>rect.size.width){
        rect.angle += 90.;
        double x = rect.size.height;
        rect.size.height = rect.size.width;
        rect.size.width = x;
      }

      //-- 3rd percept: the plane's box
      double thick=.04;
      T.addRelativeTranslation(rect.center.x, rect.center.y, -.5*thick);
      T.addRelativeRotationDeg(rect.angle, 0.,0.,1.);
      percepts.append(new PercBox(T, ARR(rect.size.width, rect.size.height, thick) ));

//      //generate a Percept
//      conv_PclCloud_ArrCloud(mesh.V, mesh.C, *hull);
//      //        mesh.makeTriangleFan();
//      mesh.makeLineStrip();
//      mesh.C = meanCol;
//      arr normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
//      percepts.append(new Plane(normal, meanPts, mesh, ""));
    }
  }

  return percepts;
}

void detectPlane(
    pcl::ModelCoefficients::Ptr& plane_coefficients,
    pcl::PointIndices::Ptr& inliers,
    Pcl::Ptr& hull,
    arr& meanPoint,
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

  if(&meanPoint){
    meanPoint.resize(3);
    for(uint i=0;i<inliers->indices.size();i++){
      int j=inliers->indices[i];
      meanPoint.p[0] += input->points[j].x;
      meanPoint.p[1] += input->points[j].y;
      meanPoint.p[2] += input->points[j].z;
    }
    meanPoint /= (double)inliers->indices.size();
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
