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
                 const Pcl::ConstPtr& input);
void projectOnPlane(
    Pcl::Ptr& hull,
    arr& meanPoint,
    arr& meanColor,
    const pcl::ModelCoefficients::ConstPtr& plane_coefficients,
    const Pcl::ConstPtr& input,
    const pcl::PointIndices::ConstPtr& inputSelect);
double getMaxZ(const Pcl::ConstPtr& input,
               const pcl::PointIndices::ConstPtr& inputSelect);
void filterPointsByIndex(Pcl::Ptr& output, const pcl::PointIndices::ConstPtr& inliers,  const Pcl::ConstPtr& input, bool positive);
void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl::ConstPtr& input, uint minSize);

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
  PerceptL percepts = PclScript_Z_plane_cluster_planes_boxes(new Pcl(inputPcl.get()), true);
  percepts_input.set()->append(percepts);
}

PerceptL PclScript_Z_plane_cluster_planes_boxes(const Pcl* newInput, bool verbosePercepts){
  Pcl::ConstPtr input(newInput);

  PerceptL percepts; //all percepts that are generated as output of the pipeline

  if(!input->size()) return percepts;

  //-- xz-filtering
  Pcl::Ptr filtered(new Pcl);
  boost::shared_ptr<std::vector<int> > z_filtered(new std::vector<int>);
  boost::shared_ptr<std::vector<int> > xz_filtered(new std::vector<int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.5, .7);
  pass.filter (*z_filtered);
  pass.setIndices (z_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (.4, 1.);
#if 0
  pass.filter(*xz_filtered);

  //-- voxel grid filter -> smaller resolution (small objects not recognized anymore!)
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (input);
  sor.setIndices(xz_filtered);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*filtered);
#else
  pass.filter(*filtered);
#endif

  //-- 1st percept: the filtered cloud itself
  mlr::Mesh mesh;
  byteA meshRgb;
  conv_PclCloud_ArrCloud(mesh.V, meshRgb, *filtered);
  copy(mesh.C, meshRgb);   mesh.C /= 255.;
//  if(verbosePercepts) percepts.append(new PercMesh(mesh));

  if(filtered->size()<100) return percepts;

  //-- detect main plane
  Pcl::Ptr hull(new Pcl);
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  arr meanPts, meanCol;
  detectPlane(plane_coefficients, inliers, filtered);
  projectOnPlane(hull, meanPts, meanCol, plane_coefficients, filtered, inliers);
  double planeHeight = meanPts(2);

  //-- 2nd percept: the main plane
  arr normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
  mlr::Transformation T;
  T.pos.set(meanPts);
  T.rot.setDiff(Vector_z, mlr::Vector(normal));
  conv_PclCloud_ArrCloud(mesh.V, NoByteA, *hull);
  mesh.makeLineStrip();
  mesh.C = meanCol;
  mesh.transform(-T);
  if(verbosePercepts) percepts.append(new PercPlane(T, mesh));

  //-- project hull in 2D and apply OpenCV rotated box fitting
  floatA X;
  copy(X, mesh.V);
  X.delColumns(2);
  cv::RotatedRect rect = cv::minAreaRect( conv_Arr2CvRef(X) );

  //-- 3rd percept: the plane's box -- with fixed thickness
  double thick=.04;
  T.addRelativeTranslation(rect.center.x, rect.center.y, -.5*thick);
  T.addRelativeRotationDeg(rect.angle, 0.,0.,1.);
//  percepts.append(new PercBox(T, ARR(rect.size.width, rect.size.height, thick), meanCol));

  if(true){
    //-- cluster the remains
    Pcl::Ptr remains(new Pcl);
    filterPointsByIndex(remains, inliers, filtered, false);
    std::vector<pcl::PointIndices> cluster_indices;
    getClusters(cluster_indices, remains, 50);

    //-- fit each cluster with a plane
    for(const pcl::PointIndices& ci: cluster_indices){
      pcl::PointIndices::ConstPtr clusterIndices = boost::make_shared<const pcl::PointIndices>(ci);
      //select
      Pcl::Ptr cluster(new Pcl);
      filterPointsByIndex(cluster, clusterIndices, remains, true);
      cout <<"cluster size =" <<cluster->size() <<endl;
      double maxZ = getMaxZ(remains, clusterIndices);
//      cout <<maxZ <<endl;
//      pass.setInputCloud(remains);
//      pass.setIndices(clusterIndices);
//      pass.setFilterFieldName ("z");
//      pass.setFilterLimits (maxZ-.13, maxZ+.03);
//      Pcl::Ptr cluster_z(new Pcl);
//      pass.filter (*cluster_z);
//      filterPointsByIndex(cluster, boost::make_shared<const pcl::PointIndices>(ci), remains, true);
//      if(cluster_z->size()<50) continue;

      //-- 1st percept: the cluster points themselves
      conv_PclCloud_ArrCloud(mesh.V, meshRgb, *cluster);
      copy(mesh.C, meshRgb);   mesh.C /= 255.;
      mesh.T.clear();
      if(verbosePercepts) percepts.append(new PercMesh(mesh));

      //detect plane
//      detectPlane(plane_coefficients, inliers, hull, meanPts, meanCol, cluster);
//      if(inliers->indices.size()<50) continue;
//      projectOnPlane(hull, meanPts, meanCol, plane_coefficients, cluster_z, pcl::PointIndices::ConstPtr());
//      projectOnPlane(hull, meanPts, meanCol, plane_coefficients, cluster, pcl::PointIndices::ConstPtr());
      projectOnPlane(hull, meanPts, meanCol, plane_coefficients, remains, clusterIndices);


      //-- 2nd percept: the hull main plane
      normal = { plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2] };
      T.pos.set(meanPts);
      T.rot.setDiff(Vector_z, mlr::Vector(normal));
      conv_PclCloud_ArrCloud(mesh.V, NoByteA, *hull);
      mesh.makeLineStrip();
      mesh.C = meanCol;
      mesh.transform(-T);
      T.addRelativeTranslation(0,0,maxZ-planeHeight);
      if(verbosePercepts) percepts.append(new PercPlane(T, mesh));

      //-- project hull in 2D and apply OpenCV rotated box fitting
      copy(X, mesh.V);
      X.delColumns(2);
      cv::RotatedRect rect = cv::minAreaRect( conv_Arr2CvRef(X) );
      if(rect.size.height < .85*rect.size.width){
        rect.angle += 90.;
        double x = rect.size.height;
        rect.size.height = rect.size.width;
        rect.size.width = x;
      }

      //-- 3rd percept: the plane's box
      double thick=.04;
      T.addRelativeTranslation(rect.center.x, rect.center.y, -1.5*thick); //WHY???
      T.addRelativeRotationDeg(rect.angle, 0.,0.,1.);
      percepts.append(new PercBox(T, ARR(rect.size.width, rect.size.height, thick), meanCol));
    }
  }

  return percepts;
}

void detectPlane(
    pcl::ModelCoefficients::Ptr& plane_coefficients,
    pcl::PointIndices::Ptr& inliers,
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
}

void projectOnPlane(
    Pcl::Ptr& hull,
    arr& meanPoint,
    arr& meanColor,
    const pcl::ModelCoefficients::ConstPtr& plane_coefficients,
    const Pcl::ConstPtr& input,
    const pcl::PointIndices::ConstPtr& inputSelect ){

  if(hull){
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (input);
    if(inputSelect) proj.setIndices(inputSelect);
    proj.setModelCoefficients (plane_coefficients);
    Pcl::Ptr projected(new Pcl);
    proj.filter(*projected);

    pcl::ConvexHull<PointT> chull;
    chull.setInputCloud (projected);
    chull.reconstruct (*hull);
  }

  if(&meanPoint){
    meanPoint = zeros(3);
    if(inputSelect){
      for(uint i=0;i<inputSelect->indices.size();i++){
        int j=inputSelect->indices[i];
        meanPoint.p[0] += input->points[j].x;
        meanPoint.p[1] += input->points[j].y;
        meanPoint.p[2] += input->points[j].z;
      }
      meanPoint /= (double)inputSelect->indices.size();
    }else{
      for(const PointT& p:input->points){
        meanPoint.p[0] += p.x;
        meanPoint.p[1] += p.y;
        meanPoint.p[2] += p.z;
      }
      meanPoint /= (double)input->points.size();
    }
  }

  if(&meanColor){
    meanColor = zeros(3);
    if(inputSelect){
      for(uint i=0;i<inputSelect->indices.size();i++){
        int j=inputSelect->indices[i];
        meanColor.p[0] += input->points[j].r;
        meanColor.p[1] += input->points[j].g;
        meanColor.p[2] += input->points[j].b;
      }
      meanColor /= 255.;
      meanColor /= (double)inputSelect->indices.size();
    }else{
      for(const PointT& p:input->points){
        meanColor.p[0] += p.r;
        meanColor.p[1] += p.g;
        meanColor.p[2] += p.b;
      }
      meanColor /= 255.;
      meanColor /= (double)input->points.size();
    }
  }
}

double getMaxZ(const Pcl::ConstPtr& input,
               const pcl::PointIndices::ConstPtr& inputSelect){
  double z=-1e10;
  if(inputSelect){
    for(uint i=0;i<inputSelect->indices.size();i++){
      const PointT& p = input->points[inputSelect->indices[i]];
      if(p.z>z) z=p.z;
    }
  }else{
    for(const PointT& p:input->points) if(p.z>z) z=p.z;
  }
  return z;
}


void filterPointsByIndex(Pcl::Ptr& output, const pcl::PointIndices::ConstPtr& inliers,  const Pcl::ConstPtr& input, bool positive) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (!positive);
  extract.filter(*output);
}

void getClusters(std::vector<pcl::PointIndices>& cluster_indices, const Pcl::ConstPtr& input, uint minSize){
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(input);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input);
  ec.extract(cluster_indices);
}

