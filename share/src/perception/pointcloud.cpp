#include "pointcloud.h"

#include <numeric>

#include <biros/biros.h>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

ObjectClusterer::ObjectClusterer() : Process("ObjectClusterer") {
  birosInfo.getVariable(data_3d, "KinectData3D", this, true);
  birosInfo.getVariable(point_clouds, "ObjectClusters", this, true);
}

struct sObjectFitterWorker {
  sObjectFitterWorker(ObjectFitterWorker *p) : p(p) {}
  ObjectFitterWorker *p;
 
  void createNewJob(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers){ 
    FittingJob outliers(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outliers);
    p->jobs->writeAccess(p);
    p->jobs->data.push(outliers);
    p->jobs->deAccess(p);
  }
  
  double confidenceForCylinder(pcl::PointIndices::Ptr &inliers, FittingResult& object, const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    // TODO: make parameters
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    double ndw = birosInfo.getParameter<double>("CylNormalDistanceWeight", p, 0.07);
    seg.setNormalDistanceWeight (ndw);
    seg.setMaxIterations (100);
    double dt = birosInfo.getParameter<double>("CylDistanceThreshold", p, 0.01);
    seg.setDistanceThreshold (dt);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (normals);

    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.size() < 500) {
      object.reset();   
      return 0;
    }
    else {
      object = coefficients_cylinder;
      inliers = inliers_cylinder;
      return 1./(1 + cloud->size() - inliers->indices.size());
    }
  }
  
  double confidenceForSphere(pcl::PointIndices::Ptr &inliers, FittingResult& object, const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    // Create the segmentation object for sphere segmentation and set all the parameters
    // TODO: make parameters
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    double ndw = birosInfo.getParameter<double>("SphereNormalDistanceWeight", p, 10);
    seg.setNormalDistanceWeight (ndw);
    seg.setMaxIterations (100);
    double dt = birosInfo.getParameter<double>("SphereDistanceThreshold", p, .0005);
    seg.setDistanceThreshold (dt);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (normals);

    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    seg.segment (*inliers_sphere, *coefficients_sphere);
    if (inliers_sphere->indices.size() < 500) {
      object.reset();   
      return 0;
    }
    else {
      object = coefficients_sphere;
      inliers = inliers_sphere;
     
      // if rest points are enough create new job
      //if (cloud->size() - inliers_sphere->indices.size() > 500) {
        //createNewJob(cloud, inliers_sphere);
      //}
      return 1./(1 + cloud->size() - inliers->indices.size());
    }
  }
};

void ObjectClusterer::open() {}

void ObjectClusterer::close() {}

void ObjectClusterer::step() {
  //get a copy of the kinect data
  pcl::PointCloud<PointT>::Ptr cloud(data_3d->get_point_cloud_copy(this));
  if(cloud->points.size() == 0) return;

  // filter all points too far away
  // TODO: filter also points too far left/right/up/down
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::PassThrough<PointT> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0,1.5);
  passthrough.filter(*cloud_filtered);
 
  // filter away the table. This is done by fitting a plane to all data and
  // remove all inliers. This assumes that there is one big plane, which
  // contains all other objects.
  pcl::IndicesPtr inliers(new std::vector<int>);
  pcl::SampleConsensusModelPlane<PointT>::Ptr planemodel 
    (new pcl::SampleConsensusModelPlane<PointT> (cloud_filtered));
  pcl::RandomSampleConsensus<PointT> ransac(planemodel);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();
  ransac.getInliers(*inliers);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered);

  if (cloud_filtered->points.size() == 0) return;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(500);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  PointCloudL _point_clouds;
  // append cluster to PointCloud list 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    _point_clouds.append(cloud_cluster);
  }

  point_clouds->set_point_clouds(_point_clouds, this);
}

void ObjectFitterIntegrator::restart() {
  objects->writeAccess(this);
  objects->objects.clear();
  objects->deAccess(this);
}

void ObjectFitterIntegrator::integrateResult(const FittingResult &result) {
  // add to ObjectSet  
  if (result.get() == 0) return;
  objects->writeAccess(this);
  objects->objects.append(result);
  objects->deAccess(this);
}

ObjectFitterWorker::ObjectFitterWorker() : Worker<FittingJob, FittingResult>("ObjectFitter (Worker)"), s(new sObjectFitterWorker(this)) {}

void ObjectFitterWorker::doWork(FittingResult &object, const FittingJob &cloud) {
  // Build kd-tree from cloud
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  
  // Estimate point normals
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_normals);

  FittingResult cyl_object;
  pcl::PointIndices::Ptr cyl_inliers;
  double cylinder_confidence = s->confidenceForCylinder(cyl_inliers, cyl_object, cloud, cloud_normals);
  FittingResult sphere_object;
  pcl::PointIndices::Ptr sphere_inliers;
  double sphere_confidence = s->confidenceForSphere(sphere_inliers, sphere_object, cloud, cloud_normals);

  JK_DEBUG(sphere_confidence);
  JK_DEBUG(cylinder_confidence);
 
  double threshold = 0.;

  pcl::PointIndices::Ptr inliers;
  if (sphere_confidence > threshold && sphere_confidence > cylinder_confidence) {
     object = sphere_object; 
     inliers = sphere_inliers;
  }
  else if (cylinder_confidence > threshold) {
    object = cyl_object;  
    inliers = cyl_inliers;
  }
  else {
    object.reset();  
    return;
  }
  //if rest points are enough create new job
  if (cloud->size() - inliers->indices.size() > 500) {
    s->createNewJob(cloud, inliers);
  }
}


