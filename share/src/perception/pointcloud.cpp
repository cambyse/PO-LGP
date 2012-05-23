#include "pointcloud.h"

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

//void ObjectFitter::open() { }

//void ObjectFitter::step() {
  //master->pause();
  //objects->set_objects(master->objects);
  //master->restart();
//}

//void ObjectFitter::close() { }


//int ObjectFitterMaster::hasNextJob() {
  //return point_clouds.get_point_clouds(this).N - jobs.size();
//}
//int ObjectFitterMaster::hasWorkingJob() {
  //return 0; 
//}
//FittingJob ObjectFitterMaster::createJob() {
  //jobs.push(point_clouds.get_point_clouds(this).N);
  
//}

//void ObjectFitterWorker::doWork(FittingResult &object, const FittingJob &cloud) {
  ////do pcl stuff  
  //pcl::NormalEstimation<PointT, pcl::Normal> ne;
  //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  //pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  //pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  //pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  //tree->setInputCloud (cloud);

  
  //// Estimate point normals
  //ne.setSearchMethod (tree);
  //ne.setInputCloud (cloud);
  //ne.setKSearch (50);
  //ne.compute (*cloud_normals);

  //// Create the segmentation object for cylinder segmentation and set all the parameters
  //seg.setOptimizeCoefficients (true);
  //seg.setModelType (pcl::SACMODEL_CYLINDER);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setNormalDistanceWeight (0.1);
  //seg.setMaxIterations (100);
  //seg.setDistanceThreshold (0.05);
  //seg.setRadiusLimits (0, 0.1);
  //seg.setInputCloud (cloud);
  //seg.setInputNormals (cloud_normals);

  //seg.segment (*inliers_cylinder, *coefficients_cylinder);

  //std::cout << "Num of inliers: " << inliers_cylinder->indices.size() << std::endl;

  //object = coefficients_cylinder;
//}
