#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_box.h>
#include <unistd.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <hardware/kinect.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char **argv) {
	KinectInterface k("Kinect"); 
	k.data_3d = new PointCloud("Kinect Data");
	k.threadOpen();
	k.threadLoop();

  while (true) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(k.data_3d->get_point_cloud(NULL), 0, 255, 0);
  viewer->addCoordinateSystem (.1);
  viewer->initCameraParameters ();

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  cloud = k.data_3d->get_point_cloud(NULL);

  pcl::IndicesPtr inliers(new std::vector<int>);

  pcl::SampleConsensusModelPlane<PointT>::Ptr planemodel (new pcl::SampleConsensusModelPlane<PointT> (cloud));
  pcl::RandomSampleConsensus<PointT> ransac(planemodel);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();
  ransac.getInliers(*inliers);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);

  // Write the planar inliers to disk
  extract.filter (*cloud);

  pcl::PassThrough<PointT> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0,1.5);

  passthrough.filter(*cloud);
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 2cm
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    tree->setInputCloud (cloud_cluster);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_cluster);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_cluster);
    seg.setInputNormals (cloud_normals);

    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    std::cout << "Num of inliers: " << inliers_cylinder->indices.size() << std::endl;
    

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    double r, g, b;
    pcl::visualization::getRandomColors(r, g, b);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_cluster, r*255, g*255 ,b*255);
    std::stringstream name, cyl;
    name << "cloud_" << j;
    cyl << "cyl_" << j;
    viewer->addPointCloud(cloud_cluster, blue, name.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, name.str());
    if ( inliers_cylinder->indices.size() / (double) cloud_cluster->points.size() > 0.6) {
      viewer->addCylinder(*coefficients_cylinder, cyl.str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, cyl.str());
    }


    j++;
  }

  
  
 
  while(!viewer->wasStopped()){
    //viewer->updatePointCloud(cloud, "cloud");
    viewer->spinOnce();
  }
  }
}
