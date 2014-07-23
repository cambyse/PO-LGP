#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;

void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<PointT>::Ptr outCloud, uint numPlanes);

void passthroughFilter(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<PointT>::Ptr outCloud,double minLimit, double maxLimit);

void normalEstimator(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr outCloud,int knn);

void planeDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane);

void substractPlane(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointIndices::Ptr inInliersPlane, pcl::PointCloud<PointT>::Ptr outCloud);

