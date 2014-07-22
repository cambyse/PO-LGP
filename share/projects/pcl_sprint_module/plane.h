
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PoinT2;


void passthroughFilter(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<PoinT2>::Ptr outCloud,double limit);

void normalEstimator(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr outCloud,int knn);

void planeDetector(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane);
