#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;

class PlaneDetector {
private:
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

public:
  PlaneDetector(pcl::PointCloud<PointT>::Ptr cloud);
  ~PlaneDetector();
  void passthroughFilter(double limit = 1.5);
  void estimateNormals(int knn = 50);

  pcl::ModelCoefficients detectPlane();
};
