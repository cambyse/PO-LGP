#ifndef ICP_H_
#define ICP_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ICP {
public:
    ICP() = default;
    virtual ~ICP() = default;
    static void apply(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out);
};

#endif /* ICP_H_ */
