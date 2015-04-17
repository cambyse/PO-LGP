#ifndef PCLMOTIONFILTER_H_
#define PCLMOTIONFILTER_H_

#include <pcl/common/common_headers.h>

class PCLMotionFilter {
    //----typedefs/classes----//

    //----members----//
    double threshold;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_input_cloud;

    //----methods----//
public:
    PCLMotionFilter();
    virtual ~PCLMotionFilter() {}
    virtual void new_input(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud);
    virtual void get_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) const;
protected:
    virtual double metric(const pcl::PointXYZRGB & p1, const pcl::PointXYZRGB & p2) const;
};

#endif /* MOTIONFILTER_H_ */
