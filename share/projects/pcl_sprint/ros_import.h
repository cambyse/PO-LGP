#ifndef ROS_IMPORT_H
#define ROS_IMPORT_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace MLR { namespace PCL {
typedef void (cb_pcl_rgba) (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

struct sFromROS;

class FromROS
{
private:
    sFromROS *s;
public:
    FromROS(const std::string& topic_name, cloud_cb* callback);
    ~FromROS();
};
}}

#endif // ROS_IMPORT_H
