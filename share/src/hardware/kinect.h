#ifndef _KINECT_H_
#define _KINECT_H_

#ifdef PCL 

#include <biros/biros.h>
#include <perception/perception.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;

class PointCloudVar : public Variable {
  public:
    PointCloudVar(const char* name);
    FIELD(pcl::PointCloud<PointT>::ConstPtr, point_cloud);
    pcl::PointCloud<PointT>::Ptr get_point_cloud_copy(Process *p) { readAccess(p); pcl::PointCloud<PointT>::Ptr tmp = point_cloud->makeShared(); deAccess(p); return tmp; }
};

class KinectSettings : public Variable {
  public:
    KinectSettings() : Variable("KinectSettings") { reg_capture_3d(); reg_capture_rgb(); }
    FIELD(bool, capture_rgb);
    FIELD(bool, capture_3d);
};

class KinectInterface : public Process {
  private:
    class sKinectInterface *s;
  public:
    PointCloudVar *data_3d;
    Image *data_rgb;

    void open();
    void step();
    void close();

    KinectInterface(const char *name);
};
#endif

#endif
