#include <biros/biros.h>
#include <perception/perception.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>


class PointCloud : public Variable {
  public:
    PointCloud(const char* name) : Variable(name) {} ;
    FIELD(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, point_cloud);

};
class KinectSettings : public Variable {
  public:
    FIELD(bool, capture_rgb);
    FIELD(bool, capture_3d);
};
class KinectInterface : public Process {
  private:
    class sKinectInterface *s;
  public:
    PointCloud *data_3d;
    Image *data_rgb;

    void open();
    void step();
    void close();

    KinectInterface(const char *name);
};
