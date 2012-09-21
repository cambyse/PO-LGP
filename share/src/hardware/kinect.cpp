#ifdef PCL
#include "kinect.h"
#include <pcl/io/openni_grabber.h>


struct sKinectInterface {
  KinectInterface *p;
  pcl::Grabber* interface;

  void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud) {
    p->data_3d->set_point_cloud(cloud, p);
  }
 
  sKinectInterface(KinectInterface *parent) : p(parent) {};
};

void KinectInterface::open() {
  s->interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
         boost::bind (&sKinectInterface::cloud_cb_, s, _1); 
                        //boost::bind (&sKinectInterface::cloud_cb, s, _1);

  s->interface->registerCallback (f);
  s->interface->start ();
}

void KinectInterface::step() {
}

void KinectInterface::close() {
  s->interface->stop();
}

KinectInterface::KinectInterface(const char* name) : Process(name), s(new sKinectInterface(this)) {
  birosInfo().getVariable(data_3d, "KinectData3D", this);
  birosInfo().getVariable(data_rgb, "KinectDataRGB", this);
}

PointCloudVar::PointCloudVar(const char *name) : Variable(name), point_cloud(new pcl::PointCloud<PointT>()) { 
  reg_point_cloud();
}
#endif
