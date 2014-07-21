#include "grab.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

struct sViewer {
	pcl::visualization::CloudViewer viewer;
	sViewer() : viewer("PCL OpenNI Viewer") {};

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }
    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&sViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }

      interface->stop ();
    }
};

    SimpleOpenNIViewer::SimpleOpenNIViewer () : s(new sViewer()) {}
    SimpleOpenNIViewer::~SimpleOpenNIViewer() { delete s; }
void SimpleOpenNIViewer::run() {
	s->run();
}
