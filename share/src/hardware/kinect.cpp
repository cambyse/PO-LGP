#include "kinect.h"



const unsigned int sKinectInterface::image_width = 640; //kinect resolution
const unsigned int sKinectInterface::image_height = 480; //kinect resolution


void KinectInterface::open() {
  s->startVideo();
  s->startDepth();

  // use hardware registration
  s->setDepthFormat(FREENECT_DEPTH_REGISTERED);
}

void KinectInterface::step() {
  MT::Array<uint16_t> depth(640*480);
  MT::Array<uint8_t> rgb(640*480*3);

  s->updateState();

  pointCloud->writeAccess(module);
  s->getDepth(pointCloud->depth);
  s->getRGB(pointCloud->rgb);
  pointCloud->deAccess(module);
}

void KinectInterface::close() {
  s->stopVideo();
  s->stopDepth();
}

Freenect::Freenect *freenect=NULL;

KinectInterface::KinectInterface(const char* name) : Process(name), s(NULL){
  freenect = new Freenect::Freenect;
  s = &(freenect->createDevice<sKinectInterface>(0));
  s->data = new FreenectData;
  s->p = this;
  pointCloud = biros().getVariable<PointCloudVar>("KinectData3D", module);
}

KinectInterface::~KinectInterface() {
  freenect->deleteDevice(0);
  s = NULL;
  delete freenect;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
sKinectInterface::convertToXYZRGBPointCloud (const std::vector<uint8_t>& rgb, const std::vector<uint16_t> &depth) const
{
  boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>);

  cloud->header.frame_id = data->frame_id++;
  cloud->height = image_height;
  cloud->width = image_width;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  register int centerX = (image_width >> 1);
  int centerY = (image_height >> 1);

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  int value_idx = 0;
  int point_idx = 0;
  for (int v = -centerY; v < centerY; ++v)
    {
      for (register int u = -centerX; u < centerX; ++u, ++value_idx, ++point_idx)
	{
	  PointT& pt = cloud->points[point_idx];
	  /// @todo Different values for these cases
	  // Check for invalid measurements

	  if (depth[value_idx] != 0 && depth[value_idx] != 2047)
	    {
	      pt.z = (float) depth[value_idx] * 0.001;
	      pt.x = static_cast<float> (u) * pt.z * constant;
	      pt.y = static_cast<float> (v) * pt.z * constant;

	      int color_idx = value_idx * 3;
	      pt.rgba = ((int)rgb[color_idx]) << 16 | ((int)rgb[color_idx+1]) << 8 | ((int)rgb[color_idx+2]);
	    }
	  else
	    {
	      pt.x = pt.y = pt.z = bad_point;
	    }
	}
    }

  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;
  return (cloud);
}
*/
