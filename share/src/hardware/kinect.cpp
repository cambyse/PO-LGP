#ifdef PCL
#include "kinect.h"
#include <perception/pointcloud.h>
#include <libfreenect.hpp>
#include <pcl/point_cloud.h>

struct FreenectData : public Variable {
  FreenectData() : Variable("Freenect Data"),
                   buffer_depth(640*480),
                   buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), 
                   new_rgb_frame(false), 
                   new_depth_frame(false),
                   frame_id(0) {
  };
	
  std::vector<uint16_t> buffer_depth;
	std::vector<uint8_t> buffer_video;
	std::vector<uint16_t> gamma;
  
	bool new_rgb_frame;
	bool new_depth_frame;

  uint frame_id;
};


struct sKinectInterface : Freenect::FreenectDevice {
  sKinectInterface(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index) {
  };

  static const unsigned int image_width;
  static const unsigned int image_height;
  static const unsigned int rgb_array_size;
  unsigned char rgb_buffer[640*480*3];

  KinectInterface *p;
  FreenectData* data;
  
  void DepthCallback(void* _depth, uint32_t timestamp) {
    data->writeAccess(p);
		uint16_t* depth = static_cast<uint16_t*>(_depth);
		std::copy(depth, depth+image_width*image_height, data->buffer_depth.begin());
		data->new_depth_frame = true;
    data->deAccess(p);
  }

	void VideoCallback(void* _rgb, uint32_t timestamp) {
    data->writeAccess(p);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+getVideoBufferSize(), data->buffer_video.begin());
		data->new_rgb_frame = true;
    data->deAccess(p);
	}

	bool getRGB(std::vector<uint8_t> &buffer) {
    data->writeAccess(p);
		if (!data->new_rgb_frame) {
      data->deAccess(p);
			return false;
    }
		buffer.swap(data->buffer_video);
		data->new_rgb_frame = false;
    data->deAccess(p);
		return true;
	}

	bool getDepth(std::vector<uint16_t> &buffer) {
    data->writeAccess(p);
		if (!data->new_depth_frame) {
      data->deAccess(p);
			return false;
    }
		buffer.swap(data->buffer_depth);
		data->new_depth_frame = false;
    data->deAccess(p);
		return true;
	}
  template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
    convertToXYZRGBPointCloud (const std::vector<uint8_t>& rgb, const std::vector<uint16_t> &depth) const;
};

const unsigned int sKinectInterface::image_width = 640; //kinect resolution
const unsigned int sKinectInterface::image_height = 480; //kinect resolution
const unsigned int sKinectInterface::rgb_array_size = image_width*image_height*3; //kinect resolution

void KinectInterface::open() {
	s->startVideo();
	s->startDepth();

  // use hardware registration
  s->setDepthFormat(FREENECT_DEPTH_REGISTERED);
}

void KinectInterface::step() {
  std::vector<uint16_t> depth(640*480);
  std::vector<uint8_t> rgb(640*480*3);

  s->updateState();

  s->getDepth(depth);
  s->getRGB(rgb);

  pcl::PointCloud<PointT>::Ptr cloud = s->convertToXYZRGBPointCloud<PointT>(rgb, depth);

  data_3d->set_point_cloud(cloud, this);
}

void KinectInterface::close() {
	s->stopVideo();
	s->stopDepth();
}

Freenect::Freenect freenect;

KinectInterface::KinectInterface(const char* name) : Process(name) {
  s = &freenect.createDevice<sKinectInterface>(0);
  s->data = new FreenectData;
  s->p = this;
  biros().getVariable(data_3d, "KinectData3D", this);
}

KinectInterface::~KinectInterface() {
  freenect.deleteDevice(0);
  s = NULL;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

#endif
