#ifndef _KINECT_H_
#define _KINECT_H_

#include <Core/module.h>

#define Kinect_image_width 640
#define Kinect_image_height 480

struct KinectPoller : Module {
  struct sKinectInterface *s;

  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)

  KinectPoller();
  virtual ~KinectPoller();

  void open();
  void step();
  void close();
};

struct Kinect2PointCloud: Module {
  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)

  ACCESS(arr, kinect_points);
  ACCESS(arr, kinect_pointColors);

  arr pts,cols;
  floatA depth;
  byteA rgb; //helpers

  Kinect2PointCloud():Module("Kinect2PointCloud"){};
  virtual ~Kinect2PointCloud(){};

  void open(){};
  void step();
  void close(){};
};

namespace MLR {
	// pack 16bit depth image into 3 8-bit channels
	void pack_kindepth2rgb(const MT::Array<uint16_t>& depth, byteA& buffer);


	// these methods are for synchronous grabbing. they can *not* be used
	// in conjunction with the modules!#

	// get depth image. will make the array a reference to an internal buffer, which is only valid
	// until the next call of this method!
	void kinect_sync_get_depth(MT::Array<uint16_t>& depth, double& timestamp);
	// get video image as reference (see above)
	void kinect_sync_get_video(byteA& video, double& timestamp);
}

BEGIN_MODULE(KinectDepthPacking)
    ACCESS(MT::Array<uint16_t>, kinect_depth);
    ACCESS(byteA, kinect_depthRgb);
END_MODULE()

#endif
