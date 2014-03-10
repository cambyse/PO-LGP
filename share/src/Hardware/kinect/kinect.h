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
	void pack_kindepth2rgb(const MT::Array<uint16_t>& depth, byteA& buffer);
}

BEGIN_MODULE(KinectDepthPacking)
    ACCESS(MT::Array<uint16_t>, kinect_depth);
    ACCESS(byteA, kinect_depthRgb);
END_MODULE()

#endif
