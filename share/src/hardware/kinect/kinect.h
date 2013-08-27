#ifndef _KINECT_H_
#define _KINECT_H_

#include <Core/module.h>

struct KinectPoller : Module {
  struct sKinectInterface *s;

  ACCESS(byteA, kinect_rgb)
  ACCESS(floatA, kinect_depth)

  KinectPoller();
  virtual ~KinectPoller();

  void open();
  void step();
  void close();
};

struct Kinect2PointCloud: Module {
  ACCESS(byteA, kinect_rgb)
  ACCESS(floatA, kinect_depth)

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


#include <libfreenect.hpp>


#endif
