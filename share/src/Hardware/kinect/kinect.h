#pragma once

#include <Core/module.h>


struct KinectThread:Module{
  struct sKinectThread *s;

  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)

  int verbose;

  KinectThread();
  ~KinectThread();

  void glViewKeys(char key);

  void open();
  void step();
  void close();
};
