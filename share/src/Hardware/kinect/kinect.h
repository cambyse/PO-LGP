#pragma once

#include <Core/thread.h>


struct KinectThread : Thread {
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
