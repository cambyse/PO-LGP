#pragma once

#include <Core/thread.h>

struct ImageViewer : Thread {
  struct sImageViewer *s;
  Access_typed<byteA> img;
  bool flipImage = false;
  ImageViewer(const char* img_name="rgb");
  ~ImageViewer();
  void open();
  void step();
  void close();
};

struct PointCloudViewer : Thread {
  struct sPointCloudViewer *s;
  Access_typed<arr> pts;
  Access_typed<arr> cols;
  PointCloudViewer(const char* pts_name="kinect_points", const char* cols_name="kinect_pointColors");
  ~PointCloudViewer();
  void open();
  void step();
  void close();
};
