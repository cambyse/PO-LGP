#pragma once

#include <Core/thread.h>

struct ImageViewer : Thread {
  struct sImageViewer *s;
  Access_typed<byteA> img;
  bool flipImage = false;
  ImageViewer(const char* img_name="rgb") : Thread(STRING("ImageViewer_"<<img_name), -1), img(this, img_name, true){}
  ~ImageViewer(){}
  void open();
  void step();
  void close();
};

struct PointCloudViewer : Thread {
  struct sPointCloudViewer *s;
  Access_typed<arr> pts;
  Access_typed<arr> cols;
  PointCloudViewer(const char* pts_name="kinect_points", const char* cols_name="kinect_pointColors")
    : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<cols_name), .1),
      pts(this, pts_name),
      cols(this, cols_name){}
  void open();
  void step();
  void close();
};
