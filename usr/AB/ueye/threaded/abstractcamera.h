#ifndef ABSTRACTCAMERA_H
#define ABSTRACTCAMERA_H

#include <QtCore>
#include "opencv2/opencv.hpp"

using namespace cv;

class AbstractCamera
{

public:

  // open camera (can be issued from differen thread)
  virtual void open() = 0;

  // release camera buffers
  virtual void close() = 0;

  // grab frame (fast)
  virtual void grab() = 0;

  // decode frame
  virtual bool retrieve(Mat &frameMat) = 0;

  int getWidth() const { return _width; }
  int getHeight() const { return _height; }
  int getFPS() const { return _fps; }

  QString getName() const { return _cameraName; }

protected:

  int _width, _height, _fps;
  QString _cameraName;

};

#endif // ABSTRACTCAMERA_H
