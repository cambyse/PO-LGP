#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QtCore>
#include <QImage>
#include "abstractcamera.h"

class CameraThread : public QThread
{
  Q_OBJECT

public:
  CameraThread(AbstractCamera *camera, bool recordData, QString outputPath);

  ~CameraThread();

  void stop();

  // called from other threads, will try to update the internal image buffer, unless the grabbing stage has locked it (in that case the current image in the buffer is returned)
  byteA getImage();

protected:
  void run();

private:

  //double getTime();
  bool getTime(double *time);

  volatile bool stopped;
  QMutex buffMutex;

  const char *path;

  bool record;

  AbstractCamera *camera;

  uchar *grabberBuff, *imageBuf;
};

#endif // CAMERATHREAD_H