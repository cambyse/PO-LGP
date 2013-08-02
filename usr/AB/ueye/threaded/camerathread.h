#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QtCore>
#include <QImage>
#include <Core/array.h>

#include "abstractcamera.h"

class CameraThread : public QThread {
  public:
    CameraThread(AbstractCamera *c, bool r, const char *p);
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

    AbstractCamera *camera;
    uchar *grabberBuff, *imageBuff;

    bool record;

    const char *path;
};

#endif // CAMERATHREAD_H
