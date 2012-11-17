#ifndef _KINECT_H_
#define _KINECT_H_

#ifdef PCL 

#include <biros/biros.h>

class PointCloudVar;

class KinectInterface : public Process {
  private:
    class sKinectInterface *s;
  public:
    PointCloudVar *data_3d;
    
    void open();
    void step();
    void close();

    KinectInterface(const char *name);
    virtual ~KinectInterface();
};
#endif

#endif
