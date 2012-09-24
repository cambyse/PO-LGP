#ifndef _KINECT_H_
#define _KINECT_H_

#ifdef PCL 

#include <biros/biros.h>

class KinectSettings : public Variable {
  public:
    KinectSettings() : Variable("KinectSettings") { reg_capture_3d(); reg_capture_rgb(); }
    FIELD(bool, capture_rgb);
    FIELD(bool, capture_3d);
};

class PointCloudVar;
//class Image;

class KinectInterface : public Process {
  private:
    class sKinectInterface *s;
  public:
    PointCloudVar *data_3d;
    //Image *data_rgb;

    void open();
    void step();
    void close();

    KinectInterface(const char *name);
};
#endif

#endif
