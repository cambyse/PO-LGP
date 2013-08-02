#ifndef _UEYE_CAMERA_H_
#define _UEYE_CAMERA_H_

#include <ueye.h>
#include "abstractcamera.h"

class UEyeCamera: public AbstractCamera {
  public:
    UEyeCamera(int camIndex, int width, int height, int fps);
    ~UEyeCamera();

    void setParams();
    void open();
    void close();
    void grab();
    bool retrieve(char *img);

    static int getNumCameras();

  private:
    int camIndex;
    int frameIndex;

    HIDS camID;
    INT camStatus;
    SENSORINFO camInfo;

    char *image;
    INT imageBuffNum;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    INT width;
    INT height;
    INT bpp;  // bits per pixel

    UINT old_pixelclock;
    UINT pixelclock;
    double fps;
    double real_fps;
    double exposure;

    static bool query_status(HIDS camID, const char *method, INT *status);

    INT getImageID(char *buff);
};

#endif // _UEYE_CAMERA_H_

