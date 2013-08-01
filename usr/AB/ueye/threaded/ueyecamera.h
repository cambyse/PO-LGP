#include<ueye.h>
#include"VideoWriter_x264.h"

class UEyeCamera: public AbstractCamera {
  public:
    UEyeCamera(int camIndex, int width, int height, int fps, const char *fname);
    ~UEyeCamera();

    void init();
    void open();
    void setParams();
    void close();
    void grab();
    bool retrieve();

    static unsigned int getNumCameras();

  private:
    int camIndex;
    int frameIndex;

    HIDS camID;
    INT camStatus;
    SENSORINFO camInfo;
    char *image;

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

    const char *fname;
    VideoWriter_x264 *vw;

    bool quit, play, rec;

    void query_status(HIDS camID, const char *method, INT *status);
}
