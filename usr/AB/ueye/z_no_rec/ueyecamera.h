#ifndef _UEYE_CAMERA_H_
#define _UEYE_CAMERA_H_

#include <ueye.h>
#include <QObject>
#include <QMutex>
#include <QString>
#include <Core/util.h>

class UEyeCamera: public QObject {
  Q_OBJECT

  public:
    UEyeCamera(int cid, int w, int h, int fps);

    int getWidth();
    int getHeight();
    int getFPS();
    MT::String getName();

    void init();
    void open();
    void close();
    void grab();
    void getImage(char *p);

    void startRec(MT::String fname);
    void stopRec();

    void quit();

    static int getNumCameras();

  private:
    HIDS camID;
    INT camStatus;
    SENSORINFO camInfo;

    int width, height, fps;
    MT::String name;

    bool quit_flag;

    int camIndex;
    int frameIndex;

    char *image, *image_copy;
    INT imageBuffNum;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    int bpp;  // bits per pixel

    UINT pixelclock;
    double real_fps;
    double exposure;

    QMutex mutex;

    static bool query_status(HIDS camID, const char *method, INT *status);

    INT getImageID(char *buff);

  private slots:
    void process();

  signals:
    void started();
    void finished();
};

#endif // _UEYE_CAMERA_H_

