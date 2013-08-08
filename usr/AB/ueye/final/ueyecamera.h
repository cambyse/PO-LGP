#ifndef _UEYE_CAMERA_H_
#define _UEYE_CAMERA_H_

#include <ueye.h>
#include <QObject>
#include <QMutex>
#include <QString>

class UEyeCamera: public QObject {
  Q_OBJECT

  public:
    UEyeCamera(int cid, int w, int h, int fps);

    int getWidth();
    int getHeight();
    int getFPS();
    QString getName();

    void init();
    void open();
    void close();
    void grab();
    void getImage(char *p);

    void quit();

    static int getNumCameras();

  private:
    HIDS camID;
    INT camStatus;
    SENSORINFO camInfo;

    int width, height, fps;
    QString name;

    bool quit_flag;

    int camIndex;
    int frameIndex;

    char *image;
    INT imageBuffNum;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    int bpp;  // bits per pixel

    UINT old_pixelclock;
    UINT pixelclock;
    double real_fps;
    double exposure;

    QMutex mutex;

    static bool query_status(HIDS camID, const char *method, INT *status);

    INT getImageID(char *buff);

  private slots:
    void process();

  signals:
    void finished();
};

#endif // _UEYE_CAMERA_H_

