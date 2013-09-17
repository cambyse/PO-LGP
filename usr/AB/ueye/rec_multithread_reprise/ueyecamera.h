#pragma once

#include <ueye.h>
#include <QObject>
#include <QMutex>
#include <QString>

#include "recworker.h"

class UEyeCamera: public QObject {
  Q_OBJECT

  public:
    static QMutex msgMutex;

    UEyeCamera(int cid, int w, int h, int fps);

    int getWidth();
    int getHeight();
    int getFPS();
    MT::String getName();

    void camInit();
    void open();
    void close();
    void camExit();
    void grab();
    void getImage(char *p);

    void startRec();
    void stopRec();

    void quit();

    static int getNumCameras();

  private:
    int nrecframes;

    HIDS camID;
    INT camStatus;
    SENSORINFO camInfo;

    int width, height, fps;
    MT::String name;

    MT::String m;

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

    QMutex imgMutex, recMutex, quitMutex;

    QThread *recthread;
    RecWorker *recworker;
    bool recflag;
    int curr_frame, nskipped_frames;

    static bool query_status(HIDS camID, const char *method, INT *status);
    static void getNowString(MT::String &str);

    INT getImageID(char *buff);

    INT CaptureVideo_wrapper(INT wait);
    void ExitCamera_wrapper();
    void WaitForNextImage_wrapper(char **p, INT *pID);

    void waitUntilExit();

    void msg(const char *m);
    void msg(const MT::String &m);
    void err();

  private slots:
    void process();

  signals:
    void started();
    void finished();
};

