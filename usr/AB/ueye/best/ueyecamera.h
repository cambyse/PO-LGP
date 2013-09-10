#pragma once

#include <ueye.h>
#include <QObject>
#include <QMutex>
#include <QString>
#include <Core/util.h>

#include "videowriter.h"

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

    void startRec(MT::String fname);
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

    VideoWriter_x264 *vw;

    static bool query_status(HIDS camID, const char *method, INT *status);

    INT getImageID(char *buff);

    INT CaptureVideo_wrapper(INT wait);
    void ExitCamera_wrapper();

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

