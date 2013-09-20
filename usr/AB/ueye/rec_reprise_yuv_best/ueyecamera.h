#pragma once

#include <ueye.h>
#include <QObject>
#include <QMutex>
#include <QString>
#include <Core/thread.h>

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
    bool getErrFlag();

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

    bool quit_flag, err_flag;

    int camIndex;
    int frameIndex;

    char *image, *image_copy;
    INT imageBuffNum;
    UEYEIMAGEINFO imgInfo;
    UEYE_CAPTURE_STATUS_INFO captInfo;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    // bits per pixel, bytes per pixel, bytes per image
    int bpp, bypp, bypimg;  // bits per pixel, bytes

    UINT pixelclock;
    double real_fps, live_fps;
    double exposure;

    QMutex imgMutex, recMutex, quitMutex;

    QThread *recthread;
    RecWorker *recworker;
    bool recflag;
    int curr_frame, nskipped_frames;

    void InitCamera_wr();
    void SetColorMode_wr(INT Mode);
    void SetColorConverter_wr(INT ColorMode, INT ConvertMode);
    void SetDisplayMode_wr(INT Mode);
    void SetExternalTrigger_wr(INT nTriggerMode);
    void GetSensorInfo_wr();

    void AllocImageMem_wr(char **buff, INT *buffID);
    void FreeImageMem_wr(char *buff, INT buffID);
    void ClearSequence_wr();
    void AddToSequence_wr(char *buff, INT buffID);

    void PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam);
    void SetFrameRate_wr();
    void Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam);

    void CaptureVideo_wr(INT wait);

    void InitImageQueue_wr();
    void ExitImageQueue_wr();

    void WaitForNextImage_wr();
    void CaptureStatus_wr(UINT nCommand);
    void GetImageInfo_wr();
    void UnlockSeqBuf_wr(INT buffID, char *buff);
    void GetFramesPerSecond_wr();

    void StopLiveVideo_wr(INT wait);
    void ExitCamera_wr();

    void GetError_wr();

    void handleCamStatus();
    void handleCaptStatus();
    void msg(const char *m);
    void msg(const MT::String &m);

    CycleTimer ct;

  private slots:
    void process();

  signals:
    void started();
    void finished();
};

