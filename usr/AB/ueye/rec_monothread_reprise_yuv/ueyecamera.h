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
    UEyeCamera(int w, int h, int fps);
    ~UEyeCamera();

    int getWidth();
    int getHeight();
    int getFPS();
    MT::String getName();
    bool getErrFlag();

    bool setup(int c1);
    bool setup(int c1, int c2);
    bool setup(int c1, int c2, int c3);
    bool setup(int c1, int c2, int c3, int c4);

    void init();
    void open();
    void close();
    void exit();
    void grab(char **p);

  private:
    void setupCommon();
    bool camInit(int cid);
    bool camOpen(int cid);
    bool camClose(int cid);
    bool camExit(int cid);
    bool grabImage(int cid, char *p);

  private:
    bool setup_flag, init_flag, open_flag;

  public:


    private:
      void grab(int cid);

    void startRec();
    void stopRec();

    void quit();

    static int getNumCameras();

  private:
    int width, height, fps;

    int nrecframes;

    int nUsedCams;
    HIDS *camID;
    SENSORINFO *camInfo;
    MT::String name; // TODO list of strings?

    int cid;
    INT camStatus;

    bool quit_flag, err_flag;

    char **img, **imgCopy;
    INT *imgBuffNum;
    UEYEIMAGEINFO *imgInfo;
    UEYE_CAPTURE_STATUS_INFO *captInfo;

    int numBuff;
    char ***camBuff;
    INT **camBuffID;

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

