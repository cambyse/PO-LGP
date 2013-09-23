#pragma once

#include <QObject>
#include <QMutex>
#include <Core/array.h>
#include <ueye.h>
#include "recworker.h"

class UEyeCamera: public QObject {
  Q_OBJECT

  private:
    int width, height, fps;

    int nUsedCams;
    HIDS *camID;
    SENSORINFO *camInfo;
    StringL name;

    int cid;
    INT camStatus;

    bool setup_flag, init_flag, open_flag, quit_flag, err_flag;

    char **img, **imgCopy;
    INT *imgBuffNum;
    UEYEIMAGEINFO *imgInfo;
    UEYE_CAPTURE_STATUS_INFO *captInfo;

  public:
    UEyeCamera(int w, int h, int fps);
    ~UEyeCamera();

    int getWidth();
    int getHeight();
    int getFPS();
    bool getErrFlag();

    void setup(int c1);
    void setup(int c1, int c2);
    void setup(int c1, int c2, int c3);
    void setup(int c1, int c2, int c3, int c4);
    void setdown(); // TODO public or private?

    // NB very important, never call these if process is underway
    void init();
    void open();
    void close();
    void exit();

    void quit();
    void queryImage(int c, char *p);
    bool queryError();

    void startRec();
    void stopRec();

  private slots:
    void camProcess();

  private:
    void setupCommon();
    void camInit();
    void camOpen();
    void camGrab();
    void camClose();
    void camExit();

    // UEye API wrappers
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

    // check camera and capture status codes
    void handleCamStatus();
    void handleCaptStatus();

  signals:
    void started();
    void finished();

  public:
    // TODO fix

    static int getNumCameras();

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

    CycleTimer ct;

};

