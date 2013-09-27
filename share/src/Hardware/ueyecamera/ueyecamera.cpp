#include <Core/thread.h>
#include <ueye.h>
#include "ueyecamera.h"

using namespace throut;

void lib_hardware_ueyecamera() { throut::throut("loading ueyecamera"); }

REGISTER_MODULE(UEyePoller)

const unsigned int ueye_width = 1280;
const unsigned int ueye_height = 1024;
const unsigned int ueye_fps = 60;
const unsigned int ueye_bpp = 24;
const unsigned int ueye_bypp = ueye_bpp / 8;
const unsigned int ueye_size = ueye_width * ueye_height * ueye_bypp;

//===========================================================================
//
// C++ interface to ueye
//

struct sUEyeInterface {
  public:
    UEyePoller *module;

    int nUsedCams;
    HIDS *camID;
    SENSORINFO *camInfo;
    StringA name;

    int numBuff;
    char ***camBuff;
    INT **camBuffID;

    UINT pixelclock;
    double real_fps, live_fps;
    double exposure;

    int cid;
    INT camStatus;

    char **img;
    INT *imgBuffNum;
    UEYEIMAGEINFO *imgInfo;
    UEYE_CAPTURE_STATUS_INFO *captInfo;

    bool setup_flag, init_flag, open_flag, err_flag; // TODO err_flag?

    sUEyeInterface();
    ~sUEyeInterface();

    void setup(int c1);
    void setup(int c1, int c2);
    void setup(int c1, int c2, int c3);
    void setup(int c1, int c2, int c3, int c4);
    void setupCommon();

    // NB very important, never call these if process is underway
    void init();
    void open();
    void grab();
    void close();
    void exit();

    void camInit();
    void camOpen();
    void camGrab();
    void camClose();
    void camExit();

    // UNIX timestamp from camera timestamp, in string format
    char *getTimeStamp();

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
};

sUEyeInterface::sUEyeInterface(): module(NULL), img(NULL), setup_flag(false), init_flag(false), open_flag(false), err_flag(false) {
  throut::throutRegHeading(this, "UEyeCamera(*): ");
}

sUEyeInterface::~sUEyeInterface() {
  for(int c = 0; c < nUsedCams; c++)
    throut::throutUnregHeading(&camID[c]);
  throut::throutUnregHeading(this);

  delete[] camID;
  delete[] camInfo;

  for(int c = 0; c < nUsedCams; c++) {
    delete[] camBuff[c];
    delete[] camBuffID[c];
  }

  delete[] img;
  delete[] imgBuffNum;
  delete[] imgInfo;
  delete[] captInfo;

  delete[] camBuff;
  delete[] camBuffID;
}

void sUEyeInterface::setup(int c1) {
  if(setup_flag)
    return;

  nUsedCams = 1;
  setupCommon();
  camID[0] = c1;
}

void sUEyeInterface::setup(int c1, int c2) {
  if(setup_flag)
    return;

  nUsedCams = 2;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
}

void sUEyeInterface::setup(int c1, int c2, int c3) {
  if(setup_flag)
    return;

  nUsedCams = 3;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
  camID[2] = c3;
}

void sUEyeInterface::setup(int c1, int c2, int c3, int c4) {
  if(setup_flag)
    return;

  nUsedCams = 4;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
  camID[2] = c3;
  camID[3] = c4;
}

void sUEyeInterface::setupCommon() {
  camID = new HIDS[nUsedCams];
  camInfo = new SENSORINFO[nUsedCams];
  name.resize(nUsedCams);

  img = new char*[nUsedCams];
  imgBuffNum = new INT[nUsedCams];
  imgInfo = new UEYEIMAGEINFO[nUsedCams];
  captInfo = new UEYE_CAPTURE_STATUS_INFO[nUsedCams];

  camBuff = new char**[nUsedCams];
  camBuffID = new INT*[nUsedCams];

  // setting up ACCESS variables
  module->ueye_num.set() = nUsedCams;
  module->ueye_rgb.set().resize(nUsedCams, ueye_size);
  module->ueye_fps.set().resize(nUsedCams);

  setup_flag = true;
}

void sUEyeInterface::init() {
  if(!setup_flag || init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camInit();
  init_flag = true;
}

void sUEyeInterface::open() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camOpen();
  open_flag = true;
}

void sUEyeInterface::grab() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camGrab();
}

void sUEyeInterface::close() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camClose();
  open_flag = false;
}

void sUEyeInterface::exit() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camExit();
  init_flag = false;
}

void sUEyeInterface::camInit() {
  throut::throut(this, "camInit()");
  InitCamera_wr();
  if(err_flag) return;

  throut::throutRegHeading(&camID[cid], STRING("UEyeCamera(" << cid << "): "));

  name(cid) << "Video_" << camID[cid];
  throut::throut(&camID[cid], STRING("- camID = " << camID[cid]));
  throut::throut(&camID[cid], STRING("- name = " << name(cid)));

  SetColorMode_wr(IS_CM_BGR8_PACKED);
  if(err_flag) return;

  SetColorConverter_wr(IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  if(err_flag) return;

  SetExternalTrigger_wr(IS_SET_TRIGGER_OFF);
  if(err_flag) return;

  /*
  GetSensorInfo_wr();
  throut::throut(&camID[cid], STRING("- sensor ID = " << camInfo[cid].SensorID));
  throut::throut(&camID[cid], STRING("- camera model = " << camInfo[cid].strSensorName));
  throut::throut(&camID[cid], STRING("- max width = " << camInfo[cid].nMaxWidth));
  throut::throut(&camID[cid], STRING("- max height = " << camInfo[cid].nMaxHeight));
  throut::throut(&camID[cid], STRING("- pixel size = " << (float)camInfo[cid].wPixelSize/100 << " µm"));
  */

  numBuff = 30;
  camBuff[cid] = new char*[numBuff];
  camBuffID[cid] = new INT[numBuff];

  ClearSequence_wr();
  if(err_flag) return;
  for(int i = 0; i < numBuff; i++) {
    AllocImageMem_wr(&camBuff[cid][i], &camBuffID[cid][i]);
    if(err_flag) return;
    AddToSequence_wr(camBuff[cid][i], camBuffID[cid][i]);
    if(err_flag) return;
  }

  // SEPARATION

  UINT pr[3];
  memset(pr, 0, 3*sizeof(UINT));

  // query possible values
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pr, sizeof(pr));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1]));

  // set value
  pixelclock = pr[1];
  PixelClock_wr(IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- set pixelclock = " << pixelclock));

  // check/read value
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- real pixelclock = " << pixelclock));

  SetFrameRate_wr();
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- set fps = " << ueye_fps));
  throut::throut(&camID[cid], STRING("- real fps = " << real_fps));

  double er[3];
  memset(er, 0, 3*sizeof(double));

  // query possible values
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)er, sizeof(er));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- exposure range = " << er[0] << ":" << er[2] << ":" << er[1]));

  // set value
  exposure = er[1];
  Exposure_wr(IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- set exposure = " << exposure));

  // check/read value
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  throut::throut(&camID[cid], STRING("- real exposure = " << exposure));
}

void sUEyeInterface::camOpen() {
  throut::throut(&camID[cid], "camOpen()");
  CaptureVideo_wr(IS_WAIT);
  if(err_flag) return;
  InitImageQueue_wr();
  if(err_flag) return;
}

void sUEyeInterface::camGrab() {
  img[cid] = NULL;
  imgBuffNum[cid] = 0;

  WaitForNextImage_wr();
  memcpy(module->ueye_rgb.set()[cid]().p, img[cid], ueye_size);
  
  UnlockSeqBuf_wr(imgBuffNum[cid], img[cid]);

  GetFramesPerSecond_wr();
  module->ueye_fps.set()(cid) = live_fps;
}

void sUEyeInterface::camClose() {
  throut::throut(&camID[cid], "camClose()");

  ExitImageQueue_wr();
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[cid][i], camBuffID[cid][i]);
}

void sUEyeInterface::camExit() {
  throut::throut(&camID[cid], "camExit()");
  ExitCamera_wr();
}

char* sUEyeInterface::getTimeStamp() {
  long int s = imgInfo[cid].TimestampSystem.wSecond;
  long int m = imgInfo[cid].TimestampSystem.wMinute;
  long int h = imgInfo[cid].TimestampSystem.wHour;
  long int y = imgInfo[cid].TimestampSystem.wYear;
  long int d = imgInfo[cid].TimestampSystem.wDay;
  
  switch(imgInfo[cid].TimestampSystem.wMonth) {
    case 12:
      d += 30;
    case 11:
      d += 31;
    case 10:
      d += 30;
    case 9:
      d += 31;
    case 8:
      d += 31;
    case 7:
      d += 30;
    case 6:
      d += 31;
    case 5:
      d += 30;
    case 4:
      d += 31;
    case 3:
      d += 28;
      if( ((y%4 == 0) && (y%100 != 0)) || y%400 == 0 ) // leap year
        ++d;
    case 2:
      d += 31; // day in current year
  }
  --d; // 0-based number of days
  y -= 1900; // year since 1900

  long int sec = s + 60*m + 3600*h + 86400*d + \
                  (y-70)*31536000 + ((y-69)/4)*86400 - \
                  ((y-1)/100)*86400 + ((y+299)/400)*86400;

  long int usec = 1000*imgInfo[cid].TimestampSystem.wMilliseconds;

  char *ts = new char[30];
  sprintf(ts, "%8li.%06li", sec, usec);

  return ts;
}

void sUEyeInterface::InitCamera_wr() {
  camStatus = is_InitCamera(&camID[cid], NULL);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "InitCamera() failed");
  handleCamStatus();
}

void sUEyeInterface::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID[cid], mode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetColorMode() failed");
  handleCamStatus();
}

void sUEyeInterface::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID[cid], ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetColorConverter() failed");
  handleCamStatus();
}

void sUEyeInterface::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID[cid], Mode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetDisplayMode() failed");
  handleCamStatus();
}

void sUEyeInterface::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID[cid], nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetExternalTrigger() failed");
  handleCamStatus();
}

void sUEyeInterface::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID[cid], &camInfo[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetSensorInfo() failed");
  handleCamStatus();
}

void sUEyeInterface::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID[cid], ueye_width, ueye_height, ueye_bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "AllocImageMem() failed");
  handleCamStatus();
}

void sUEyeInterface::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "FreeImageMem() failed");
  handleCamStatus();
}

void sUEyeInterface::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ClearSequence() failed");
  handleCamStatus();
}

void sUEyeInterface::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "AddToSequence() failed");
  handleCamStatus();
}

void sUEyeInterface::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "PixelClock() failed");
  handleCamStatus();
}

void sUEyeInterface::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID[cid], ueye_fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetFrameRate() failed");
  handleCamStatus();
}

void sUEyeInterface::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "Exposure() failed");
  handleCamStatus();
}

void sUEyeInterface::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "CaptureVideo() failed");
  handleCamStatus();
}

void sUEyeInterface::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID[cid], 0);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "InitImageQueue() failed");
  handleCamStatus();
}

void sUEyeInterface::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ExitImageQueue() failed");
  handleCamStatus();
}

void sUEyeInterface::WaitForNextImage_wr() {
  camStatus = is_WaitForNextImage(camID[cid], 1<<31, &img[cid], &imgBuffNum[cid]);
  switch(camStatus) {
    case IS_CAPTURE_STATUS:
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_GET);
      handleCaptStatus();
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_RESET);
      WaitForNextImage_wr();
    case IS_SUCCESS:
      return;
  }
  throut::throut(&camID[cid], "WaitForNextImage() failed");
  handleCamStatus();
}

void sUEyeInterface::CaptureStatus_wr(UINT nCommand) {
  camStatus = is_CaptureStatus(camID[cid], nCommand, (void*)&captInfo[cid], sizeof(captInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "CaptureStatus() failed");
  handleCamStatus();
}

void sUEyeInterface::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID[cid], imgBuffNum[cid], &imgInfo[cid], sizeof(imgInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetImageInfo() failed");
  handleCamStatus();
}

void sUEyeInterface::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID[cid], buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "UnlockSeqBuf() failed");
  handleCamStatus();
}

void sUEyeInterface::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID[cid], &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetFramesPerSecond() failed");
  handleCamStatus();
}

void sUEyeInterface::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "StopLiveVideo() failed");
  handleCamStatus();
}

void sUEyeInterface::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ExitCamera() failed");
  handleCamStatus();
}

void sUEyeInterface::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID[cid], &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    throut::throut(&camID[cid], (const char *)m);
    return;
  }
  throut::throut(&camID[cid], "GetError() failed");
  handleCamStatus();
}

#define UEYE_ERR_CASE(X) case X: throut::throut(&camID[cid], #X); break;

void sUEyeInterface::handleCamStatus() {
  switch(camStatus) {
    case IS_NO_SUCCESS:
      GetError_wr(); break;
    UEYE_ERR_CASE(IS_ALL_DEVICES_BUSY)
    UEYE_ERR_CASE(IS_BAD_STRUCTURE_SIZE)
    UEYE_ERR_CASE(IS_CANT_ADD_TO_SEQUENCE)
    UEYE_ERR_CASE(IS_CANT_CLEANUP_MEMORY)
    UEYE_ERR_CASE(IS_CANT_COMMUNICATE_WITH_DRIVER)
    UEYE_ERR_CASE(IS_CANT_OPEN_DEVICE)
    UEYE_ERR_CASE(IS_CANT_OPEN_REGISTRY)
    UEYE_ERR_CASE(IS_CANT_READ_REGISTRY)
    UEYE_ERR_CASE(IS_CAPTURE_RUNNING)
    UEYE_ERR_CASE(IS_CRC_ERROR)
    UEYE_ERR_CASE(IS_DEVICE_ALREADY_PAIRED)
    UEYE_ERR_CASE(IS_DEVICE_NOT_COMPATIBLE)
    UEYE_ERR_CASE(IS_DR_CANNOT_CREATE_SURFACE)
    UEYE_ERR_CASE(IS_DR_CANNOT_CREATE_TEXTURE)
    UEYE_ERR_CASE(IS_DR_CANNOT_CREATE_VERTEX_BUFFER)
    UEYE_ERR_CASE(IS_DR_DEVICE_OUT_OF_MEMORY)
    UEYE_ERR_CASE(IS_DR_LIBRARY_NOT_FOUND)
    UEYE_ERR_CASE(IS_ERROR_CPU_IDLE_STATES_CONFIGURATION)
    UEYE_ERR_CASE(IS_FILE_WRITE_OPEN_ERROR)
    UEYE_ERR_CASE(IS_INCOMPATIBLE_SETTING)
    UEYE_ERR_CASE(IS_INVALID_BUFFER_SIZE)
    UEYE_ERR_CASE(IS_INVALID_CAMERA_TYPE)
    UEYE_ERR_CASE(IS_INVALID_CAMERA_HANDLE) // equal to IS_INVALID_HANDLE
    UEYE_ERR_CASE(IS_INVALID_CAPTURE_MODE)
    UEYE_ERR_CASE(IS_INVALID_COLOR_FORMAT)
    UEYE_ERR_CASE(IS_INVALID_DEVICE_ID)
    UEYE_ERR_CASE(IS_INVALID_EXPOSURE_TIME)
    UEYE_ERR_CASE(IS_INVALID_IP_CONFIGURATION)
    UEYE_ERR_CASE(IS_INVALID_MEMORY_POINTER)
    UEYE_ERR_CASE(IS_INVALID_MODE)
    UEYE_ERR_CASE(IS_INVALID_PARAMETER)
    UEYE_ERR_CASE(IS_INVALID_PIXEL_CLOCK)
    UEYE_ERR_CASE(IS_IO_REQUEST_FAILED)
    UEYE_ERR_CASE(IS_NETWORK_CONFIGURATION_INVALID)
    UEYE_ERR_CASE(IS_NETWORK_FRAME_SIZE_INCOMPATIBLE)
    UEYE_ERR_CASE(IS_NO_ACTIVE_IMG_MEM)
    UEYE_ERR_CASE(IS_NO_IMAGE_MEM_ALLOCATED)
    UEYE_ERR_CASE(IS_NO_IR_FILTER)
    UEYE_ERR_CASE(IS_NOT_CALIBRATED)
    UEYE_ERR_CASE(IS_NOT_SUPPORTED)
    UEYE_ERR_CASE(IS_NULL_POINTER)
    UEYE_ERR_CASE(IS_OUT_OF_MEMORY)
    UEYE_ERR_CASE(IS_SEQUENCE_BUF_ALREADY_LOCKED)
    UEYE_ERR_CASE(IS_SEQUENCE_LIST_EMPTY)
    UEYE_ERR_CASE(IS_STARTER_FW_UPLOAD_NEEDED)
    UEYE_ERR_CASE(IS_SUBNET_MISMATCH)
    UEYE_ERR_CASE(IS_SUBNETMASK_MISMATCH)
    UEYE_ERR_CASE(IS_TIMED_OUT)
    UEYE_ERR_CASE(IS_TRIGGER_ACTIVATED)
    default:
      throut::throut(&camID[cid], "error - unhandled camStatus");
  }
  err_flag = true;
}

#define UEYE_ERR_IF(X) if(captInfo[cid].adwCapStatusCnt_Detail[X]) \
  throut::throut(&camID[cid], STRING("CaptureStatus: " << captInfo[cid].adwCapStatusCnt_Detail[X] << " of " << #X));

void sUEyeInterface::handleCaptStatus() {
  if(captInfo[cid].dwCapStatusCnt_Total > 0) {
    throut::throut(&camID[cid], STRING("CaptureStatus: " << captInfo[cid].dwCapStatusCnt_Total << " elements"));
    UEYE_ERR_IF(IS_CAP_STATUS_API_NO_DEST_MEM)
    UEYE_ERR_IF(IS_CAP_STATUS_API_CONVERSION_FAILED)
    UEYE_ERR_IF(IS_CAP_STATUS_API_IMAGE_LOCKED)
    UEYE_ERR_IF(IS_CAP_STATUS_DRV_OUT_OF_BUFFERS)
    UEYE_ERR_IF(IS_CAP_STATUS_DRV_DEVICE_NOT_READY)
    UEYE_ERR_IF(IS_CAP_STATUS_USB_TRANSFER_FAILED)
    UEYE_ERR_IF(IS_CAP_STATUS_DEV_TIMEOUT)
    UEYE_ERR_IF(IS_CAP_STATUS_ETH_BUFFER_OVERRUN)
    UEYE_ERR_IF(IS_CAP_STATUS_ETH_MISSED_IMAGES)
  }
}

//===========================================================================
//
// Poller
//

UEyePoller::UEyePoller() : Module("UEyeInterface"), s(NULL) {
  throut::throutRegHeading(this, "UEyePoller: ");
}

UEyePoller::~UEyePoller() {
  throut::throutUnregHeading(this);
}

void UEyePoller::open() {
  throut::throut(this, "opening");

  uint numCams = MT::getParameter<int>("ueye_numCams");
  // TODO how to get the actual camera numbers through parameters

  s = new sUEyeInterface();
  s->module = this;

  switch(numCams) {
    case 1:
      s->setup(0);
      break;
    case 2:
      s->setup(0, 0);
      break;
    case 3:
      s->setup(0, 0, 0);
      break;
    case 4:
      s->setup(0, 0, 0, 0);
      break;
    default:
      throut::throut(this, "WRONG NUMBER OF CAMERAS");
      break;
  }
  s->init();
  s->open();

  throut::throut(this, "opened successfully");
}

void UEyePoller::step() {
  s->grab();
}

void UEyePoller::close() {
  throut::throut(this, "closing");
  s->close();
  s->exit();
  delete s;
  throut::throut(this, "closed successfully");
}

