#ifdef UEYE_INSTALLED

#include <Core/thread.h>
#include <ueye.h>
#include "ueyecamera.h"

void lib_hardware_ueyecamera() { cout << "loading ueyecamera" << endl; }

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

TStream tout(cout);

struct sUEyeInterface {
  public:
    UEyePoller *module;

    HIDS camID;
    SENSORINFO camInfo;
    String name;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    UINT pixelclock;
    double real_fps, live_fps;
    double exposure;

    int cid;
    INT camStatus;

    char *img;
    INT imgBuffNum;
    UEYEIMAGEINFO imgInfo;
    UEYE_CAPTURE_STATUS_INFO captInfo;

    bool setup_flag, init_flag, open_flag, err_flag; // TODO err_flag?

    sUEyeInterface(uint cid);
    ~sUEyeInterface();

    // NB very important, never call these if process is underway
    void setup();
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

sUEyeInterface::sUEyeInterface(uint cid): module(NULL), camID(cid), img(NULL), setup_flag(false), init_flag(false), open_flag(false), err_flag(false) {
  tout.reg(this) << "UEyeCamera(" << camID << "): ";
}

sUEyeInterface::~sUEyeInterface() {
  tout.unreg(this);

  delete[] camBuff;
  delete[] camBuffID;
}

void sUEyeInterface::setup() {
  if(setup_flag)
    return;
  /*
  module->ueye_rgb.set().resize(nUsedCams, ueye_size);
  module->ueye_fps.set().resize(nUsedCams);
  */
  setup_flag = true;
}

void sUEyeInterface::init() {
  if(!setup_flag || init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "init()" << endl;
  InitCamera_wr();
  if(err_flag) return;

  name << "Video_" << camID;
  tout(this) << "- camID = " << camID << endl;
  tout(this) << "- name = " << name << endl;

  SetColorMode_wr(IS_CM_BGR8_PACKED);
  if(err_flag) return;

  SetColorConverter_wr(IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  if(err_flag) return;

  SetExternalTrigger_wr(IS_SET_TRIGGER_OFF);
  if(err_flag) return;

  /*
  GetSensorInfo_wr();
  tout(this) << "- sensor ID = " << camInfo.SensorID << endl;
  tout(this) << "- camera model = " << camInfo.strSensorName << endl;
  tout(this) << "- max width = " << camInfo.nMaxWidth << endl;
  tout(this) << "- max height = " << camInfo.nMaxHeight << endl;
  tout(this) << "- pixel size = " << (float)camInfo.wPixelSize/100 << " µm" << endl;
  */

  numBuff = 30;
  camBuff = new char*[numBuff];
  camBuffID = new INT[numBuff];

  ClearSequence_wr();
  if(err_flag) return;
  for(int i = 0; i < numBuff; i++) {
    AllocImageMem_wr(&camBuff[i], &camBuffID[i]);
    if(err_flag) return;
    AddToSequence_wr(camBuff[i], camBuffID[i]);
    if(err_flag) return;
  }

  // SEPARATION

  UINT pr[3];
  memset(pr, 0, 3*sizeof(UINT));

  // query possible values
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pr, sizeof(pr));
  if(err_flag) return;
  tout(this) << "- poxelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1] << endl;

  // set value
  pixelclock = pr[1];
  PixelClock_wr(IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  tout(this) << "- set pixelclock = " << pixelclock << endl;

  // check/read value
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  tout(this) << "- real pixelclock = " << pixelclock << endl;

  SetFrameRate_wr();
  if(err_flag) return;
  tout(this) << "- set fps = " << ueye_fps << endl;
  tout(this) << "- real fps = " << real_fps << endl;

  double er[3];
  memset(er, 0, 3*sizeof(double));

  // query possible values
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)er, sizeof(er));
  if(err_flag) return;
  tout(this) << "- explosure range = " << er[0] << ":" << er[2] << ":" << er[1] << endl;

  // set value
  exposure = er[1];
  Exposure_wr(IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  tout(this) << "- set exposure = " << exposure << endl;

  // check/read value
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  tout(this) << "- real exposure = " << exposure << endl;

  init_flag = true;
}

void sUEyeInterface::open() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "open()" << endl;
  CaptureVideo_wr(IS_WAIT);
  if(err_flag) return;

  InitImageQueue_wr();
  if(err_flag) return;

  open_flag = true;
}

void sUEyeInterface::grab() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }

  img = NULL;
  imgBuffNum = 0;
  WaitForNextImage_wr();
  //memcpy(module->ueye_rgb.set()().p, img, ueye_size);
  
  UnlockSeqBuf_wr(imgBuffNum, img);

  GetFramesPerSecond_wr();
  module->ueye_fps.set()(cid) = live_fps;
}

void sUEyeInterface::close() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "close()" << endl;

  ExitImageQueue_wr();
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[i], camBuffID[i]);

  open_flag = false;
}

void sUEyeInterface::exit() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "exit()" << endl;
  ExitCamera_wr();

  init_flag = false;
}

char* sUEyeInterface::getTimeStamp() {
  // TODO fix this is probably wrong..
  long int s = imgInfo.TimestampSystem.wSecond;
  long int m = imgInfo.TimestampSystem.wMinute;
  long int h = imgInfo.TimestampSystem.wHour;
  long int y = imgInfo.TimestampSystem.wYear;
  long int d = imgInfo.TimestampSystem.wDay;
  
  switch(imgInfo.TimestampSystem.wMonth) {
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

  long int usec = 1000*imgInfo.TimestampSystem.wMilliseconds;

  char *ts = new char[30];
  sprintf(ts, "%8li.%06li", sec, usec);

  return ts;
}

void sUEyeInterface::InitCamera_wr() {
  camStatus = is_InitCamera(&camID, NULL);
  if(camStatus == IS_SUCCESS) {
    tout.reg(this) << "UEyeCamera(" << camID << "): "; // overwrite the header
    return;
  }
  tout(this) << "InitCamera() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID, mode);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "SetColorMode() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID, ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "SetColorConverter() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID, Mode);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "SetDisplayMode() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID, nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "SetExternalTrigger() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID, &camInfo);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "GetSensorInfo() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID, ueye_width, ueye_height, ueye_bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "AllocImageMem() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "FreeImageMem() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "ClearSequence() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "AddToSequence() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID, nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "PixelClock() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID, ueye_fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "SetFrameRate() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID, nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "Exposure() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID, wait);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "CaptureVideo() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID, 0);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "InitImageQueue() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "ExitImageQueue() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::WaitForNextImage_wr() {
  camStatus = is_WaitForNextImage(camID, 1<<31, &img, &imgBuffNum);
  switch(camStatus) {
    case IS_CAPTURE_STATUS:
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_GET);
      handleCaptStatus();
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_RESET);
      WaitForNextImage_wr();
    case IS_SUCCESS:
      return;
  }
  tout(this) << "WaitForNextImage() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::CaptureStatus_wr(UINT nCommand) {
  camStatus = is_CaptureStatus(camID, nCommand, (void*)&captInfo, sizeof(captInfo));
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "CaptureStatus() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID, imgBuffNum, &imgInfo, sizeof(imgInfo));
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "GetImageInfo() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID, buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "UnlockSeqBuf() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID, &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "GetFramesPerSecond() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID, wait);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "StopLiveVideo() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID);
  if(camStatus == IS_SUCCESS)
    return;
  tout(this) << "ExitCamera() failed" << endl;
  handleCamStatus();
}

void sUEyeInterface::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID, &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    tout(this) << (const char*)m << endl;
    return;
  }
  tout(this) << "GetError() failed" << endl;
  handleCamStatus();
}

#define UEYE_ERR_CASE(X) case X: tout(this) << #X << endl; break;

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
      tout(this) << "error - unhandled camStatus" << endl;
  }
  err_flag = true;
}

#define UEYE_ERR_IF(X) if(captInfo.adwCapStatusCnt_Detail[X]) \
  tout(this) << "CaptureStatus: " << captInfo.adwCapStatusCnt_Detail[X] << " of " << #X << endl;

void sUEyeInterface::handleCaptStatus() {
  if(captInfo.dwCapStatusCnt_Total > 0) {
    tout(this) << "CaptureStatus: " << captInfo.dwCapStatusCnt_Total << " elements" << endl;
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
  tout.reg(this) << "UEyePoller: ";
}

UEyePoller::~UEyePoller() {
  tout.unreg(this);
}

void UEyePoller::open(uint _cid) {
  cid.set() = _cid;
  tout(this) << "opening" << endl;

  //uint numCams = MT::getParameter<int>("ueye_numCams");
  // TODO how to get the actual camera numbers through parameters
  // TODO doesn't matter.. now this code only manages one camera

  s = new sUEyeInterface(_cid);
  s->module = this;

  s->setup(); // TODO include in init?
  s->init();
  s->open();

  tout(this) << "opened successfully" << endl;
}

void UEyePoller::step() {
  s->grab();
}

void UEyePoller::close() {
  tout(this) << "closing" << endl;
  s->close();
  s->exit();
  delete s;
  tout(this) << "closed successfully" << endl;
}

#endif // UEYE_INSTALLED
