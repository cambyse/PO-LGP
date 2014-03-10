#include <Core/thread.h>
#include <ueye.h>
#include "ueyecamera.h"

REGISTER_MODULE(UEyePoller)

void lib_hardware_ueyecamera() { cout << "loading ueyecamera" << endl; }

//const unsigned int c_ueye_width = 1280;
//const unsigned int c_ueye_height = 1024;
const unsigned int c_ueye_width = 1280;
const unsigned int c_ueye_height = 1024;
const unsigned int c_ueye_fps = 60;
const unsigned int c_ueye_bpp = 24;
const unsigned int c_ueye_bypp = c_ueye_bpp / 8;
const unsigned int c_ueye_size = c_ueye_width * c_ueye_height * c_ueye_bypp;

//===========================================================================
//
// C++ interface to ueye
//

TStream tout(cout);

struct sUEyeInterface {
  public:
    HIDS camID;
    SENSORINFO camInfo;
    String name;

    int numBuff;
    char **camBuff;
    INT *camBuffID;

    UINT pixelclock;
    double real_fps, live_fps;
    double exposure;
    int frame_count;

    uint cid;
    INT camStatus;
    double tstamp;

    char *img;
    INT imgBuffNum;
    UEYEIMAGEINFO imgInfo;
    UEYE_CAPTURE_STATUS_INFO captInfo;

    bool setup_flag, init_flag, open_flag, err_flag; // TODO err_flag?

    sUEyeInterface(int cameraID);
    ~sUEyeInterface();

    // NB very important, never call these if process is underway
    void camSetup();
    void camInit();
    void camOpen();
    bool camGrab(byteA& image, double& timestamp, unsigned int timeout=1<<31);
    void camClose();
    void camExit();

    // UNIX timestamp from camera timestamp, in string format
    void updateTimestamp();

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

    bool WaitForNextImage_wr(unsigned int timeout=1<<31);
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

sUEyeInterface::sUEyeInterface(int cameraID): camID(cameraID), img(NULL), setup_flag(false), init_flag(false), open_flag(false), err_flag(false) {
	//camID = MT::getParameter<int>(STRING(m->name << "_camID"));
	tout.reg(this) << "UEyeCamera(" << camID << "): ";
}

sUEyeInterface::~sUEyeInterface() {
  tout.unreg(this);

  delete[] camBuff;
  delete[] camBuffID;
}

void sUEyeInterface::camSetup() {
  if(setup_flag)
    return;

  uint enable = IS_CONFIG_OPEN_MP_ENABLE;
  int ret;
  switch((ret = is_Configuration(IS_CONFIG_OPEN_MP_CMD_SET_ENABLE, &enable, 4))) {
      case IS_SUCCESS:
      tout(this) << "Using OpenMP for uEYE color conversion" << endl;
      break;
    case IS_NOT_SUPPORTED:
      tout(this) << "OpenMP for uEYE color conversion not supported" << endl;
      break;
  default:
      tout(this) << "Error configuring OpenMP: " << ret << endl;
  }

  tout(this) << "camSetup()" << endl;
  setup_flag = true;
  frame_count = 0;
}

void sUEyeInterface::camInit() {
  if(!setup_flag || init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "camInit()" << endl;
  InitCamera_wr();
  if(err_flag) return;

  name << "Video_" << camID;
  tout(this) << "- camID = " << camID << endl;
  tout(this) << "- name = " << name << endl;

  SetColorMode_wr(IS_CM_BGR8_PACKED);
  //SetColorMode_wr(IS_CM_UYVY_PACKED);
  if(err_flag) return;

  SetColorConverter_wr(IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
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
  tout(this) << "- pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1] << endl;

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
  //if(err_flag) return;
  tout(this) << "- set fps = " << c_ueye_fps << endl;
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

void sUEyeInterface::camOpen() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "camOpen()" << endl;
  CaptureVideo_wr(IS_WAIT);
  if(err_flag) return;

  InitImageQueue_wr();
  if(err_flag) return;

  open_flag = true;
}

bool sUEyeInterface::camGrab(byteA& image, double& timestamp, unsigned int timeout) {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    tout(this) << "bad prep before grab: setup="
    		<< setup_flag << ", init=" << init_flag << ", open=" << open_flag << endl;
    return false;
  }

  img = NULL;
  imgBuffNum = 0;
  if(WaitForNextImage_wr(timeout)) {
	  updateTimestamp();
	  // make sure receiver is large enough
	  image.resize(c_ueye_height, c_ueye_width, c_ueye_bypp);
	  memcpy(image.p, img, c_ueye_size);
	  timestamp = tstamp;

	  UnlockSeqBuf_wr(imgBuffNum, img);

	  // TODO do we even want this?
	  frame_count++;
	  if(frame_count % 1000 == 0) {
		  GetFramesPerSecond_wr();
		  tout(this) << "current fps: " << live_fps << endl;
		  //module->ueye_fps.set() = live_fps;
	  }
	  return true;
  } else {
	  return false;
  }
}

void sUEyeInterface::camClose() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "camClose()" << endl;

  ExitImageQueue_wr();
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[i], camBuffID[i]);

  open_flag = false;
}

void sUEyeInterface::camExit() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }

  tout(this) << "camExit()" << endl;
  ExitCamera_wr();

  init_flag = false;
}

void sUEyeInterface::updateTimestamp() {
  GetImageInfo_wr();

  // TODO missing fields ? what to do with them?
  tm tmp;
  tmp.tm_sec = imgInfo.TimestampSystem.wSecond;
  tmp.tm_min = imgInfo.TimestampSystem.wMinute;
  tmp.tm_hour = imgInfo.TimestampSystem.wHour;
  tmp.tm_mday = imgInfo.TimestampSystem.wDay;
  tmp.tm_mon = imgInfo.TimestampSystem.wMonth - 1;
  tmp.tm_year = imgInfo.TimestampSystem.wYear - 1900;
  //tmp.tm_wday = ???; // ignored
  //tmp.tm_yday = ???; // ignored
  tmp.tm_isdst = -1; // -1 means info not available. Maybe set it?

  /*
  char b[101];
  sprintf(b, "%02d.%02d.%04d, %02d:%02d:%02d:%03d",
    imgInfo.TimestampSystem.wDay,
    imgInfo.TimestampSystem.wMonth,
    imgInfo.TimestampSystem.wYear,
    imgInfo.TimestampSystem.wHour,
    imgInfo.TimestampSystem.wMinute,
    imgInfo.TimestampSystem.wSecond,
    imgInfo.TimestampSystem.wMilliseconds);
  cout << "imgInfo: " << b << endl;
  strftime(b, 20, "%c", &tmp);
  cout << "tm: " << b << endl;
  */

  tstamp = MT::toTime(tmp)
          + imgInfo.TimestampSystem.wMilliseconds / 1000.;
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
  camStatus = is_AllocImageMem(camID, c_ueye_width, c_ueye_height, c_ueye_bpp, buff, buffID);
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
  camStatus = is_SetFrameRate(camID, c_ueye_fps, &real_fps);
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

bool sUEyeInterface::WaitForNextImage_wr(unsigned int timeout) {
	unsigned int attempt = 0;
	while(attempt++ < 3) {
	  camStatus = is_WaitForNextImage(camID, timeout, &img, &imgBuffNum);
	  switch(camStatus) {
		case IS_CAPTURE_STATUS:
		  CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_GET);
		  handleCaptStatus();
		  CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_RESET);
		  break;
		case IS_TIMED_OUT:
			tout(this) << "timeout waiting for image" << endl;
			return false;
		case IS_SUCCESS:
		  return true;
	  }
	}
  tout(this) << "WaitForNextImage() failed" << endl;
  handleCamStatus();
  return false;
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
      tout(this) << "error - unhandled camStatus: " << camStatus << endl;
  }
  err_flag = true;
  HALT("FIND THAT ERROR!");
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

void UEyePoller::open() {
  tout(this) << "opening" << endl;

  s = new sUEyeInterface(MT::getParameter<int>(STRING(name << "_camID")));

  s->camSetup();
  s->camInit();
  s->camOpen();

  tout(this) << "opened successfully" << endl;
}

void UEyePoller::step() {
	Access_typed<byteA>::WriteToken token(&ueye_rgb);
	s->camGrab(ueye_rgb(), ueye_rgb.tstamp());
}

void UEyePoller::close() {
  tout(this) << "closing" << endl;
  s->camClose();
  s->camExit();
  delete s;
  tout(this) << "closed successfully" << endl;
}

