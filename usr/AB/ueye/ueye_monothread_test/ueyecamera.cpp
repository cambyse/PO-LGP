#include <iostream>
#include <cstdlib>
#include <cstdio>
#include "ueyecamera.h"

#include <QThread>
using namespace std;

UEyeCamera::UEyeCamera(int w, int h, int f): width(w), height(h), fps(f) {
  img = NULL;

  err_flag = false;

  setup_flag = false;
  init_flag = false;
  open_flag = false;
}

UEyeCamera::~UEyeCamera() {
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

int UEyeCamera::getNumCameras() {
  // TODO clean and check for errors here.
  INT numCams;
  INT status = is_GetNumberOfCameras(&numCams);
  if(status == IS_SUCCESS)
    return (int)numCams;

  cout << "getNumCameras(): An error of some sort" << endl;
  return -1;
}

void UEyeCamera::setup(int c1) {
  if(setup_flag)
    return;

  nUsedCams = 1;
  setupCommon();
  camID[0] = c1;
}

void UEyeCamera::setup(int c1, int c2) {
  if(setup_flag)
    return;

  nUsedCams = 2;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
}

void UEyeCamera::setup(int c1, int c2, int c3) {
  if(setup_flag)
    return;

  nUsedCams = 3;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
  camID[2] = c3;
}

void UEyeCamera::setup(int c1, int c2, int c3, int c4) {
  if(setup_flag)
    return;

  nUsedCams = 4;
  setupCommon();
  camID[0] = c1;
  camID[1] = c2;
  camID[2] = c3;
  camID[3] = c4;
}

void UEyeCamera::init() {
  if(!setup_flag || init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camInit();
  init_flag = true;
  emit inited();
}

void UEyeCamera::open() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camOpen();
  open_flag = true;
  emit opened();
}

void UEyeCamera::close() {
  if(!setup_flag || !init_flag || !open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camClose();
  open_flag = false;
  emit closed();
}

void UEyeCamera::exit() {
  if(!setup_flag || !init_flag || open_flag) {
    err_flag = true;
    return;
  }
  for(cid = 0; cid < nUsedCams; cid++)
    camExit();
  init_flag = false;
  emit exited();
}

void UEyeCamera::setupCommon() {
  camID = new HIDS[nUsedCams];
  camInfo = new SENSORINFO[nUsedCams];

  img = new char*[nUsedCams];
  imgBuffNum = new INT[nUsedCams];
  imgInfo = new UEYEIMAGEINFO[nUsedCams];
  captInfo = new UEYE_CAPTURE_STATUS_INFO[nUsedCams];

  camBuff = new char**[nUsedCams];
  camBuffID = new INT*[nUsedCams];

  setup_flag = true;
}

void UEyeCamera::camInit() {
  cout << "camInit()" << endl;

  InitCamera_wr();
  if(err_flag) return;

  SetColorMode_wr(IS_CM_BGR8_PACKED);
  if(err_flag) return;
  bpp = 24;
  bypp = bpp/8;
  bypimg = bypp * width * height;

  SetColorConverter_wr(IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  if(err_flag) return;

  SetExternalTrigger_wr(IS_SET_TRIGGER_OFF);
  if(err_flag) return;

  numBuff = 9;
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
  cout << "- pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1] << endl;

  // set value
  pixelclock = pr[1];
  PixelClock_wr(IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  cout << "- set pixelclock = " << pixelclock << endl;

  // check/read value
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET, (void*)&pixelclock, sizeof(pixelclock));
  if(err_flag) return;
  cout << "- real pixelclock = " << pixelclock << endl;

  SetFrameRate_wr();
  if(err_flag) return;
  cout << "- set fps = " << fps << endl;
  cout << "- real fps = " << real_fps << endl;

  double er[3];
  memset(er, 0, 3*sizeof(double));

  // query possible values
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)er, sizeof(er));
  if(err_flag) return;
  cout << "- exposure range = " << er[0] << ":" << er[2] << ":" << er[1] << endl;

  // set value
  exposure = er[1];
  Exposure_wr(IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  cout << "- set exposure = " << exposure << endl;

  // check/read value
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  if(err_flag) return;
  cout << "- real exposure = " << exposure << endl;
}

void UEyeCamera::camOpen() {
  cout << "camOpen()" << endl;
  CaptureVideo_wr(IS_WAIT);//100);//IS_WAIT); // IS_WAIT or IS_DONT_WAIT
  if(err_flag) return;
  InitImageQueue_wr();
  if(err_flag) return;
}

void UEyeCamera::camClose() {
  cout << "camClose()" << endl;

  ExitImageQueue_wr();
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[cid][i], camBuffID[cid][i]);
}

void UEyeCamera::camExit() {
  cout << "camExit" << endl;
  ExitCamera_wr();
}

void UEyeCamera::camGrab() {
  img[cid] = NULL;
  imgBuffNum[cid] = 0;

  WaitForNextImage_wr();
  GetImageInfo_wr();

  /*
  char s[100];
  sprintf(s, "%02d-%02d-%02d--%02d-%02d-%02d-%03d",
    imgInfo[cid].TimestampSystem.wYear - 2000,
    imgInfo[cid].TimestampSystem.wMonth,
    imgInfo[cid].TimestampSystem.wDay,
    imgInfo[cid].TimestampSystem.wHour,
    imgInfo[cid].TimestampSystem.wMinute,
    imgInfo[cid].TimestampSystem.wSecond,
    imgInfo[cid].TimestampSystem.wMilliseconds);
  //cout << s << endl;
  */

  UnlockSeqBuf_wr(imgBuffNum[cid], img[cid]);
}

void UEyeCamera::camProcess() {
  cout << "Starting camProcess" << endl;
  emit started();

  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  for(bool quit = false; !quit; ) {
    for(cid = 0; cid < nUsedCams; cid++)
      camGrab();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  cout << "Finishing camProcess" << endl;
  emit finished();
}

void UEyeCamera::quit() {
  cout << "quit()" << endl;
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

void UEyeCamera::InitCamera_wr() {
  camStatus = is_InitCamera(&camID[cid], NULL);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "InitCamera() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID[cid], mode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetColorMode() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID[cid], ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetColorConverter() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID[cid], Mode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetDisplayMode() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID[cid], nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetExternalTrigger() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID[cid], &camInfo[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetSensorInfo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID[cid], width, height, bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "AllocImageMem() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "FreeImageMem() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ClearSequence() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "AddToSequence() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "PixelClock() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID[cid], fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetFrameRate() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "Exposure() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "CaptureVideo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID[cid], 0);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "InitImageQueue() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ExitImageQueue() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::WaitForNextImage_wr() {
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
  cout << "WaitForNextImage() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::CaptureStatus_wr(UINT nCommand) {
  camStatus = is_CaptureStatus(camID[cid], nCommand, (void*)&captInfo[cid], sizeof(captInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  cout << "CaptureStatus() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID[cid], imgBuffNum[cid], &imgInfo[cid], sizeof(imgInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetImageInfo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID[cid], buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "UnlockSeqBuf() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID[cid], &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetFramesPerSecond() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "StopLiveVideo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ExitCamera() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID[cid], &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    cout << (const char *)m << endl;
    return;
  }
  cout << "GetError() failed" << endl;
  handleCamStatus();
}

#define _UEYE_ERRCASE(X) case X: cout << #X << endl; break;

void UEyeCamera::handleCamStatus() {
  switch(camStatus) {
    case IS_NO_SUCCESS:
      GetError_wr(); break;
    _UEYE_ERRCASE(IS_ALL_DEVICES_BUSY)
    _UEYE_ERRCASE(IS_BAD_STRUCTURE_SIZE)
    _UEYE_ERRCASE(IS_CANT_ADD_TO_SEQUENCE)
    _UEYE_ERRCASE(IS_CANT_CLEANUP_MEMORY)
    _UEYE_ERRCASE(IS_CANT_COMMUNICATE_WITH_DRIVER)
    _UEYE_ERRCASE(IS_CANT_OPEN_DEVICE)
    _UEYE_ERRCASE(IS_CANT_OPEN_REGISTRY)
    _UEYE_ERRCASE(IS_CANT_READ_REGISTRY)
    _UEYE_ERRCASE(IS_CAPTURE_RUNNING)
    _UEYE_ERRCASE(IS_CRC_ERROR)
    _UEYE_ERRCASE(IS_DEVICE_ALREADY_PAIRED)
    _UEYE_ERRCASE(IS_DEVICE_NOT_COMPATIBLE)
    _UEYE_ERRCASE(IS_DR_CANNOT_CREATE_SURFACE)
    _UEYE_ERRCASE(IS_DR_CANNOT_CREATE_TEXTURE)
    _UEYE_ERRCASE(IS_DR_CANNOT_CREATE_VERTEX_BUFFER)
    _UEYE_ERRCASE(IS_DR_DEVICE_OUT_OF_MEMORY)
    _UEYE_ERRCASE(IS_DR_LIBRARY_NOT_FOUND)
    _UEYE_ERRCASE(IS_ERROR_CPU_IDLE_STATES_CONFIGURATION)
    _UEYE_ERRCASE(IS_FILE_WRITE_OPEN_ERROR)
    _UEYE_ERRCASE(IS_INCOMPATIBLE_SETTING)
    _UEYE_ERRCASE(IS_INVALID_BUFFER_SIZE)
    _UEYE_ERRCASE(IS_INVALID_CAMERA_TYPE)
    _UEYE_ERRCASE(IS_INVALID_CAMERA_HANDLE) // equal to IS_INVALID_HANDLE
    _UEYE_ERRCASE(IS_INVALID_CAPTURE_MODE)
    _UEYE_ERRCASE(IS_INVALID_COLOR_FORMAT)
    _UEYE_ERRCASE(IS_INVALID_DEVICE_ID)
    _UEYE_ERRCASE(IS_INVALID_EXPOSURE_TIME)
    _UEYE_ERRCASE(IS_INVALID_IP_CONFIGURATION)
    _UEYE_ERRCASE(IS_INVALID_MEMORY_POINTER)
    _UEYE_ERRCASE(IS_INVALID_MODE)
    _UEYE_ERRCASE(IS_INVALID_PARAMETER)
    _UEYE_ERRCASE(IS_INVALID_PIXEL_CLOCK)
    _UEYE_ERRCASE(IS_IO_REQUEST_FAILED)
    _UEYE_ERRCASE(IS_NETWORK_CONFIGURATION_INVALID)
    _UEYE_ERRCASE(IS_NETWORK_FRAME_SIZE_INCOMPATIBLE)
    _UEYE_ERRCASE(IS_NO_ACTIVE_IMG_MEM)
    _UEYE_ERRCASE(IS_NO_IMAGE_MEM_ALLOCATED)
    _UEYE_ERRCASE(IS_NO_IR_FILTER)
    _UEYE_ERRCASE(IS_NOT_CALIBRATED)
    _UEYE_ERRCASE(IS_NOT_SUPPORTED)
    _UEYE_ERRCASE(IS_NULL_POINTER)
    _UEYE_ERRCASE(IS_OUT_OF_MEMORY)
    _UEYE_ERRCASE(IS_SEQUENCE_BUF_ALREADY_LOCKED)
    _UEYE_ERRCASE(IS_SEQUENCE_LIST_EMPTY)
    _UEYE_ERRCASE(IS_STARTER_FW_UPLOAD_NEEDED)
    _UEYE_ERRCASE(IS_SUBNET_MISMATCH)
    _UEYE_ERRCASE(IS_SUBNETMASK_MISMATCH)
    _UEYE_ERRCASE(IS_TIMED_OUT)
    _UEYE_ERRCASE(IS_TRIGGER_ACTIVATED)
    default:
      cout << "error - unhandled camStatus" << endl;
  }
  err_flag = true;
}

#define _UEYE_ERRIF(X) if(captInfo[cid].adwCapStatusCnt_Detail[X]) \
  cout << "CaptureStatus: " << captInfo[cid].adwCapStatusCnt_Detail[X] << " of " << #X << endl;

void UEyeCamera::handleCaptStatus() {
  cout << "CaptureStatus: " << captInfo[cid].dwCapStatusCnt_Total << " elements" << endl;
  _UEYE_ERRIF(IS_CAP_STATUS_API_NO_DEST_MEM)
  _UEYE_ERRIF(IS_CAP_STATUS_API_CONVERSION_FAILED)
  _UEYE_ERRIF(IS_CAP_STATUS_API_IMAGE_LOCKED)
  _UEYE_ERRIF(IS_CAP_STATUS_DRV_OUT_OF_BUFFERS)
  _UEYE_ERRIF(IS_CAP_STATUS_DRV_DEVICE_NOT_READY)
  _UEYE_ERRIF(IS_CAP_STATUS_USB_TRANSFER_FAILED)
  _UEYE_ERRIF(IS_CAP_STATUS_DEV_TIMEOUT)
  _UEYE_ERRIF(IS_CAP_STATUS_ETH_BUFFER_OVERRUN)
  _UEYE_ERRIF(IS_CAP_STATUS_ETH_MISSED_IMAGES)
}

int UEyeCamera::getWidth() {
  return width;
}

int UEyeCamera::getHeight() {
  return height;
}

int UEyeCamera::getFPS() {
  return fps;
}

bool UEyeCamera::getErrFlag() {
  return err_flag;
}

#include "ueyecamera_moc.cpp"

