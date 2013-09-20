#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int cid, int w, int h, int f):
                camID(HIDS(cid)), width(w), height(h), fps(f) {
  // remove eventually. Gets any available camera.
  camID = 0;

  image = NULL;
  err_flag = false;

  sFirst[0] = '\0';
  sLast[0] = '\0';
}

void UEyeCamera::camInit() {
  cout << "InitCamera()" << endl;
  InitCamera_wr();

  //SetColorMode_wr(IS_CM_BGR8_PACKED);
  SetColorMode_wr(IS_CM_UYVY_PACKED);
  bpp = 16; // ONLY CHANGE THIS
  bypp = bpp/8;
  bypimg = bypp * width * height;

  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_SOFTWARE_5X5);
  SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_HARDWARE_3X3); // TODO fails
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_OPENCL_3X3); // TODO fails
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_OPENCL_5X5); // TODO fails

  //SetDisplayMode_wr(IS_SET_DM_DIB); // Default anyway..

  SetExternalTrigger_wr(IS_SET_TRIGGER_OFF);
  //SetExternalTrigger_wr(IS_SET_TRIGGER_HI_LO);

  numBuff = 9;
  camBuff = new char*[numBuff];
  camBuffID = new INT[numBuff];

  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++) {
    AllocImageMem_wr(&camBuff[i], &camBuffID[i]);
    AddToSequence_wr(camBuff[i], camBuffID[i]);
  }

  // SEPARATION

  UINT pr[3];
  memset(pr, 0, 3*sizeof(UINT));

  // query possible values
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pr, sizeof(pr));
  cout << " - pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1] << endl;

  // set value
  pixelclock = pr[1];
  PixelClock_wr(IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  cout << " - set pixelclock = " << pixelclock << endl;

  // check/read value
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET, (void*)&pixelclock, sizeof(pixelclock));
  cout << " - real pixelclock = " << pixelclock << endl;

  SetFrameRate_wr();
  cout << " - set fps = " << fps << endl;
  cout << " - real fps = " << real_fps << endl;

  double er[3];
  memset(er, 0, 3*sizeof(double));

  // query possible values
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)er, sizeof(er));
  cout << " - exposure range = " << er[0] << ":" << er[2] << ":" << er[1] << endl;

  // set value
  exposure = er[1];
  Exposure_wr(IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  cout << " - set exposure = " << exposure << endl;

  // check/read value
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  cout << " - real exposure = " << exposure << endl;
}

void UEyeCamera::open() {
  cout << "CaptureVideo()" << endl;
  CaptureVideo_wr(IS_DONT_WAIT);
}

void UEyeCamera::close() {
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[i], camBuffID[i]);
  delete camBuff;
  delete camBuffID;
}

void UEyeCamera::camExit() {
  ExitCamera_wr();
}

void UEyeCamera::grab() {
  imageBuffNum = 0; image = NULL;

  cout << "=====================================" << endl;
  WaitForNextImage_wr();
  GetImageInfo_wr();

  sprintf(sLast, "%02d-%02d-%02d--%02d-%02d-%02d-%03d",
    imgInfo.TimestampSystem.wYear - 2000,
    imgInfo.TimestampSystem.wMonth,
    imgInfo.TimestampSystem.wDay,
    imgInfo.TimestampSystem.wHour,
    imgInfo.TimestampSystem.wMinute,
    imgInfo.TimestampSystem.wSecond,
    imgInfo.TimestampSystem.wMilliseconds);
  if(sFirst[0] == '\0') {
    memcpy(sFirst, sLast, 30);
  }
  cout << sLast << endl;
  
  UnlockSeqBuf_wr(imageBuffNum, image);

  GetFramesPerSecond_wr();
  cout << "fps = " << live_fps << endl;
}

void UEyeCamera::InitCamera_wr() {
  camStatus = is_InitCamera(&camID, NULL);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "InitCamera() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID, mode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetColorMode() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID, ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetColorConverter() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID, Mode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetDisplayMode() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID, nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetExternalTrigger() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID, &camInfo);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetSensorInfo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID, width, height, bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "AllocImageMem() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "FreeImageMem() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ClearSequence() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "AddToSequence() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID, nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "PixelClock() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID, fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "SetFrameRate() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID, nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "Exposure() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID, wait);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "CaptureVideo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID, 0);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "InitImageQueue() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ExitImageQueue() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::WaitForNextImage_wr() {
  camStatus = is_WaitForNextImage(camID, 1<<31, &image, &imageBuffNum);
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
  camStatus = is_CaptureStatus(camID, nCommand, (void*)&captInfo, sizeof(captInfo));
  if(camStatus == IS_SUCCESS)
    return;
  cout << "CaptureStatus() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID, imageBuffNum, &imgInfo, sizeof(imgInfo));
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetImageInfo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID, buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "UnlockSeqBuf() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID, &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "GetFramesPerSecond() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID, wait);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "StopLiveVideo() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID);
  if(camStatus == IS_SUCCESS)
    return;
  cout << "ExitCamera() failed" << endl;
  handleCamStatus();
}

void UEyeCamera::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID, &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    cout << (const char *)m << endl;
    return;
  }
  cout << "GetError() failed" << endl;
  handleCamStatus();
}

#define _UEYE_ERRCASE(XXX) case XXX: cout << #XXX << endl;

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
      cout << "UNDEFINED VALUE" << endl;
  }
  err_flag = true;
}

#define _UEYE_ERRIF(XXX) if(captInfo.adwCapStatusCnt_Detail[XXX]) \
  cout << "CaptureStatus: " << captInfo.adwCapStatusCnt_Detail[XXX] << " of " << #XXX << endl;

void UEyeCamera::handleCaptStatus() {
  cout << "CaptureStatus: " << captInfo.dwCapStatusCnt_Total << " elements" << endl;
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

