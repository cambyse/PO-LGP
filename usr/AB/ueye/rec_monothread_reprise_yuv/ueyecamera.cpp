#include <iostream>
#include <QCoreApplication>
#include <cstdlib>
#include "ueyecamera.h"

using namespace std;

UEyeCamera::UEyeCamera(int w, int h, int f): width(w), height(h), fps(f) {
  image = NULL;

  recthread = NULL;
  recworker = NULL;
  recflag = false;

  nrecframes = 0;

  curr_frame = 0;
  nskipped_frames = 0;

  err_flag = false;

  setup_flag = false;
  init_flag = false;
  open_flag = false;

  ct.cycleStart();
}

UEyeCamera::~UEyeCamera() {
  // TODO free all resources,
  // TODO close and exit cams
}

int UEyeCamera::getNumCameras() {
  // TODO clean and check for errors here.
  INT numCams;
  INT status = is_GetNumberOfCameras(&numCams);
  if(status == IS_SUCCESS)
    return (int)numCams;

  cout << "UEyeCamera::getNumCameras() An error of some sort" << endl;
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
  if(!setup_flag || init_flag || open_flag)
    return;
  for(cid = 0; cid < nUsedCams; cid++)
    camInit();
  init_flag = true;
}

void UEyeCamera::open() {
  if(!setup_flag || !init_flag || open_flag)
    return;
  for(cid = 0; cid < nUsedCams; cid++)
    camOpen();
  open_flag = true;
}

void UEyeCamera::close() {
  if(!setup_flag || !init_flag || !open_flag)
    return true;
  for(cid = 0; cid < nUsedCams; cid++)
    camClose();
  open_flag = false;
}

void UEyeCamera::exit() {
  if(!setup_flag || !init_flag || open_flag)
    return;
  for(cid = 0; cid < nUsedCams; cid++)
    camExit(i);
  init_flag = false;
}

void UEyeCamera::setupCommon() {
  camID = new HIDS[nUsedCams];
  camInfo = new SENSORINFO[nUsedCams];

  img = new char*[nUsedCams];
  imgCopy = new char*[nUsedCams];
  imgBuffNum = new INT[nUsedCams];
  imgInfo = new UEYEIMAGEINFO[nUsedCams];
  captInfo = new UEYE_CAPTURE_STATUS_INFO[nUsedCams];

  camBuff = new char**[nUsedCams];
  camBuffID = new INT*[nUsedCams];

  setup_flag = true;
}

void UEyeCamera::camInit() {
  msg("camInit()");
  InitCamera_wr();

  //SetColorMode_wr(IS_CM_BGR8_PACKED);
  SetColorMode_wr(IS_CM_UYVY_PACKED);
  bpp = 16; // ONLY CHANGE THIS
  bypp = bpp/8;
  bypimg = bypp * width * height;

  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_SOFTWARE_5X5);
  SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_HARDWARE_3X3);
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_OPENCL_3X3);
  //SetColorConverter_wr(IS_CM_UYVY_PACKED, IS_CONV_MODE_OPENCL_5X5);

  // TODO optimize using opengl
  //SetDisplayMode_wr(IS_SET_DM_DIB); // Default anyway..

  SetExternalTrigger_wr(IS_SET_TRIGGER_OFF);
  //SetExternalTrigger_wr(IS_SET_TRIGGER_HI_LO);

  /*
  GetSensorInfo_wr();
  msg(STRING(" - sensor ID = " << camInfo.SensorID));
  msg(STRING(" - camera model = " << camInfo.strSensorName));
  msg(STRING(" - max width = " << camInfo.nMaxWidth));
  msg(STRING(" - max height = " << camInfo.nMaxHeight));
  msg(STRING(" - pixel size = " << (float)camInfo.wPixelSize/100 << " µm"));
  */

  numBuff = 9;
  camBuff[cid] = new char*[numBuff];
  camBuffID[cid] = new INT[numBuff];
  imgCopy[cid] = new char[bypimg];

  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++) {
    AllocImageMem_wr(&camBuff[cid][i], &camBuffID[cid][i]);
    AddToSequence_wr(camBuff[cid][i], camBuffID[cid][i]);
  }

  // SEPARATION

  UINT pr[3];
  memset(pr, 0, 3*sizeof(UINT));

  // query possible values
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET_RANGE, (void*)pr, sizeof(pr));
  msg(STRING(" - pixelclock range = " << pr[0] << ":" << pr[2] << ":" << pr[1]));

  // set value
  pixelclock = pr[1];
  PixelClock_wr(IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock));
  msg(STRING(" - set pixelclock = " << pixelclock));

  // check/read value
  PixelClock_wr(IS_PIXELCLOCK_CMD_GET, (void*)&pixelclock, sizeof(pixelclock));
  msg(STRING(" - real pixelclock = " << pixelclock));

  SetFrameRate_wr();
  msg(STRING(" - set fps = " << fps));
  msg(STRING(" - real fps = " << real_fps));

  double er[3];
  memset(er, 0, 3*sizeof(double));

  // query possible values
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, (void*)er, sizeof(er));
  msg(STRING(" - exposure range = " << er[0] << ":" << er[2] << ":" << er[1]));

  // set value
  exposure = er[1];
  Exposure_wr(IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  msg(STRING(" - set exposure = " << exposure));

  // check/read value
  Exposure_wr(IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&exposure, sizeof(exposure));
  msg(STRING(" - real exposure = " << exposure));
}

void UEyeCamera::camOpen() {
  msg("CaptureVideo()");
  CaptureVideo_wr(IS_DONT_WAIT);
}

void UEyeCamera::camClose() {
  if(recflag)
    stopRec();

  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[cid][i], camBuffID[cid][i]);
  delete camBuff[cid];
  delete camBuffID[cid];
  delete imgCopy[cid];

  camExit();
}

void UEyeCamera::camExit() {
  ExitCamera_wr();
}

void UEyeCamera::startRec() {
  recMutex.lock();
  if(!recflag && recworker == NULL) {
    MT::String nowStr, nameStr;
    MT::getNowString(nowStr);
    nameStr << "z." << nowStr << "." << name << ".avi";
    recworker = new RecWorker(nameStr, width, height, fps);
    recflag = true;

    recthread = new QThread();

    connect(recthread, SIGNAL(started()), recworker, SLOT(process()));
    //connect(recworker, SIGNAL(finished()), recthread, SLOT(quit()));

    //connect(recworker, SIGNAL(finished()), recworker, SLOT(deleteLater()));
    //connect(recthread, SIGNAL(finished()), recthread, SLOT(deleteLater()));

    //connect(this, SIGNAL(frame()), recworker, SLOT(processFrame()));

    recworker->moveToThread(recthread);
    recthread->start();
    nrecframes = 0;
  }

  nskipped_frames = 0;
  recMutex.unlock();
}

void UEyeCamera::stopRec() {
  recMutex.lock();
  recflag = false;

  recworker->quit();
  recworker->processBuffer();
  delete recworker;
  recworker = NULL;

  msg(STRING("Number of skipped frames = " << nskipped_frames));

  recMutex.unlock();
}

void UEyeCamera::grab() {
  imageBuffNum = 0; image = NULL;

  ct.cycleDone();
  msg(STRING("busyDt: " << ct.busyDt << " busyDtMax: " << ct.busyDtMax << " cyclDt: " << ct.cyclDt));

  WaitForNextImage_wr();
  GetImageInfo_wr();
    // TODO how to handle the time?
  char s[100];
  sprintf(s, "%02d-%02d-%02d--%02d-%02d-%02d-%03d",
    imgInfo.TimestampSystem.wYear - 2000,
    imgInfo.TimestampSystem.wMonth,
    imgInfo.TimestampSystem.wDay,
    imgInfo.TimestampSystem.wHour,
    imgInfo.TimestampSystem.wMinute,
    imgInfo.TimestampSystem.wSecond,
    imgInfo.TimestampSystem.wMilliseconds);
  msg(STRING(s));

  ct.cycleStart();

  /*
  MT::String m << "got " << imageBuffNum;
  if(imageBuffNum != (curr_frame%numBuff+1)) {
    m << " instead of " << (curr_frame%numBuff+1);
    nskipped_frames++;
  }
  msg(m);
  curr_frame = imageBuffNum;
  */

  imgMutex.lock();
  memcpy(image_copy, image, bypimg);
  imgMutex.unlock();

  recMutex.lock();
  if(recflag) {
    char *p = new char[bypimg];
    memcpy(p, image_copy, bypimg);
    recworker->bufferFrame(p);
  }
  nrecframes++;
  recMutex.unlock();
  
  UnlockSeqBuf_wr(imageBuffNum, image);

  /*
  GetFramesPerSecond_wr();
  if(live_fps < 55)
    msg(STRING("fps = " << live_fps));
  else
    msg("===============");
  */
}

void UEyeCamera::getImage(char *p) {
  imgMutex.lock();
  memcpy(p, image_copy, bypimg);
  imgMutex.unlock();
}

void UEyeCamera::process() {
  emit started();
  
  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  InitImageQueue_wr();
  for(bool quit = false; !quit; ) {
    grab();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }
  ExitImageQueue_wr();
  close();

  emit finished();
}

void UEyeCamera::quit() {
  msg("quit()");
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

void UEyeCamera::InitCamera_wr() {
  camStatus = is_InitCamera(&camID[cid], NULL);
  if(camStatus == IS_SUCCESS) {
    msg(STRING("video_" << camID[cid]));
    msg(STRING("-- camID " << camID[cid]));
    msg(STRING("-- name " << name[cid]));
    return;
  }
  msg("InitCamera() failed");
  handleCamStatus();
}

void UEyeCamera::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID[cid], mode);
  if(camStatus == IS_SUCCESS)
    return;
  msg("SetColorMode() failed");
  handleCamStatus();
}

void UEyeCamera::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID[cid], ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  msg("SetColorConverter() failed");
  handleCamStatus();
}

void UEyeCamera::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID[cid], Mode);
  if(camStatus == IS_SUCCESS)
    return;
  msg("SetDisplayMode() failed");
  handleCamStatus();
}

void UEyeCamera::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID[cid], nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  msg("SetExternalTrigger() failed");
  handleCamStatus();
}

void UEyeCamera::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID[cid], &camInfo);
  if(camStatus == IS_SUCCESS)
    return;
  msg("GetSensorInfo() failed");
  handleCamStatus();
}

void UEyeCamera::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID[cid], width, height, bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  msg("AllocImageMem() failed");
  handleCamStatus();
}

void UEyeCamera::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  msg("FreeImageMem() failed");
  handleCamStatus();
}

void UEyeCamera::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  msg("ClearSequence() failed");
  handleCamStatus();
}

void UEyeCamera::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  msg("AddToSequence() failed");
  handleCamStatus();
}

void UEyeCamera::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  msg("PixelClock() failed");
  handleCamStatus();
}

void UEyeCamera::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID[cid], fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  msg("SetFrameRate() failed");
  handleCamStatus();
}

void UEyeCamera::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  msg("Exposure() failed");
  handleCamStatus();
}

void UEyeCamera::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  msg("CaptureVideo() failed");
  handleCamStatus();
}

void UEyeCamera::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID[cid], 0);
  if(camStatus == IS_SUCCESS)
    return;
  msg("InitImageQueue() failed");
  handleCamStatus();
}

void UEyeCamera::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  msg("ExitImageQueue() failed");
  handleCamStatus();
}

void UEyeCamera::WaitForNextImage_wr() {
  camStatus = is_WaitForNextImage(camID[cid], 1<<31, &image, &imageBuffNum);
  switch(camStatus) {
    case IS_CAPTURE_STATUS:
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_GET);
      handleCaptStatus();
      CaptureStatus_wr(IS_CAPTURE_STATUS_INFO_CMD_RESET);
      WaitForNextImage_wr();
    case IS_SUCCESS:
      return;
  }
  msg("WaitForNextImage() failed");
  handleCamStatus();
}

void UEyeCamera::CaptureStatus_wr(UINT nCommand) {
  camStatus = is_CaptureStatus(camID[cid], nCommand, (void*)&captInfo, sizeof(captInfo));
  if(camStatus == IS_SUCCESS)
    return;
  msg("CaptureStatus() failed");
  handleCamStatus();
}

void UEyeCamera::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID[cid], imageBuffNum, &imgInfo, sizeof(imgInfo));
  if(camStatus == IS_SUCCESS)
    return;
  msg("GetImageInfo() failed");
  handleCamStatus();
}

void UEyeCamera::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID[cid], buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  msg("UnlockSeqBuf() failed");
  handleCamStatus();
}

void UEyeCamera::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID[cid], &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  msg("GetFramesPerSecond() failed");
  handleCamStatus();
}

void UEyeCamera::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  msg("StopLiveVideo() failed");
  handleCamStatus();
}

void UEyeCamera::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return
  msg("ExitCamera() failed");
  handleCamStatus();
}

void UEyeCamera::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID[cid], &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    msg((const char *)m);
    return;
  }
  msg("GetError() failed");
  handleCamStatus();
}

#define _UEYE_ERRCASE(XXX) case XXX: msg(#XXX); break;

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
      msg(NULL);
  }
  // TODO fix.. QCore loop not started yet, so quitting doesn't work.
  err_flag = true;
  QCoreApplication::quit();
}

#define _UEYE_ERRIF(XXX) if(captInfo.adwCapStatusCnt_Detail[XXX]) \
  msg(STRING("CaptureStatus: " << captInfo.adwCapStatusCnt_Detail[XXX] << " of " << #XXX));

void UEyeCamera::handleCaptStatus() {
  msg(STRING("CaptureStatus: " << captInfo.dwCapStatusCnt_Total << " elements"));
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

void UEyeCamera::msg(const char *m) {
  if(m == NULL) {
    msg("no message");
    return;
  }
  cout << "UEyeCamera(" << camID[cid] << ") - " << m << endl;
}

void UEyeCamera::msg(const MT::String &m) {
  if(m.N == 0) {
    msg("no message");
    return;
  }
  cout << "UEyeCamera(" << camID[cid] << ") - " << m << endl;
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

MT::String UEyeCamera::getName() {
  return name;
}

bool UEyeCamera::getErrFlag() {
  return err_flag;
}

#include "ueyecamera_moc.cpp"

