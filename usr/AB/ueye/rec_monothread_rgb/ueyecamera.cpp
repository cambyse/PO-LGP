#include <QCoreApplication>
#include <cstdlib>
#include "ueyecamera.h"

UEyeCamera::UEyeCamera(int w, int h, int f): width(w), height(h), fps(f) {
  img = NULL;

  recthread = NULL;
  recworker = NULL;
  recflag = false;

  err_flag = false;

  setup_flag = false;
  init_flag = false;
  open_flag = false;

  throut::throutRegHeading(this, "UEyeCamera(*): ");

  ct.reset();
}

UEyeCamera::~UEyeCamera() {
  for(int c = 0; c < nUsedCams; c++)
    throut::throutUnregHeading(&camID[c]);
  throut::throutUnregHeading(this);

  delete[] camID;
  delete[] camInfo;

  for(int c = 0; c < nUsedCams; c++) {
    delete name(c);
    delete[] imgCopy[c];
    delete[] camBuff[c];
    delete[] camBuffID[c];
  }

  delete[] img;
  delete[] imgCopy;
  delete[] imgBuffNum;
  delete[] imgInfo;
  delete[] captInfo;

  delete[] camBuff;
  delete[] camBuffID;

  delete[] recworker;
  delete[] recthread;
}

int UEyeCamera::getNumCameras() {
  // TODO clean and check for errors here.
  INT numCams;
  INT status = is_GetNumberOfCameras(&numCams);
  if(status == IS_SUCCESS)
    return (int)numCams;

  throut::throut("UEyeCamera(*): getNumCameras() An error of some sort");
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
  name.resize(nUsedCams);

  img = new char*[nUsedCams];
  imgCopy = new char*[nUsedCams];
  imgBuffNum = new INT[nUsedCams];
  imgInfo = new UEYEIMAGEINFO[nUsedCams];
  captInfo = new UEYE_CAPTURE_STATUS_INFO[nUsedCams];

  camBuff = new char**[nUsedCams];
  camBuffID = new INT*[nUsedCams];

  recthread = new QThread*[nUsedCams];
  recworker = new RecWorker*[nUsedCams];

  setup_flag = true;
}

void UEyeCamera::camInit() {
  throut::throut(this, "camInit()");
  InitCamera_wr();
  if(err_flag) return;

  throut::throutRegHeading(&camID[cid], STRING("UEyeCamera(" << cid << "): "));

  name(cid) = new String(STRING("Video_" << camID[cid]));
  throut::throut(&camID[cid], STRING("- camID = " << camID[cid]));
  throut::throut(&camID[cid], STRING("- name = " << *name(cid)));

  SetColorMode_wr(IS_CM_BGR8_PACKED);
  if(err_flag) return;
  bpp = 24;
  bypp = bpp/8;
  bypimg = bypp * width * height;

  SetColorConverter_wr(IS_CM_BGR8_PACKED, IS_CONV_MODE_SOFTWARE_3X3);
  if(err_flag) return;

  // TODO optimize using opengl
  //SetDisplayMode_wr(IS_SET_DM_DIB); // Default anyway..

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
  imgCopy[cid] = new char[bypimg];

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
  throut::throut(&camID[cid], STRING("- set fps = " << fps));
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

void UEyeCamera::camOpen() {
  throut::throut(&camID[cid], "camOpen()");
  CaptureVideo_wr(IS_WAIT);//DONT_WAIT);
  if(err_flag) return;
  InitImageQueue_wr();
  if(err_flag) return;
}

void UEyeCamera::camClose() {
  throut::throut(&camID[cid], "camClose()");
  stopRec();

  ExitImageQueue_wr();
  StopLiveVideo_wr(IS_WAIT);
  
  ClearSequence_wr();
  for(int i = 0; i < numBuff; i++)
    FreeImageMem_wr(camBuff[cid][i], camBuffID[cid][i]);
}

void UEyeCamera::camExit() {
  throut::throut(&camID[cid], "camExit()");
  ExitCamera_wr();
}

void UEyeCamera::startRec() {
  recMutex.lock();
  if(!recflag) {
    MT::String nowStr;
    MT::getNowString(nowStr);

    for(int c = 0; c < nUsedCams; c++) {
      MT::String nameStr(STRING("z." << nowStr << "." << (*name(c)) << ".avi"));
      MT::String timeStr(STRING("z." << nowStr << ".Times" << (*name(c)) << ".dat"));

      recworker[c] = new RecWorker(nameStr, timeStr, width, height, fps);
      recthread[c] = new QThread();

      connect(recthread[c], SIGNAL(started()), recworker[c], SLOT(process()));
      connect(recworker[c], SIGNAL(finished()), recthread[c], SLOT(quit()));
      
      // just inserted
      connect(recworker[c], SIGNAL(finished()), recworker[c], SLOT(deleteLater()));
      connect(recthread[c], SIGNAL(finished()), recthread[c], SLOT(deleteLater()));

      recworker[c]->moveToThread(recthread[c]);
    }
    for(int c = 0; c < nUsedCams; c++)
      recthread[c]->start();

    recflag = true;
  }
  recMutex.unlock();
}

void UEyeCamera::stopRec() {
  recMutex.lock();
  if(recflag) {
    for(int c = 0; c < nUsedCams; c++)
      recworker[c]->quit();

    /*
    for(int c = 0; c < nUsedCams; c++)
      recworker[c]->processBuffer();

    for(int c = 0; c < nUsedCams; c++) {
      recworker[c]->deleteLater();
      recworker[c] = NULL;
    }
    */
    recflag = false;
  }
  recMutex.unlock();
}

void UEyeCamera::camGrab() {
  img[cid] = NULL;
  imgBuffNum[cid] = 0;

  //ct.cycleDone();
  //throut::throut(&camID[cid], STRING("busyDt: " << ct.busyDt << " cyclDt: " << ct.cyclDt));

  WaitForNextImage_wr();
  //ct.cycleStart();

  imgMutex.lock();
  memcpy(imgCopy[cid], img[cid], bypimg);
  imgMutex.unlock();

  recMutex.lock();
  if(recflag) {
    char *p = new char[bypimg];
    memcpy(p, imgCopy[cid], bypimg);

    GetImageInfo_wr();
    char *s = getTimeStamp();
    //throut::throut(&camID[cid], STRING("timestamp = " << s));
    
    recworker[cid]->bufferFrame(p, s);
  }
  recMutex.unlock();
  
  //throut::throut(&camID[cid], STRING("unlocking imgBuffNum = " << imgBuffNum[cid]));
  UnlockSeqBuf_wr(imgBuffNum[cid], img[cid]);

  /*
  GetFramesPerSecond_wr();
  throut::throut(&camID[cid], STRING("fps = " << live_fps));
  */
}

char* UEyeCamera::getTimeStamp() {
  /*
  sec = 0-59
  min = 0-59
  hour = 0-23
  day = 1-31
  month = 0-11
  year = years since 1900
  yday = day of the year, 0-365
  */

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

  //sec + 60*min + 3600*hour + 86400*yday + (year-70)*31536000 + ((year-69)/4)*86400 - ((year-1)/100)*86400 + ((year+299)/400)*86400
  long int sec = s + 60*m + 3600*h + 86400*d + \
                  (y-70)*31536000 + ((y-69)/4)*86400 - \
                  ((y-1)/100)*86400 + ((y+299)/400)*86400;

  long int usec = 1000*imgInfo[cid].TimestampSystem.wMilliseconds;

  char *ts = new char[30];
  sprintf(ts, "%8li.%06li", sec, usec);

  return ts;
}

void UEyeCamera::queryImage(int c, char *p) {
  imgMutex.lock();
  memcpy(p, imgCopy[c], bypimg);
  imgMutex.unlock();
}

bool UEyeCamera::queryError() {
  return err_flag;
}

void UEyeCamera::camProcess() {
  emit started();
  
  quitMutex.lock();
  quit_flag = false;
  quitMutex.unlock();

  for(bool quit = false; !quit; ) {
    for(cid = 0; cid < nUsedCams; cid++) // TODO is this safe? who else can access to cid?
      camGrab();

    quitMutex.lock();
    quit = quit_flag;
    quitMutex.unlock();
  }

  close();
  exit();

  emit finished();
}

void UEyeCamera::quit() {
  throut::throut(this, "quit()");
  quitMutex.lock();
  quit_flag = true;
  quitMutex.unlock();
}

void UEyeCamera::InitCamera_wr() {
  camStatus = is_InitCamera(&camID[cid], NULL);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "InitCamera() failed");
  handleCamStatus();
}

void UEyeCamera::SetColorMode_wr(INT mode) {
  camStatus = is_SetColorMode(camID[cid], mode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetColorMode() failed");
  handleCamStatus();
}

void UEyeCamera::SetColorConverter_wr(INT ColorMode, INT ConvertMode) {
  camStatus = is_SetColorConverter(camID[cid], ColorMode, ConvertMode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetColorConverter() failed");
  handleCamStatus();
}

void UEyeCamera::SetDisplayMode_wr(INT Mode) {
  camStatus = is_SetDisplayMode(camID[cid], Mode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetDisplayMode() failed");
  handleCamStatus();
}

void UEyeCamera::SetExternalTrigger_wr(INT nTriggerMode) {
  camStatus = is_SetExternalTrigger(camID[cid], nTriggerMode);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetExternalTrigger() failed");
  handleCamStatus();
}

void UEyeCamera::GetSensorInfo_wr() {
  camStatus = is_GetSensorInfo(camID[cid], &camInfo[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetSensorInfo() failed");
  handleCamStatus();
}

void UEyeCamera::AllocImageMem_wr(char **buff, INT *buffID) {
  camStatus = is_AllocImageMem(camID[cid], width, height, bpp, buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "AllocImageMem() failed");
  handleCamStatus();
}

void UEyeCamera::FreeImageMem_wr(char *buff, INT buffID) {
  camStatus = is_FreeImageMem(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "FreeImageMem() failed");
  handleCamStatus();
}

void UEyeCamera::ClearSequence_wr() {
  camStatus = is_ClearSequence(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ClearSequence() failed");
  handleCamStatus();
}

void UEyeCamera::AddToSequence_wr(char *buff, INT buffID) {
  camStatus = is_AddToSequence(camID[cid], buff, buffID);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "AddToSequence() failed");
  handleCamStatus();
}

void UEyeCamera::PixelClock_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_PixelClock(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "PixelClock() failed");
  handleCamStatus();
}

void UEyeCamera::SetFrameRate_wr() {
  camStatus = is_SetFrameRate(camID[cid], fps, &real_fps);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "SetFrameRate() failed");
  handleCamStatus();
}

void UEyeCamera::Exposure_wr(UINT nCommand, void *pParam, UINT cbSizeOfParam) {
  camStatus = is_Exposure(camID[cid], nCommand, pParam, cbSizeOfParam);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "Exposure() failed");
  handleCamStatus();
}

void UEyeCamera::CaptureVideo_wr(INT wait) {
  camStatus = is_CaptureVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "CaptureVideo() failed");
  handleCamStatus();
}

void UEyeCamera::InitImageQueue_wr() {
  camStatus = is_InitImageQueue(camID[cid], 0);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "InitImageQueue() failed");
  handleCamStatus();
}

void UEyeCamera::ExitImageQueue_wr() {
  camStatus = is_ExitImageQueue(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ExitImageQueue() failed");
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
  throut::throut(&camID[cid], "WaitForNextImage() failed");
  handleCamStatus();
}

void UEyeCamera::CaptureStatus_wr(UINT nCommand) {
  camStatus = is_CaptureStatus(camID[cid], nCommand, (void*)&captInfo[cid], sizeof(captInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "CaptureStatus() failed");
  handleCamStatus();
}

void UEyeCamera::GetImageInfo_wr() {
  camStatus = is_GetImageInfo(camID[cid], imgBuffNum[cid], &imgInfo[cid], sizeof(imgInfo[cid]));
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetImageInfo() failed");
  handleCamStatus();
}

void UEyeCamera::UnlockSeqBuf_wr(INT buffID, char *buff) {
  camStatus = is_UnlockSeqBuf(camID[cid], buffID, buff);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "UnlockSeqBuf() failed");
  handleCamStatus();
}

void UEyeCamera::GetFramesPerSecond_wr() {
  camStatus = is_GetFramesPerSecond(camID[cid], &live_fps);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "GetFramesPerSecond() failed");
  handleCamStatus();
}

void UEyeCamera::StopLiveVideo_wr(INT wait) {
  camStatus = is_StopLiveVideo(camID[cid], wait);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "StopLiveVideo() failed");
  handleCamStatus();
}

void UEyeCamera::ExitCamera_wr() {
  camStatus = is_ExitCamera(camID[cid]);
  if(camStatus == IS_SUCCESS)
    return;
  throut::throut(&camID[cid], "ExitCamera() failed");
  handleCamStatus();
}

void UEyeCamera::GetError_wr() {
  IS_CHAR *m;
  camStatus = is_GetError(camID[cid], &camStatus, &m);
  if(camStatus == IS_SUCCESS) {
    throut::throut(&camID[cid], (const char *)m);
    return;
  }
  throut::throut(&camID[cid], "GetError() failed");
  handleCamStatus();
}

#define _UEYE_ERRCASE(X) case X: throut::throut(&camID[cid], #X); break;

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
      throut::throut(&camID[cid], "error - unhandled camStatus");
  }
  // TODO fix.. QCore loop not started yet, so quitting doesn't work.
  err_flag = true;
  QCoreApplication::quit();
}

#define _UEYE_ERRIF(X) if(captInfo[cid].adwCapStatusCnt_Detail[X]) \
  throut::throut(&camID[cid], STRING("CaptureStatus: " << captInfo[cid].adwCapStatusCnt_Detail[X] << " of " << #X));

void UEyeCamera::handleCaptStatus() {
  if(captInfo[cid].dwCapStatusCnt_Total > 0) {
    throut::throut(&camID[cid], STRING("CaptureStatus: " << captInfo[cid].dwCapStatusCnt_Total << " elements"));
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

